// SPDX-License-Identifier: GPL-2.0
/*
 * v4h-cam-aggregator: one VLC-readable capture node for the R-Car V4H camera.
 *
 * VIN0 delivers raw Bayer frames and the ISP core is a memory-to-memory block
 * (input1, params, stats, output1) that libcamera normally drives from
 * userspace. This driver runs that loop in the kernel and exposes the
 * debayered result as one plain single-plane capture node (BGR32), which VLC
 * (single-plane V4L2_CAP_VIDEO_CAPTURE only) can open.
 *
 * There is one node per camera connector the device tree enables: J1 (CSI40,
 * ISP0, VIN0..7) is /dev/video21 and J2 (CSI41, ISP1, VIN8..15) is
 * /dev/video22 (video_nr and video_nr + 1).
 *
 * Everything needed to get a picture happens when the node starts streaming:
 *
 *  - the camera is found in the media graph (ISP <- CSI-2 <- sensor); Raspberry
 *    Pi Camera v2 (imx219) and v3 (imx708) are known, the Bayer order is read
 *    from the sensor and the smallest sensor mode not below min_w x min_h is
 *    used (imx219: 1920x1080, imx708: 2304x1296);
 *  - the media pad formats (sensor, CSI-2, ISP) and the video formats of VIN
 *    and of the ISP input1/output1 are set here, nothing is needed from
 *    media-ctl or v4l2-ctl;
 *  - sensor exposure and gain (AE) and the white balance gains (AWB) are
 *    regulated from a coarse sample of the raw frame, so the picture follows
 *    the light without any userspace 3A.
 *
 * It does not modify rcar-vin or rcar-isp: the five vb2 queues are found
 * through the video_device children of the two platform devices and are
 * driven with the exported vb2 helpers. While the node streams the queues are
 * owned by this driver, so other users (libcamera, v4l2-ctl) get -EBUSY.
 */
#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/rkisp1-config.h>
#include <linux/media/v4l2-isp.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define AGG_NSET	4		/* ISP jobs in flight */
#define AGG_DEF_W	1920		/* node size until a sensor is found */
#define AGG_DEF_H	1080

#define AGG_GRID_X	48		/* raw sample grid used by AE/AWB */
#define AGG_GRID_Y	27
#define AGG_BLACK10	64		/* black level, 10 bit (imx219, imx708) */
#define AGG_AE_TARGET	200		/* wanted mean of green, 10 bit, black removed */
#define AGG_SAT10	1000		/* above this a raw sample is clipped */

static int video_nr = 21;
module_param(video_nr, int, 0444);
MODULE_PARM_DESC(video_nr, "number of the capture node of J1 (/dev/videoN), J2 gets the next one, -1 = automatic");

static int min_w = AGG_DEF_W;
module_param(min_w, int, 0444);
MODULE_PARM_DESC(min_w, "use the smallest sensor mode that is at least this wide");

static int min_h = AGG_DEF_H;
module_param(min_h, int, 0444);
MODULE_PARM_DESC(min_h, "use the smallest sensor mode that is at least this high");

static int max_fps = 30;
module_param(max_fps, int, 0644);
MODULE_PARM_DESC(max_fps, "frames per second delivered; faster sensor frames are dropped (0 = all)");

static bool auto_ae = true;
module_param(auto_ae, bool, 0644);
MODULE_PARM_DESC(auto_ae, "regulate sensor exposure and gain from the picture");

static bool auto_awb = true;
module_param(auto_awb, bool, 0644);
MODULE_PARM_DESC(auto_awb, "regulate the white balance gains from the picture");

/* Used when the matching auto_* switch is off. */
static int awb_red = 340;
module_param(awb_red, int, 0644);
MODULE_PARM_DESC(awb_red, "manual white balance gain red, 256 = 1.0 (range 0..1023)");
static int awb_green = 256;
module_param(awb_green, int, 0644);
MODULE_PARM_DESC(awb_green, "white balance gain green, 256 = 1.0");
static int awb_blue = 385;
module_param(awb_blue, int, 0644);
MODULE_PARM_DESC(awb_blue, "manual white balance gain blue, 256 = 1.0");
static int black = 256;
module_param(black, int, 0644);
MODULE_PARM_DESC(black, "black level subtracted from every colour, 12-bit units (imx219, imx708: 256)");
static int expo;
module_param(expo, int, 0644);
MODULE_PARM_DESC(expo, "first (or, with auto_ae off, fixed) sensor exposure, 0 = sensor default");
static int gain;
module_param(gain, int, 0644);
MODULE_PARM_DESC(gain, "first (or, with auto_ae off, fixed) analogue gain code, 0 = sensor default");

static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "log every step (to find where a crash happens)");

#define DBG(fmt, ...) \
	do { if (debug) pr_info("v4h-cam-aggregator: " fmt "\n", ##__VA_ARGS__); } while (0)

/* sRGB gamma curve for the 17 point gamma-out block (10 bit). */
static const u16 agg_gamma[17] = {
	0, 284, 398, 481, 549, 608, 661, 709, 752,
	793, 831, 867, 901, 934, 965, 994, 1023
};

enum agg_q {
	Q_VIN,
	Q_IN1,
	Q_PAR,
	Q_STA,
	Q_OUT,
	Q_NUM
};

/* Nodes opened for real so that the open() of their driver powers the hardware. */
enum agg_node {
	N_VIN,
	N_IN1,
	N_OUT,
	N_NUM
};

enum agg_gain_model {
	GAIN_NONE,
	GAIN_IMX219,		/* gain = 256 / (256 - code) */
	GAIN_IMX708,		/* gain = 1024 / (1024 - code) */
};

struct agg_ext {
	struct video_device *vdev;
	struct vb2_queue *q;
	enum v4l2_buf_type type;
	bool mplane;
	bool owned;
	bool streaming;
	u32 bpl;
	u32 sizeimage;
	void *vaddr[AGG_NSET];
};

struct agg_cam {
	struct v4l2_subdev *sensor, *csi2, *isp;
	struct v4l2_ctrl *exp, *gain;
	enum agg_gain_model gm;
	u32 code;
	u32 fourcc;
	unsigned int bayer;	/* 0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR */
	unsigned int w, h;		/* sensor mode: sensor, CSI-2, ISP pad and VIN */
	unsigned int out_w, out_h;	/* picture size: ISP input1/output1 and the node */
	unsigned int crop_x, crop_y;	/* where the picture is cut from the sensor frame */
};

struct agg_buf {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

/* One capture node per camera connector: CSI-2 -> ISP -> VIN group. */
struct agg_port {
	const char *name;		/* connector on the board */
	const char *vin_pdev;		/* platform device of the first VIN of the ISP */
	const char *vin_vdev;		/* name of its video device */
	const char *isp_pdev;		/* platform device of the ISP */
};

struct agg {
	const struct agg_port *port;
	unsigned int idx;
	struct platform_device *pdev;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue queue;
	struct mutex lock;		/* vdev + queue */

	spinlock_t qlock;		/* buf_list */
	struct list_head buf_list;
	unsigned int sequence;
	unsigned long dropped;

	unsigned int w, h;		/* size of the node, from the sensor mode */
	unsigned int out_size;		/* w * h * 4 */
	struct agg_cam cam;

	/* 3A state, used by the frame thread only */
	int awb_r, awb_b;		/* 8.8 fixed point gains */
	unsigned int exp;		/* sensor exposure */
	unsigned int gain_q8;		/* analogue gain * 256 */
	unsigned int ae_iter, settle, fcnt;

	struct agg_ext ext[Q_NUM];
	struct file *node_file[N_NUM];
	struct task_struct *thread;
};

/* -------------------------------------------------------------------------
 * Finding the queues of VIN and ISP
 */

struct find_ctx {
	const char *name;
	struct video_device *vdev;
};

static int agg_match_vdev(struct device *dev, void *data)
{
	struct find_ctx *c = data;
	struct video_device *v;

	if (strncmp(dev_name(dev), "video", 5))
		return 0;

	v = container_of(dev, struct video_device, dev);
	if (strcmp(v->name, c->name))
		return 0;

	c->vdev = v;
	return 1;
}

static struct video_device *agg_find_vdev(const char *pdev_name, const char *vdev_name)
{
	struct find_ctx c = { .name = vdev_name };
	struct device *pdev = bus_find_device_by_name(&platform_bus_type, NULL, pdev_name);

	if (!pdev) {
		pr_err("v4h-cam-aggregator: platform device %s not found\n", pdev_name);
		return NULL;
	}
	device_for_each_child(pdev, &c, agg_match_vdev);
	put_device(pdev);

	if (!c.vdev || !c.vdev->queue) {
		pr_err("v4h-cam-aggregator: video device '%s' not found\n", vdev_name);
		return NULL;
	}
	return c.vdev;
}

/* -------------------------------------------------------------------------
 * Driving an external queue
 */

static void agg_qlock(struct vb2_queue *q)
{
	if (q->lock)
		mutex_lock(q->lock);
}

static void agg_qunlock(struct vb2_queue *q)
{
	if (q->lock)
		mutex_unlock(q->lock);
}

static int agg_ext_reqbufs(struct agg *a, enum agg_q i, unsigned int count)
{
	struct agg_ext *e = &a->ext[i];
	struct v4l2_requestbuffers rb = {
		.count = count, .type = e->type, .memory = V4L2_MEMORY_MMAP,
	};
	int ret;

	agg_qlock(e->q);
	ret = vb2_reqbufs(e->q, &rb);
	agg_qunlock(e->q);
	if (ret)
		return ret;
	if (count && rb.count < count)
		return -ENOMEM;
	return 0;
}

static int agg_ext_qbuf(struct agg *a, enum agg_q i, unsigned int idx, unsigned int bytesused)
{
	struct agg_ext *e = &a->ext[i];
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = { };
	struct v4l2_buffer b = { .type = e->type, .memory = V4L2_MEMORY_MMAP, .index = idx };
	int ret;

	if (e->mplane) {
		planes[0].bytesused = bytesused;
		b.m.planes = planes;
		b.length = 1;
	} else {
		b.bytesused = bytesused;
	}
	agg_qlock(e->q);
	ret = vb2_qbuf(e->q, NULL, &b);
	agg_qunlock(e->q);
	return ret;
}

/* Returns the index of a dequeued buffer, or a negative error. */
static int agg_ext_dqbuf(struct agg *a, enum agg_q i, unsigned int timeout_ms)
{
	struct agg_ext *e = &a->ext[i];
	unsigned long end = jiffies + msecs_to_jiffies(timeout_ms);

	do {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = { };
		struct v4l2_buffer b = { .type = e->type, .memory = V4L2_MEMORY_MMAP };
		int ret;

		if (e->mplane) {
			b.m.planes = planes;
			b.length = 1;
		}
		agg_qlock(e->q);
		ret = vb2_dqbuf(e->q, &b, true);
		agg_qunlock(e->q);
		if (!ret)
			return b.index;
		if (ret != -EAGAIN)
			return ret;
		if (kthread_should_stop())
			return -EINTR;
		usleep_range(1000, 2000);
	} while (time_before(jiffies, end));

	return -ETIMEDOUT;
}

static int agg_ext_streamon(struct agg *a, enum agg_q i)
{
	struct agg_ext *e = &a->ext[i];
	int ret;

	agg_qlock(e->q);
	ret = vb2_streamon(e->q, e->type);
	agg_qunlock(e->q);
	if (!ret)
		e->streaming = true;
	return ret;
}

static void agg_ext_release(struct agg *a)
{
	/* ISP queues first, then VIN. */
	static const enum agg_q order[Q_NUM] = { Q_IN1, Q_PAR, Q_STA, Q_OUT, Q_VIN };

	for (unsigned int n = 0; n < Q_NUM; n++) {
		struct agg_ext *e = &a->ext[order[n]];

		if (!e->q)
			continue;
		if (e->streaming) {
			agg_qlock(e->q);
			vb2_streamoff(e->q, e->type);
			agg_qunlock(e->q);
			e->streaming = false;
		}
		if (e->owned) {
			agg_ext_reqbufs(a, order[n], 0);
			e->q->owner = NULL;
			e->owned = false;
		}
	}
}

static void agg_ext_close(struct agg *a)
{
	agg_ext_release(a);
	for (unsigned int n = 0; n < N_NUM; n++) {
		if (a->node_file[n]) {
			filp_close(a->node_file[n], NULL);
			a->node_file[n] = NULL;
		}
	}
}

/* -------------------------------------------------------------------------
 * Finding the camera and the mode to use
 */

static const struct {
	u32 code;
	u32 fourcc;
} agg_raw_fmt[] = {
	/* Index = Bayer order: RGGB, GRBG, GBRG, BGGR. */
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10 },
};

static bool agg_is_subdev(const struct media_pad *pad)
{
	return pad && is_media_entity_v4l2_subdev(pad->entity);
}

/*
 * ISP <- CSI-2 <- sensor, taken from the links of the media graph, then the
 * Bayer order and the mode of the sensor.
 */
static int agg_detect(struct agg *a)
{
	struct video_device *vv = agg_find_vdev(a->port->vin_pdev, a->port->vin_vdev);
	struct agg_cam c = { };
	struct media_device *mdev;
	struct media_entity *ent, *isp = NULL;
	struct media_pad *pad;
	struct v4l2_subdev_format fmt = { .which = V4L2_SUBDEV_FORMAT_ACTIVE };
	struct v4l2_subdev_frame_size_enum fse = { .which = V4L2_SUBDEV_FORMAT_ACTIVE };
	unsigned int best_area = 0, big_area = 0, bw = 0, bh = 0, gw = 0, gh = 0;
	char name[48];
	int ret;

	if (!vv)
		return -ENODEV;
	mdev = vv->v4l2_dev ? vv->v4l2_dev->mdev : NULL;
	if (!mdev)
		return -ENODEV;

	snprintf(name, sizeof(name), "rcar_isp %s", a->port->isp_pdev);
	mutex_lock(&mdev->graph_mutex);
	media_device_for_each_entity(ent, mdev) {
		if (is_media_entity_v4l2_subdev(ent) && !strcmp(ent->name, name)) {
			isp = ent;
			break;
		}
	}
	if (isp) {
		c.isp = media_entity_to_v4l2_subdev(isp);
		pad = media_pad_remote_pad_first(&isp->pads[0]);
		if (agg_is_subdev(pad) && !strncmp(pad->entity->name, "rcar_csi2", 9)) {
			c.csi2 = media_entity_to_v4l2_subdev(pad->entity);
			pad = media_pad_remote_pad_first(&pad->entity->pads[0]);
			if (agg_is_subdev(pad) && pad->entity->function == MEDIA_ENT_F_CAM_SENSOR)
				c.sensor = media_entity_to_v4l2_subdev(pad->entity);
		}
	}
	mutex_unlock(&mdev->graph_mutex);

	if (!c.sensor || !c.csi2 || !c.isp) {
		DBG("detect: no camera in the media graph (isp %p csi2 %p sensor %p)",
		    c.isp, c.csi2, c.sensor);
		return -ENODEV;
	}

	if (!strncmp(c.sensor->entity.name, "imx219", 6))
		c.gm = GAIN_IMX219;
	else if (!strncmp(c.sensor->entity.name, "imx708", 6))
		c.gm = GAIN_IMX708;

	/* The sensor decides the Bayer order (imx708 changes it with the flips). */
	ret = v4l2_subdev_call_state_active(c.sensor, pad, get_fmt, &fmt);
	if (ret)
		return ret;
	for (c.bayer = 0; c.bayer < ARRAY_SIZE(agg_raw_fmt); c.bayer++)
		if (agg_raw_fmt[c.bayer].code == fmt.format.code)
			break;
	if (c.bayer == ARRAY_SIZE(agg_raw_fmt)) {
		pr_err("v4h-cam-aggregator: sensor code 0x%04x is not a 10 bit Bayer format\n",
		       fmt.format.code);
		return -EINVAL;
	}
	c.code = agg_raw_fmt[c.bayer].code;
	c.fourcc = agg_raw_fmt[c.bayer].fourcc;

	/* Smallest mode not below min_w x min_h, else the largest one. */
	fse.code = c.code;
	for (fse.index = 0; !v4l2_subdev_call_state_active(c.sensor, pad, enum_frame_size, &fse);
	     fse.index++) {
		unsigned int w = fse.max_width, h = fse.max_height, area = w * h;

		if (w >= min_w && h >= min_h) {
			if (!best_area || area < best_area) {
				best_area = area;
				bw = w;
				bh = h;
			}
		} else if (area > big_area) {
			big_area = area;
			gw = w;
			gh = h;
		}
	}
	c.w = best_area ? bw : gw;
	c.h = best_area ? bh : gh;
	if (!c.w || !c.h)
		return -EINVAL;

	/*
	 * The ISP does not scale: when the sensor mode is bigger than min_w x
	 * min_h (imx708 has no 1920x1080 mode) the picture is cut from the middle
	 * of the frame. The offsets are even so the Bayer order is kept.
	 */
	c.out_w = min_t(unsigned int, c.w, max(min_w, 16)) & ~1U;
	c.out_h = min_t(unsigned int, c.h, max(min_h, 16)) & ~1U;
	c.crop_x = ((c.w - c.out_w) / 2) & ~1U;
	c.crop_y = ((c.h - c.out_h) / 2) & ~1U;

	a->cam = c;
	DBG("detect: %s, mode %ux%u, picture %ux%u at %u,%u, code 0x%04x, bayer %u",
	    c.sensor->entity.name, c.w, c.h, c.out_w, c.out_h, c.crop_x, c.crop_y,
	    c.code, c.bayer);
	return 0;
}

/* Take the node size from the sensor; the defaults stay when there is no camera. */
static void agg_refresh(struct agg *a)
{
	if (!agg_detect(a)) {
		a->w = a->cam.out_w;
		a->h = a->cam.out_h;
	} else {
		a->w = AGG_DEF_W;
		a->h = AGG_DEF_H;
	}
	a->out_size = a->w * a->h * 4;
}

/* -------------------------------------------------------------------------
 * Pipeline formats, set here instead of with media-ctl and v4l2-ctl
 */

static int agg_set_subdev_fmt(struct v4l2_subdev *sd, u32 code, unsigned int w, unsigned int h)
{
	struct v4l2_subdev_format f = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = 0,
		.format = { .width = w, .height = h, .code = code, .field = V4L2_FIELD_NONE },
	};

	return v4l2_subdev_call_state_active(sd, pad, set_fmt, &f);
}

/* S_FMT on a video node of VIN or of the ISP, with that node's own file. */
static int agg_s_fmt(struct agg *a, enum agg_q q, enum agg_node n, struct v4l2_format *f)
{
	struct video_device *vd = a->ext[q].vdev;
	struct file *file = a->node_file[n];
	const struct v4l2_ioctl_ops *ops = vd->ioctl_ops;
	int ret = -ENOTTY;

	if (vd->lock)
		mutex_lock(vd->lock);
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (ops->vidioc_s_fmt_vid_cap)
			ret = ops->vidioc_s_fmt_vid_cap(file, file->private_data, f);
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		if (ops->vidioc_s_fmt_vid_out_mplane)
			ret = ops->vidioc_s_fmt_vid_out_mplane(file, file->private_data, f);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		if (ops->vidioc_s_fmt_vid_cap_mplane)
			ret = ops->vidioc_s_fmt_vid_cap_mplane(file, file->private_data, f);
		break;
	default:
		break;
	}
	if (vd->lock)
		mutex_unlock(vd->lock);
	return ret;
}

static int agg_set_formats(struct agg *a)
{
	struct agg_cam *c = &a->cam;
	struct v4l2_format f = { };
	int ret;

	ret = agg_set_subdev_fmt(c->sensor, c->code, c->w, c->h);
	if (!ret)
		ret = agg_set_subdev_fmt(c->csi2, c->code, c->w, c->h);
	if (!ret)
		ret = agg_set_subdev_fmt(c->isp, c->code, c->w, c->h);
	if (ret) {
		pr_err("v4h-cam-aggregator: setting the media pad formats failed: %d\n", ret);
		return ret;
	}

	/* VIN: raw frames. */
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f.fmt.pix.width = c->w;
	f.fmt.pix.height = c->h;
	f.fmt.pix.pixelformat = c->fourcc;
	f.fmt.pix.field = V4L2_FIELD_NONE;
	ret = agg_s_fmt(a, Q_VIN, N_VIN, &f);
	if (ret || f.fmt.pix.width != c->w || f.fmt.pix.height != c->h ||
	    f.fmt.pix.pixelformat != c->fourcc) {
		pr_err("v4h-cam-aggregator: VIN format %ux%u failed (%d)\n", c->w, c->h, ret);
		return ret ? ret : -EPIPE;
	}
	a->ext[Q_VIN].bpl = f.fmt.pix.bytesperline;
	a->ext[Q_VIN].sizeimage = f.fmt.pix.sizeimage;

	/* ISP input1: the same raw frames. */
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f.fmt.pix_mp.width = c->out_w;
	f.fmt.pix_mp.height = c->out_h;
	f.fmt.pix_mp.pixelformat = c->fourcc;
	f.fmt.pix_mp.field = V4L2_FIELD_NONE;
	f.fmt.pix_mp.num_planes = 1;
	ret = agg_s_fmt(a, Q_IN1, N_IN1, &f);
	if (ret || f.fmt.pix_mp.width != c->out_w || f.fmt.pix_mp.height != c->out_h ||
	    f.fmt.pix_mp.pixelformat != c->fourcc) {
		pr_err("v4h-cam-aggregator: ISP input format %ux%u failed (%d)\n",
		       c->out_w, c->out_h, ret);
		return ret ? ret : -EPIPE;
	}
	a->ext[Q_IN1].bpl = f.fmt.pix_mp.plane_fmt[0].bytesperline;
	a->ext[Q_IN1].sizeimage = f.fmt.pix_mp.plane_fmt[0].sizeimage;

	/* ISP output1: XBGR32, same size as the input. */
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f.fmt.pix_mp.width = c->out_w;
	f.fmt.pix_mp.height = c->out_h;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_XBGR32;
	f.fmt.pix_mp.field = V4L2_FIELD_NONE;
	f.fmt.pix_mp.num_planes = 1;
	ret = agg_s_fmt(a, Q_OUT, N_OUT, &f);
	if (ret || f.fmt.pix_mp.width != c->out_w || f.fmt.pix_mp.height != c->out_h ||
	    f.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_XBGR32) {
		pr_err("v4h-cam-aggregator: ISP output format %ux%u failed (%d)\n",
		       c->out_w, c->out_h, ret);
		return ret ? ret : -EPIPE;
	}
	a->ext[Q_OUT].bpl = f.fmt.pix_mp.plane_fmt[0].bytesperline;
	a->ext[Q_OUT].sizeimage = f.fmt.pix_mp.plane_fmt[0].sizeimage;

	DBG("formats: sensor %ux%u, picture %ux%u, raw bpl %u/%u size %u/%u, out bpl %u size %u",
	    c->w, c->h, c->out_w, c->out_h,
	    a->ext[Q_VIN].bpl, a->ext[Q_IN1].bpl, a->ext[Q_VIN].sizeimage,
	    a->ext[Q_IN1].sizeimage, a->ext[Q_OUT].bpl, a->ext[Q_OUT].sizeimage);
	return 0;
}

/* -------------------------------------------------------------------------
 * Setting up the five queues
 */

static int agg_open_node(struct agg *a, enum agg_node n, enum agg_q q)
{
	char path[32];

	snprintf(path, sizeof(path), "/dev/video%d", a->ext[q].vdev->num);
	a->node_file[n] = filp_open(path, O_RDWR | O_NONBLOCK, 0);
	if (IS_ERR(a->node_file[n])) {
		int ret = PTR_ERR(a->node_file[n]);

		a->node_file[n] = NULL;
		pr_err("v4h-cam-aggregator: open %s failed: %d\n", path, ret);
		return ret;
	}
	DBG("setup: %s opened, hardware powered", path);
	return 0;
}

static unsigned int agg_gain_to_q8(enum agg_gain_model gm, int code)
{
	switch (gm) {
	case GAIN_IMX219:
		return 65536 / (256 - code);
	case GAIN_IMX708:
		return 262144 / (1024 - code);
	default:
		return 256;
	}
}

static int agg_q8_to_gain(enum agg_gain_model gm, unsigned int q8)
{
	switch (gm) {
	case GAIN_IMX219:
		return 256 - 65536 / q8;
	case GAIN_IMX708:
		return 1024 - 262144 / q8;
	default:
		return 0;
	}
}

/* First exposure and gain of the sensor; the sensor ranges depend on the mode. */
static void agg_3a_start(struct agg *a)
{
	struct agg_cam *c = &a->cam;
	int code;

	c->exp = v4l2_ctrl_find(c->sensor->ctrl_handler, V4L2_CID_EXPOSURE);
	c->gain = v4l2_ctrl_find(c->sensor->ctrl_handler, V4L2_CID_ANALOGUE_GAIN);
	if (c->gm == GAIN_NONE)
		c->gain = NULL;

	a->awb_r = 300;
	a->awb_b = 360;
	a->ae_iter = 0;
	a->settle = 6;
	a->fcnt = 0;
	a->exp = 0;
	a->gain_q8 = 256;

	if (c->exp) {
		a->exp = clamp_t(s64, expo ? expo : c->exp->default_value,
				 c->exp->minimum, c->exp->maximum);
		v4l2_ctrl_s_ctrl(c->exp, a->exp);
	}
	if (c->gain) {
		code = clamp_t(s64, gain ? gain : c->gain->default_value,
			       c->gain->minimum, c->gain->maximum);
		v4l2_ctrl_s_ctrl(c->gain, code);
		a->gain_q8 = agg_gain_to_q8(c->gm, code);
	}
	DBG("3a: first exposure %u gain x%u/256", a->exp, a->gain_q8);
}

static int agg_ext_setup(struct agg *a)
{
	static const struct {
		const char *name;	/* ISP video devices; the VIN one comes from the port */
		enum v4l2_buf_type type;
	} desc[Q_NUM] = {
		[Q_VIN] = { NULL, V4L2_BUF_TYPE_VIDEO_CAPTURE },
		[Q_IN1] = { "input1", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE },
		[Q_PAR] = { "params", V4L2_BUF_TYPE_META_OUTPUT },
		[Q_STA] = { "stats", V4L2_BUF_TYPE_META_CAPTURE },
		[Q_OUT] = { "output1", V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE },
	};
	char name[96];
	int ret;

	memset(a->ext, 0, sizeof(a->ext));

	ret = agg_detect(a);
	if (ret) {
		pr_err("v4h-cam-aggregator: no camera found in the V4H media graph (%d)\n", ret);
		return ret;
	}
	if (a->cam.out_w != a->w || a->cam.out_h != a->h) {
		pr_err("v4h-cam-aggregator: camera picture %ux%u differs from the node format %ux%u, reopen the node\n",
		       a->cam.out_w, a->cam.out_h, a->w, a->h);
		return -EPIPE;
	}

	DBG("setup: looking up the five queues");
	for (unsigned int i = 0; i < Q_NUM; i++) {
		struct agg_ext *e = &a->ext[i];

		if (i == Q_VIN)
			snprintf(name, sizeof(name), "%s", a->port->vin_vdev);
		else
			snprintf(name, sizeof(name), "rcar_isp %s %s", a->port->isp_pdev,
				 desc[i].name);

		e->vdev = agg_find_vdev(i == Q_VIN ? a->port->vin_pdev : a->port->isp_pdev, name);
		if (!e->vdev)
			return -ENODEV;
		e->q = e->vdev->queue;
		e->type = desc[i].type;
		e->mplane = e->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE ||
			    e->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		if (e->q->type != e->type) {
			pr_err("v4h-cam-aggregator: queue '%s' has type %u, expected %u\n",
			       name, e->q->type, e->type);
			return -EINVAL;
		}
	}

	DBG("setup: queues found, checking that nobody uses them");
	/* All or nothing: refuse when anybody else already uses a queue. */
	for (unsigned int i = 0; i < Q_NUM; i++) {
		struct agg_ext *e = &a->ext[i];

		if (e->q->owner || vb2_is_busy(e->q)) {
			pr_err("v4h-cam-aggregator: queue %u is in use (libcamera running?)\n", i);
			return -EBUSY;
		}
	}

	/*
	 * rvin_open() and risp_io_open() do pm_runtime_get, v4l2_pipeline_pm_get()
	 * (sensor, CSI-2) and, for the ISP, enable the core clock and release the
	 * reset. Skipping them leaves the hardware half asleep, so open the nodes
	 * we set formats on for real; the same files serve the S_FMT calls.
	 */
	ret = agg_open_node(a, N_VIN, Q_VIN);
	if (!ret)
		ret = agg_open_node(a, N_IN1, Q_IN1);
	if (!ret)
		ret = agg_open_node(a, N_OUT, Q_OUT);
	if (ret)
		goto err;

	ret = agg_set_formats(a);
	if (ret)
		goto err;

	for (unsigned int i = 0; i < Q_NUM; i++)
		a->ext[i].q->owner = a;

	for (unsigned int i = 0; i < Q_NUM; i++) {
		struct agg_ext *e = &a->ext[i];

		e->owned = true;
		DBG("setup: REQBUFS on queue %u", i);
		ret = agg_ext_reqbufs(a, i, AGG_NSET);
		if (ret) {
			pr_err("v4h-cam-aggregator: REQBUFS on queue %u failed: %d\n", i, ret);
			goto err;
		}
		for (unsigned int k = 0; k < AGG_NSET; k++) {
			e->vaddr[k] = vb2_plane_vaddr(vb2_get_buffer(e->q, k), 0);
			if (!e->vaddr[k]) {
				pr_err("v4h-cam-aggregator: no kernel mapping for queue %u buffer %u\n",
				       i, k);
				ret = -ENOMEM;
				goto err;
			}
		}
	}

	agg_3a_start(a);

	DBG("setup: buffers allocated and mapped, queueing VIN buffers");
	/* The raw frames start flowing now. */
	for (unsigned int k = 0; k < AGG_NSET; k++) {
		ret = agg_ext_qbuf(a, Q_VIN, k, 0);
		if (ret)
			goto err;
	}
	DBG("setup: VIN STREAMON");
	ret = agg_ext_streamon(a, Q_VIN);
	if (ret) {
		pr_err("v4h-cam-aggregator: VIN STREAMON failed: %d\n", ret);
		goto err;
	}
	DBG("setup: done");
	return 0;
err:
	agg_ext_close(a);
	return ret;
}

/* -------------------------------------------------------------------------
 * ISP parameters: black level, white balance, demosaic, sRGB gamma out.
 */

static unsigned int agg_fill_params(struct agg *a, void *p)
{
	struct v4l2_isp_params_buffer *hdr = p;
	u8 *d = hdr->data;
	size_t off = 0;
	struct rkisp1_ext_params_bls_config *bls = (void *)(d + off);
	struct rkisp1_ext_params_awb_gain_config *awb;
	struct rkisp1_ext_params_bdm_config *bdm;
	struct rkisp1_ext_params_goc_config *goc;
	int red = auto_awb ? a->awb_r : awb_red;
	int blue = auto_awb ? a->awb_b : awb_blue;
	int green = auto_awb ? 256 : awb_green;

	hdr->version = V4L2_ISP_PARAMS_VERSION_V1;

	memset(bls, 0, sizeof(*bls));
	bls->header.type = RKISP1_EXT_PARAMS_BLOCK_TYPE_BLS;
	bls->header.flags = black ? RKISP1_EXT_PARAMS_FL_BLOCK_ENABLE :
				    RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE;
	bls->header.size = sizeof(*bls);
	bls->config.fixed_val.r = clamp(black, 0, 4095);
	bls->config.fixed_val.gr = clamp(black, 0, 4095);
	bls->config.fixed_val.gb = clamp(black, 0, 4095);
	bls->config.fixed_val.b = clamp(black, 0, 4095);
	off += sizeof(*bls);

	awb = (void *)(d + off);
	memset(awb, 0, sizeof(*awb));
	awb->header.type = RKISP1_EXT_PARAMS_BLOCK_TYPE_AWB_GAIN;
	awb->header.flags = RKISP1_EXT_PARAMS_FL_BLOCK_ENABLE;
	awb->header.size = sizeof(*awb);
	awb->config.gain_red = clamp(red, 0, 1023);
	awb->config.gain_green_r = clamp(green, 0, 1023);
	awb->config.gain_blue = clamp(blue, 0, 1023);
	awb->config.gain_green_b = clamp(green, 0, 1023);
	off += sizeof(*awb);

	bdm = (void *)(d + off);
	memset(bdm, 0, sizeof(*bdm));
	bdm->header.type = RKISP1_EXT_PARAMS_BLOCK_TYPE_BDM;
	bdm->header.flags = RKISP1_EXT_PARAMS_FL_BLOCK_ENABLE;
	bdm->header.size = sizeof(*bdm);
	off += sizeof(*bdm);

	goc = (void *)(d + off);
	memset(goc, 0, sizeof(*goc));
	goc->header.type = RKISP1_EXT_PARAMS_BLOCK_TYPE_GOC;
	goc->header.flags = RKISP1_EXT_PARAMS_FL_BLOCK_ENABLE;
	goc->header.size = sizeof(*goc);
	goc->config.mode = RKISP1_CIF_ISP_GOC_MODE_EQUIDISTANT;
	for (unsigned int i = 0; i < ARRAY_SIZE(agg_gamma); i++)
		goc->config.gamma_y[i] = agg_gamma[i];
	off += sizeof(*goc);

	hdr->data_size = off;
	return sizeof(*hdr) + off;
}

/* -------------------------------------------------------------------------
 * AE and AWB from a coarse sample of the raw frame
 */

/* Index of R, G, G, B in a 2x2 quad (p00 p01 p10 p11) for every Bayer order. */
static const u8 agg_quad[4][4] = {
	{ 0, 1, 2, 3 },		/* RGGB */
	{ 1, 0, 3, 2 },		/* GRBG */
	{ 2, 0, 3, 1 },		/* GBRG */
	{ 3, 1, 2, 0 },		/* BGGR */
};

static void agg_3a(struct agg *a, const u8 *raw, u32 bpl)
{
	struct agg_cam *c = &a->cam;
	const u8 *t = agg_quad[c->bayer];
	u32 sr = 0, sg = 0, sb = 0, nu = 0, sgall = 0, n = 0;
	int mg_all, ratio, f;

	for (unsigned int gy = 0; gy < AGG_GRID_Y; gy++) {
		unsigned int y = (gy * a->h / AGG_GRID_Y) & ~1U;

		for (unsigned int gx = 0; gx < AGG_GRID_X; gx++) {
			unsigned int x = (gx * a->w / AGG_GRID_X) & ~1U;
			const __le16 *r0 = (const __le16 *)(raw + (size_t)y * bpl + x * 2);
			const __le16 *r1 = (const __le16 *)((const u8 *)r0 + bpl);
			u16 p[4];
			u32 g;

			p[0] = le16_to_cpu(r0[0]) & 0x3ff;
			p[1] = le16_to_cpu(r0[1]) & 0x3ff;
			p[2] = le16_to_cpu(r1[0]) & 0x3ff;
			p[3] = le16_to_cpu(r1[1]) & 0x3ff;
			g = (p[t[1]] + p[t[2]]) / 2;

			sgall += g;
			n++;
			if (max(max(p[0], p[1]), max(p[2], p[3])) < AGG_SAT10) {
				sr += p[t[0]];
				sg += g;
				sb += p[t[3]];
				nu++;
			}
		}
	}
	if (!n)
		return;

	if (auto_awb && nu >= n / 8) {
		int mr = max_t(int, sr / nu - AGG_BLACK10, 1);
		int mg = max_t(int, sg / nu - AGG_BLACK10, 1);
		int mb = max_t(int, sb / nu - AGG_BLACK10, 1);
		int tr = clamp(mg * 256 / mr, 128, 1000);
		int tb = clamp(mg * 256 / mb, 128, 1000);

		a->awb_r += (tr - a->awb_r) / 4;
		a->awb_b += (tb - a->awb_b) / 4;
	}

	if (!auto_ae || !c->exp)
		return;

	mg_all = max_t(int, sgall / n - AGG_BLACK10, 1);
	ratio = AGG_AE_TARGET * 256 / mg_all;
	if (ratio > 230 && ratio < 282)
		return;			/* within about 10 percent */

	f = clamp(ratio, 96, 640);
	if (a->ae_iter++ >= 3)
		f = 256 + (f - 256) / 2;	/* settle gently after the first steps */

	{
		unsigned int gmin = c->gain ? agg_gain_to_q8(c->gm, c->gain->minimum) : 256;
		unsigned int gmax = c->gain ? agg_gain_to_q8(c->gm, c->gain->maximum) : 256;
		u64 total = div_u64((u64)a->exp * a->gain_q8 * f, 256);
		u64 e = div_u64(total, gmin);
		unsigned int g;
		int code;

		/* Exposure first, then gain. */
		e = clamp_t(u64, e, c->exp->minimum, c->exp->maximum);
		g = clamp_t(u64, div64_u64(total, e), gmin, gmax);

		if (e != a->exp) {
			v4l2_ctrl_s_ctrl(c->exp, e);
			a->exp = e;
		}
		if (c->gain) {
			code = clamp_t(int, agg_q8_to_gain(c->gm, g), c->gain->minimum,
				       c->gain->maximum);
			if (agg_gain_to_q8(c->gm, code) != a->gain_q8) {
				v4l2_ctrl_s_ctrl(c->gain, code);
				a->gain_q8 = agg_gain_to_q8(c->gm, code);
			}
		}
		a->settle = 3;		/* the sensor needs a couple of frames to apply it */
		DBG("3a: mean %d -> exposure %u gain x%u/256, awb %d/%d", mg_all, a->exp,
		    a->gain_q8, a->awb_r, a->awb_b);
	}
}

/* -------------------------------------------------------------------------
 * Frame loop
 */

static void agg_copy(void *dst, unsigned int dbpl, unsigned int dsize,
		     const void *src, unsigned int sbpl, unsigned int ssize,
		     unsigned int rowbytes, unsigned int rows)
{
	if (dbpl == sbpl) {
		memcpy(dst, src, min(dsize, ssize));
		return;
	}
	for (unsigned int y = 0; y < rows; y++)
		memcpy(dst + (size_t)y * dbpl, src + (size_t)y * sbpl, rowbytes);
}

static void agg_emit_frame(struct agg *a, int out_idx)
{
	struct agg_buf *buf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&a->qlock, flags);
	if (!list_empty(&a->buf_list)) {
		buf = list_first_entry(&a->buf_list, struct agg_buf, list);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&a->qlock, flags);

	if (!buf) {
		a->dropped++;
		return;
	}

	agg_copy(vb2_plane_vaddr(&buf->vb.vb2_buf, 0), a->w * 4, a->out_size,
		 a->ext[Q_OUT].vaddr[out_idx], a->ext[Q_OUT].bpl, a->ext[Q_OUT].sizeimage,
		 a->w * 4, a->h);
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, a->out_size);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = a->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static int agg_thread(void *data)
{
	struct agg *a = data;
	u64 min_interval = max_fps > 0 ? div_u64(NSEC_PER_SEC, max_fps) * 9 / 10 : 0;
	u64 last_ns = 0;
	unsigned int queued = 0;
	bool started = false;
	int ret = 0;

	while (!kthread_should_stop()) {
		int vi, k;
		unsigned int pused;
		u8 *win;
		size_t win_off;

		if (queued < 3)
			DBG("loop: waiting for a VIN frame (queued %u)", queued);
		vi = agg_ext_dqbuf(a, Q_VIN, 200);
		if (vi == -ETIMEDOUT) {
			struct vb2_queue *vq = a->ext[Q_VIN].q;

			DBG("VIN queue: streaming=%u queued=%u owned=%d error=%u done_empty=%d",
			    vq->streaming, vq->queued_count,
			    atomic_read(&vq->owned_by_drv_count), vq->error,
			    list_empty(&vq->done_list));
			continue;
		}
		if (vi < 0) {
			ret = vi;
			break;
		}

		/* A sensor faster than max_fps: recycle the frame, the ISP never sees it. */
		if (started && min_interval) {
			u64 now = ktime_get_ns();

			if (now - last_ns < min_interval) {
				ret = agg_ext_qbuf(a, Q_VIN, vi, 0);
				if (ret)
					break;
				continue;
			}
			last_ns = now;
		}

		if (queued >= AGG_NSET) {
			/* Reuse the oldest set: collect its completion first. */
			int oi = agg_ext_dqbuf(a, Q_OUT, 1000);

			if (oi < 0) {
				pr_err("v4h-cam-aggregator: ISP output timeout/error %d\n", oi);
				ret = oi;
				break;
			}
			if (agg_ext_dqbuf(a, Q_IN1, 500) < 0 ||
			    agg_ext_dqbuf(a, Q_PAR, 500) < 0 ||
			    agg_ext_dqbuf(a, Q_STA, 500) < 0) {
				pr_err("v4h-cam-aggregator: ISP input/params/stats not returned\n");
				ret = -EIO;
				break;
			}
			agg_emit_frame(a, oi);
			k = oi;
		} else {
			k = queued;
		}

		if (queued < 3)
			DBG("loop: got VIN frame %d, set %d", vi, k);

		/* The picture is a window of the sensor frame. */
		win = a->ext[Q_VIN].vaddr[vi] + (size_t)a->cam.crop_y * a->ext[Q_VIN].bpl +
		      a->cam.crop_x * 2;
		win_off = win - (u8 *)a->ext[Q_VIN].vaddr[vi];

		/* AE and AWB look at every fourth frame, when the last change has settled. */
		if (a->settle)
			a->settle--;
		else if (!(a->fcnt++ & 3))
			agg_3a(a, win, a->ext[Q_VIN].bpl);

		/* VIN frame -> ISP input1. */
		agg_copy(a->ext[Q_IN1].vaddr[k], a->ext[Q_IN1].bpl, a->ext[Q_IN1].sizeimage,
			 win, a->ext[Q_VIN].bpl, a->ext[Q_VIN].sizeimage - win_off,
			 a->w * 2, a->h);
		ret = agg_ext_qbuf(a, Q_VIN, vi, 0);
		if (ret)
			break;

		pused = agg_fill_params(a, a->ext[Q_PAR].vaddr[k]);

		ret = agg_ext_qbuf(a, Q_IN1, k, a->ext[Q_IN1].sizeimage);
		if (!ret)
			ret = agg_ext_qbuf(a, Q_PAR, k, pused);
		if (!ret)
			ret = agg_ext_qbuf(a, Q_STA, k, 0);
		if (!ret)
			ret = agg_ext_qbuf(a, Q_OUT, k, 0);
		if (ret)
			break;
		queued++;

		if (!started && queued == AGG_NSET) {
			DBG("loop: ISP STREAMON params/stats/input1/output1");
			ret = agg_ext_streamon(a, Q_PAR);
			if (!ret)
				ret = agg_ext_streamon(a, Q_STA);
			if (!ret)
				ret = agg_ext_streamon(a, Q_IN1);
			if (!ret)
				ret = agg_ext_streamon(a, Q_OUT);
			if (ret) {
				pr_err("v4h-cam-aggregator: ISP STREAMON failed: %d\n", ret);
				break;
			}
			started = true;
			last_ns = ktime_get_ns();
			DBG("loop: ISP started");
		}
	}

	if (ret && !kthread_should_stop()) {
		pr_err("v4h-cam-aggregator: frame loop stopped: %d\n", ret);
		vb2_queue_error(&a->queue);
		/* Wait for the stop request so that the thread can be reaped. */
		while (!kthread_should_stop())
			msleep(20);
	}
	return 0;
}

/* -------------------------------------------------------------------------
 * The capture node VLC opens
 */

static int agg_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	struct agg *a = vb2_get_drv_priv(vq);

	if (*nplanes)
		return sizes[0] < a->out_size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = a->out_size;
	if (*nbuffers < 3)
		*nbuffers = 3;
	return 0;
}

static int agg_buf_prepare(struct vb2_buffer *vb)
{
	struct agg *a = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < a->out_size)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, a->out_size);
	return 0;
}

static void agg_buf_queue(struct vb2_buffer *vb)
{
	struct agg *a = vb2_get_drv_priv(vb->vb2_queue);
	struct agg_buf *buf = container_of(to_vb2_v4l2_buffer(vb), struct agg_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&a->qlock, flags);
	list_add_tail(&buf->list, &a->buf_list);
	spin_unlock_irqrestore(&a->qlock, flags);
}

static void agg_return_buffers(struct agg *a, enum vb2_buffer_state state)
{
	struct agg_buf *buf, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&a->qlock, flags);
	list_for_each_entry_safe(buf, tmp, &a->buf_list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&a->qlock, flags);
}

static int agg_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct agg *a = vb2_get_drv_priv(vq);
	int ret;

	a->sequence = 0;
	a->dropped = 0;

	ret = agg_ext_setup(a);
	if (ret)
		goto err_return;

	a->thread = kthread_run(agg_thread, a, "v4h-cam-agg%u", a->idx);
	if (IS_ERR(a->thread)) {
		ret = PTR_ERR(a->thread);
		a->thread = NULL;
		agg_ext_close(a);
		goto err_return;
	}
	return 0;

err_return:
	agg_return_buffers(a, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void agg_stop_streaming(struct vb2_queue *vq)
{
	struct agg *a = vb2_get_drv_priv(vq);

	if (a->thread) {
		kthread_stop(a->thread);
		a->thread = NULL;
	}
	agg_ext_close(a);
	agg_return_buffers(a, VB2_BUF_STATE_ERROR);
	pr_info("v4h-cam-aggregator: stopped, %u frames delivered, %lu dropped (no buffer queued)\n",
		a->sequence, a->dropped);
}

static const struct vb2_ops agg_qops = {
	.queue_setup = agg_queue_setup,
	.buf_prepare = agg_buf_prepare,
	.buf_queue = agg_buf_queue,
	.start_streaming = agg_start_streaming,
	.stop_streaming = agg_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int agg_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct agg *a = video_drvdata(file);

	strscpy(cap->driver, "v4h-cam-aggregator", sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "V4H Camera %s (aggregated)", a->port->name);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:v4h-cam-aggregator-%u", a->idx);
	return 0;
}

static int agg_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_BGR32;
	return 0;
}

static int agg_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct agg *a = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	memset(pix, 0, sizeof(*pix));
	pix->width = a->w;
	pix->height = a->h;
	pix->pixelformat = V4L2_PIX_FMT_BGR32;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = a->w * 4;
	pix->sizeimage = a->out_size;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	return 0;
}

static int agg_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
	struct agg *a = video_drvdata(file);

	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_BGR32)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = a->w;
	fsize->discrete.height = a->h;
	return 0;
}

static int agg_enum_input(struct file *file, void *priv, struct v4l2_input *i)
{
	if (i->index)
		return -EINVAL;
	i->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(i->name, "Camera", sizeof(i->name));
	return 0;
}

static int agg_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int agg_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static int agg_g_parm(struct file *file, void *priv, struct v4l2_streamparm *p)
{
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(&p->parm.capture, 0, sizeof(p->parm.capture));
	p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	p->parm.capture.timeperframe.numerator = 1;
	p->parm.capture.timeperframe.denominator = max_fps > 0 ? max_fps : 30;
	return 0;
}

static const struct v4l2_ioctl_ops agg_ioctl_ops = {
	.vidioc_querycap = agg_querycap,
	.vidioc_enum_fmt_vid_cap = agg_enum_fmt,
	.vidioc_g_fmt_vid_cap = agg_g_fmt,
	.vidioc_s_fmt_vid_cap = agg_g_fmt,
	.vidioc_try_fmt_vid_cap = agg_g_fmt,
	.vidioc_enum_framesizes = agg_enum_framesizes,
	.vidioc_enum_input = agg_enum_input,
	.vidioc_g_input = agg_g_input,
	.vidioc_s_input = agg_s_input,
	.vidioc_g_parm = agg_g_parm,
	.vidioc_s_parm = agg_g_parm,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

/*
 * The sensor may be bound long after this driver: look at the camera every
 * time the node is opened, so the format the application sees is the real one.
 */
static int agg_open(struct file *file)
{
	struct agg *a = video_drvdata(file);
	int ret = v4l2_fh_open(file);

	if (ret)
		return ret;

	mutex_lock(&a->lock);
	if (!vb2_is_busy(&a->queue))
		agg_refresh(a);
	mutex_unlock(&a->lock);
	return 0;
}

static const struct v4l2_file_operations agg_fops = {
	.owner = THIS_MODULE,
	.open = agg_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
};

static const struct agg_port agg_ports[] = {
	/* CSI40 -> ISP0 -> VIN0..7 (connector J1) */
	{ "J1", "e6ef0000.video", "VIN0 output", "fed00000.isp" },
	/* CSI41 -> ISP1 -> VIN8..15 (connector J2) */
	{ "J2", "e6ef8000.video", "VIN8 output", "fed20000.isp" },
};

static int agg_add_port(unsigned int idx)
{
	const struct agg_port *port = &agg_ports[idx];
	struct device *vin;
	struct agg *a;
	struct vb2_queue *q;
	int ret;

	/* Only for connectors that the device tree enables. */
	vin = bus_find_device_by_name(&platform_bus_type, NULL, port->vin_pdev);
	if (!vin)
		return -ENODEV;
	put_device(vin);

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	a->port = port;
	a->idx = idx;
	mutex_init(&a->lock);
	spin_lock_init(&a->qlock);
	INIT_LIST_HEAD(&a->buf_list);
	a->w = AGG_DEF_W;
	a->h = AGG_DEF_H;
	a->out_size = a->w * a->h * 4;

	a->pdev = platform_device_register_simple(KBUILD_MODNAME, idx, NULL, 0);
	if (IS_ERR(a->pdev)) {
		ret = PTR_ERR(a->pdev);
		goto err_free;
	}

	/* The platform device has no driver: v4l2_device_register() uses dev->driver. */
	snprintf(a->v4l2_dev.name, sizeof(a->v4l2_dev.name), "%s-%u", KBUILD_MODNAME, idx);
	ret = v4l2_device_register(&a->pdev->dev, &a->v4l2_dev);
	if (ret)
		goto err_pdev;

	q = &a->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_READ;
	q->drv_priv = a;
	q->buf_struct_size = sizeof(struct agg_buf);
	q->ops = &agg_qops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers = 2;
	q->lock = &a->lock;
	q->dev = &a->pdev->dev;
	ret = vb2_queue_init(q);
	if (ret)
		goto err_v4l2;

	snprintf(a->vdev.name, sizeof(a->vdev.name), "V4H Camera %s (aggregated)", port->name);
	a->vdev.v4l2_dev = &a->v4l2_dev;
	a->vdev.fops = &agg_fops;
	a->vdev.ioctl_ops = &agg_ioctl_ops;
	a->vdev.release = video_device_release_empty;
	a->vdev.queue = q;
	a->vdev.lock = &a->lock;
	a->vdev.vfl_dir = VFL_DIR_RX;
	a->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	video_set_drvdata(&a->vdev, a);

	ret = video_register_device(&a->vdev, VFL_TYPE_VIDEO, video_nr < 0 ? -1 : video_nr + idx);
	if (ret)
		goto err_v4l2;

	pr_info("v4h-cam-aggregator: %s: /dev/video%d ready (VIN %s, ISP %s); the camera is detected when the node is opened\n",
		port->name, a->vdev.num, port->vin_pdev, port->isp_pdev);
	return 0;

err_v4l2:
	v4l2_device_unregister(&a->v4l2_dev);
err_pdev:
	platform_device_unregister(a->pdev);
err_free:
	kfree(a);
	return ret;
}

static int __init agg_init(void)
{
	unsigned int added = 0;

	for (unsigned int i = 0; i < ARRAY_SIZE(agg_ports); i++)
		if (!agg_add_port(i))
			added++;

	return added ? 0 : -ENODEV;
}

module_init(agg_init);

MODULE_DESCRIPTION("Single-plane camera node for the R-Car V4H (VIN to ISP aggregator, imx219 and imx708)");
MODULE_LICENSE("GPL");
