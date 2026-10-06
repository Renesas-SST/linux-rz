// SPDX-License-Identifier: GPL-2.0

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/pm_runtime.h>

#include "remoteproc_internal.h"


/* Common CM33/CA55 address map (identical on RZ/V2H and RZ/{G,V}2L) */
#define CM33_SRAM_START			0x00000000
#define CM33_SRAM_END			0x3FFFFFFF
#define CA55_SRAM_START			0x00000000
#define CA55_DDR_START			0x40000000
#define CA55_DDR_CM33_START		0x40010000
#define CA55_DDR_CM33_END		0x43EFFFFF
#define CM33_TO_CA55_MASK		0x0FFFFFFF

#define RSC_TBL_SIZE			0x1000
#define RZ_POLL_TIMEOUT_US		100000

/* RZ/V2H CM33 DDR (view) range */
#define RZV2H_CM33_DDR_START		0x80000000
#define RZV2H_CM33_DDR_END		0x9FFFFFFF

/* RZ/G2L CM33 DDR (view) range */
#define RZG2L_CM33_DDR_START		0x60000000
#define RZG2L_CM33_DDR_END		0x7FFFFFFF

/* ------------------------------------------------------------------ */
/* RZ/V2H specific registers and masks                                */
/* ------------------------------------------------------------------ */
#define RZV2H_CPG_CLKON_1_CLK2_ON_MASK	0x00040000
#define RZV2H_CPG_BUS_10_MSTOP		0xD24
#define RZV2H_CPG_CLKON_1		0x604
#define RZV2H_CPG_CLKON_0		0x600
#define RZV2H_CPG_LP_CM33_CTL1		0xC1C
#define RZV2H_CPG_BUS_12_MSTOP		0xD2C
#define RZV2H_CPG_CM33_CTL		0xC0C
#define RZV2H_CPG_RST_1			0x904
#define RZV2H_CPG_RST_2			0x908
#define RZV2H_CPG_RSTMON_0		0xA00
#define RZV2H_CPG_RSTMON_1		0xA04
#define RZV2H_CPG_CLKMON_0		0x800
#define RZV2H_CPG_CLKMON_1		0x804
#define RZV2H_SYS_MCPU_CFG2		0x80C
#define RZV2H_SYS_MCPU_CFG3		0x810
#define RZV2H_CPG_LP_CR8_CTL3		0xC44
#define RZV2H_CPG_CR8_CONFIG1		0xC14
#define RZV2H_CPG_LP_CR8_CTL4		0xC48
#define RZV2H_CPG_CR8_CORESTATUS    0xC10

/* CPG_CR8_CORESTATUS: STANDBYWFI of both cores */
#define RZV2H_CR8_STANDBYWFI		GENMASK(17, 16)

/* CPG_BUS_m_MSTOP: upper half is the write enable of the lower half */
#define RZV2H_MSTOP_SET(bit)		(BIT((bit) + 16) | BIT(bit))
#define RZV2H_MSTOP_CLEAR(bit)		BIT((bit) + 16)
#define RZV2H_MSTOP_CR8_TCM			10	/* CPG_BUS_10_MSTOP */
#define RZV2H_MSTOP_MCPU_TO_ACPU	9	/* CPG_BUS_12_MSTOP */

/* RZ/V2H CR8 TCM mapping */
#define RZV2H_CR8_CORE0_ITCM_AXI_START	0x12040000
#define RZV2H_CR8_CORE1_ITCM_AXI_START	0x12080000
#define RZV2H_CR8_CORE_TCM_MAP_SIZE	0x00040000
#define RZV2H_RESET_CTRL_READY		BIT(4)
#define RZV2H_RESET_RELEASEREQ		BIT(3)
#define RZV2H_POWERUP_ACT			BIT(0)

/* RZ/V2H core IDs (from "renesas,rz-core") */
#define RZV2H_CM33_CORE_NUMBER		0x0
#define RZV2H_CR8_CORE0_NUMBER		0x1
#define RZV2H_CR8_CORE1_NUMBER		0x2

/* ------------------------------------------------------------------ */
/* RZ/G2L (and RZ/V2L) specific registers and masks                   */
/* ------------------------------------------------------------------ */
#define RZG2L_CPG_CLKON_CM33_CLK0_ON_MASK	0x00000001
#define RZG2L_CPG_SIPLL3_MON		0x13C
#define RZG2L_PLL3_RESET		BIT(0)
#define RZG2L_CPG_CLKON_CM33		0x504
#define RZG2L_CPG_CLKMON_CM33		0x684
#define RZG2L_CPG_RST_CM33		0x804
#define RZG2L_CPG_RSTMON_CM33		0x984
#define RZG2L_SYS_CM33_CFG0		0x804
#define RZG2L_SYS_CM33_CFG1		0x808
#define RZG2L_SYS_CM33_CFG2		0x80C
#define RZG2L_SYS_CM33_CFG3		0x810

enum rz_rproc_variant {
	RZ_VARIANT_RZV2H,
	RZ_VARIANT_RZG2L,
};

/* Per-variant descriptor */
struct rz_rproc_data {
	enum rz_rproc_variant variant;
	int (*start)(struct rproc *rproc);
	int (*stop)(struct rproc *rproc);
	int (*parse_fw)(struct rproc *rproc, const struct firmware *fw);

	/* Reset controls (CM33 variants only) */
	const char * const *reset_names;
	int num_resets;

	/* CPG reset-monitor register + mask for CM33 detach detection */
	u32 rstmon_reg;
	u32 rstmon_mask;

	bool needs_mem_region_request; /* RZ/G2L requests mem regions */
	bool detach_on_boot;           /* set RPROC_DETACHED if running */
};

struct rz_rproc_pdata {
	const struct rz_rproc_data *data;
	struct reset_control *resets[3];
	struct regmap *cpg_regmap;
	struct regmap *sysc_regmap;
	u32 bootaddr[2];
	u32 core; /* RZ/V2H core id; 0 (CM33) for RZ/G2L */
	bool cr8_cluster_held;
};

/*
 * RZ/V2H CR8 cluster state.
 *
 * The two CR8 cores share the same cluster hardware (clocks and reset). Per
 * the hardware manual, "The CR8 does not support per-core reset" - reset and
 * clock control are cluster-level. The only per-core control is nCPUHALT
 * (CR8_CONFIG1 BIT(0)/BIT(1)).
 *
 * A refcount tracks how many CR8 cores use the cluster so it is brought up by
 * the first user (0 -> 1) and torn down only by the last one (1 -> 0). Each
 * core holds at most one reference (cr8_cluster_held). It is protected by a
 * mutex to avoid races when the two cores are started/stopped concurrently.
 *
 * The reference is taken in .prepare and dropped in .unprepare: remoteproc
 * loads the ELF (rproc_load_segments) before calling .start, and the TCMs are
 * only accessible while the cluster is clocked and out of reset. Bringing the
 * cluster up in .start would make every boot after a .stop write the TCMs of
 * a gated cluster, which stalls the AXI bus and hangs the whole SoC.
 *
 * A core can only be (re)started from its reset vector while the cluster is
 * being brought up: nCPUHALT merely pauses and resumes a CPU. Restarting a
 * core that already ran, while the other core keeps the cluster up, would
 * resume it at its old PC on top of the newly loaded image, so it is refused.
 */
static DEFINE_MUTEX(rzv2h_cr8_cluster_lock);
static unsigned int rzv2h_cr8_cluster_refcnt;
/* BIT(core id) of the CR8 cores that ran since the cluster was brought up */
static unsigned long rzv2h_cr8_ran_mask;

static int rzv2h_cr8_cluster_get(struct rproc *rproc);
static void rzv2h_cr8_cluster_put(struct rproc *rproc);

static bool rz_rproc_is_cr8(const struct rz_rproc_pdata *pdata)
{
	return pdata->data->variant == RZ_VARIANT_RZV2H &&
		   (pdata->core == RZV2H_CR8_CORE0_NUMBER ||
		pdata->core == RZV2H_CR8_CORE1_NUMBER);
}

/* ================================================================== */
/* Common helpers                                                     */
/* ================================================================== */

static int rz_rproc_mem_alloc(struct rproc *rproc,
				  struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void __iomem *va;

	dev_dbg(dev, "map memory: %pa+%zx\n", &mem->dma, mem->len);
	va = devm_ioremap_wc(dev, mem->dma, mem->len);
	if (!va) {
		dev_err(dev, "unable to map memory region: %pa+%zx\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	mem->va = va;

	return 0;
}

static int rz_rproc_mem_release(struct rproc *rproc,
				struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;

	dev_dbg(dev, "unmap memory: %pa\n", &mem->dma);
	devm_iounmap(dev, mem->va);

	return 0;
}

static int rz_rproc_add_carveouts(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	struct resource *res;
	int index = 0;
	int i;
	u32 da;

	/* Register resources */
	for (i = 0; i < pdev->num_resources; i++) {
		res = pdev->resource + i;

		/* No need to translate pa to da, RZ use same map */
		da = res->start;

		mem = rproc_mem_entry_init(dev, NULL, res->start,
					   resource_size(res), da,
					   rz_rproc_mem_alloc,
					   rz_rproc_mem_release,
					   res->name);
		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
	}

	/* Register associated reserved memory regions */
	of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		if (rmem->base > U32_MAX)
			return -EINVAL;

		/* No need to translate pa to da, RZ use same map */
		da = rmem->base;

		bool is_vdev_buffer = str_has_prefix(it.node->name, "vdev") &&
							  strstr(it.node->name, "buffer");
		bool is_vdev_vring = str_has_prefix(it.node->name, "vdev") &&
							 strstr(it.node->name, "vring");
		char lookup_name[24];
		const char *carveout_name = it.node->name;
		unsigned int vring_idx;

		/*
		 * The remoteproc core looks up vring carveouts with the per-rproc
		 * pattern "vdev%dvring%d" where %d is rvdev->index (always 0 for
		 * the first vdev of each rproc). Our DT uses globally-unique names
		 * (e.g. vdev2vring0 on cr8_1), so rewrite them to the canonical
		 * per-rproc form before registering.
		 */
		if (is_vdev_vring) {
			const char *vring_tag = strstr(it.node->name, "vring");

			if (vring_tag && kstrtouint(vring_tag + strlen("vring"),
						10, &vring_idx) == 0) {
				snprintf(lookup_name, sizeof(lookup_name),
						 "vdev0vring%u", vring_idx);
				carveout_name = lookup_name;
			}
		} else if (is_vdev_buffer) {
			carveout_name = "vdev0buffer";
		}

		if (!is_vdev_buffer) {
			mem = rproc_mem_entry_init(dev, NULL, rmem->base,
						   rmem->size, da,
						   rz_rproc_mem_alloc,
						   rz_rproc_mem_release,
						   carveout_name);
		} else {
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   rmem->base,
							   carveout_name);
		}

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}

	return 0;
}

static int rz_rproc_prepare(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct rz_rproc_pdata *pdata = rproc->priv;
	int ret;

	ret = rz_rproc_add_carveouts(rproc);
	if (ret)
		return ret;

	/* If the remote core is already running (DETACHED), skip startup */
	if (rproc->state == RPROC_DETACHED) {
		dev_info(dev, "remote core already running, skip startup\n");
		return 0;
	}

	/* CR8: the TCMs must be accessible before the ELF is loaded */
	if (rz_rproc_is_cr8(pdata))
		return rzv2h_cr8_cluster_get(rproc);

	return 0;
}

static int rz_rproc_unprepare(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	if (rz_rproc_is_cr8(pdata))
		rzv2h_cr8_cluster_put(rproc);

	return 0;
}

static int rz_rproc_attach(struct rproc *rproc)
{
	return 0;
}

static void rz_rproc_kick(struct rproc *rproc, int vqid)
{
	/* Not supported Linux RPMsg yet */
}

/* ================================================================== */
/* Address translation                                                */
/* ================================================================== */

static int cm33_to_ca55(struct rz_rproc_pdata *pdata, u64 *da)
{
	u32 ddr_start, ddr_end;

	/* SRAM range is identical on both SoCs */
	if ((CM33_SRAM_END >= *da) && (*da >= CM33_SRAM_START)) {
		*da = CA55_SRAM_START + (*da & CM33_TO_CA55_MASK);
		return 0;
	}

	/* DDR view differs per SoC - select by compatible/variant */
	if (pdata->data->variant == RZ_VARIANT_RZV2H) {
		ddr_start = RZV2H_CM33_DDR_START;
		ddr_end   = RZV2H_CM33_DDR_END;
	} else {
		ddr_start = RZG2L_CM33_DDR_START;
		ddr_end   = RZG2L_CM33_DDR_END;
	}

	if ((ddr_end >= *da) && (*da >= ddr_start)) {
		*da = CA55_DDR_START + (*da & CM33_TO_CA55_MASK);
		return 0;
	}

	return -EINVAL;
}

static void *rz_rproc_da_to_va(struct rproc *rproc, u64 da, size_t len,
				   bool *is_iomem)
{
	struct device *dev = rproc->dev.parent;
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *carveout;
	void *ptr = NULL;
	int ret;

	if (pdata->data->variant == RZ_VARIANT_RZV2H &&
		(pdata->core == RZV2H_CR8_CORE0_NUMBER ||
		 pdata->core == RZV2H_CR8_CORE1_NUMBER)) {
		/*
		 * CR8 cores use a flat address map: the ELF paddr equals the
		 * physical (AXI) address, except for the low TCM window which
		 * is aliased at 0x0..0x3FFFF and must be remapped to the
		 * per-core ITCM AXI base. All other addresses are used as-is
		 * to look up carveouts - they must NOT go through the CM33
		 * translation path.
		 */
		if (da < RZV2H_CR8_CORE_TCM_MAP_SIZE) {
			if (pdata->core == RZV2H_CR8_CORE0_NUMBER)
				da += RZV2H_CR8_CORE0_ITCM_AXI_START;
			else
				da += RZV2H_CR8_CORE1_ITCM_AXI_START;
		}
	} else {
		/* CM33 core (RZ/V2H or RZ/G2L) */
		if ((CA55_DDR_CM33_END >= da) && (da >= CA55_DDR_CM33_START)) {
			/* @da is address of trace buffer. Do nothing. */
		} else {
			ret = cm33_to_ca55(pdata, &da);
			if (ret) {
				dev_err(dev, "invalid address 0x%llx\n", da);
				return ptr;
		}
	}
}

	list_for_each_entry(carveout, &rproc->carveouts, node) {
		int offset = da - carveout->da;

		if (!carveout->va)
			continue;
		if (offset < 0)
			continue;
		if (offset + len > carveout->len)
			continue;

		ptr = carveout->va + offset;
		break;
	}

	return ptr;
}

/* ================================================================== */
/* RZ/V2H CM33 startup / shutdown                                     */
/* ================================================================== */

static int rzv2h_cpg_poll(struct rz_rproc_pdata *pdata, unsigned int reg,
			  u32 mask, u32 expected)
{
	u32 val;

	return regmap_read_poll_timeout(pdata->cpg_regmap, reg, val,
					(val & mask) == expected, 10,
					RZ_POLL_TIMEOUT_US);
}

static int rzv2h_cm33_assert_reset(struct rz_rproc_pdata *pdata)
{
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380000);
	return rzv2h_cpg_poll(pdata, RZV2H_CPG_RSTMON_0, 0x000E0000, 0x000E0000);
}

/* Releasing Cold Reset (Normal mode), HW manual Table 2.2-30 */
static int rzv2h_cm33_release_reset(struct rz_rproc_pdata *pdata)
{
	int ret;

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380008);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_RSTMON_0, 0x000E0000, 0x000C0000);
	if (ret)
		return ret;

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380038);
	return rzv2h_cpg_poll(pdata, RZV2H_CPG_RSTMON_0, 0x000E0000, 0);
}

static int rzv2h_cm33_clk_off(struct rz_rproc_pdata *pdata)
{
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00040000);
	return rzv2h_cpg_poll(pdata, RZV2H_CPG_CLKMON_0,
				  RZV2H_CPG_CLKON_1_CLK2_ON_MASK, 0);
}

static int rzv2h_cm33_set_vtor(struct rz_rproc_pdata *pdata, unsigned int reg,
				   u32 addr)
{
	u32 val;
	int ret;

	ret = regmap_write(pdata->sysc_regmap, reg, addr);
	if (!ret)
		ret = regmap_read(pdata->sysc_regmap, reg, &val);
	if (!ret && val != addr)
		ret = -EACCES;  /* locked by SYS_MCPU_CFG5 */

	return ret;
}

static int rzv2h_cm33_startup(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct rz_rproc_pdata *pdata = rproc->priv;
	u32 clkmon;
	int ret;

	/* Clear MSTOP between the MCPU bus and the ACPU bus */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_BUS_12_MSTOP,
			 RZV2H_MSTOP_CLEAR(RZV2H_MSTOP_MCPU_TO_ACPU));

	regmap_read(pdata->cpg_regmap, RZV2H_CPG_CLKMON_0, &clkmon);

	ret = rzv2h_cm33_assert_reset(pdata);
	if (ret)
		goto err;

	if (clkmon & RZV2H_CPG_CLKON_1_CLK2_ON_MASK) {
		ret = rzv2h_cm33_clk_off(pdata);
		if (ret)
			goto err;
	}

	/*
	 * If the boot vectors cannot be programmed the CM33 would start from its
	 * boot ROM, which re-initialises the SoC and hangs the running system.
	 * Leave the core in reset instead.
	 */
	ret = rzv2h_cm33_set_vtor(pdata, RZV2H_SYS_MCPU_CFG2, pdata->bootaddr[0]);
	if (!ret)
		ret = rzv2h_cm33_set_vtor(pdata, RZV2H_SYS_MCPU_CFG3,
					  pdata->bootaddr[1]);
	if (ret) {
		dev_err(dev, "failed to set CM33 boot vectors: %d\n", ret);
		return ret;
	}
	dev_info(dev, "CM33 bootaddr secure=0x%08x non-secure=0x%08x\n",
		 pdata->bootaddr[0], pdata->bootaddr[1]);

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00040004);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_CLKMON_0,
				 RZV2H_CPG_CLKON_1_CLK2_ON_MASK,
				 RZV2H_CPG_CLKON_1_CLK2_ON_MASK);
	if (ret)
		goto err;

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CM33_CTL1, 0x00003100);

	/* Fetch disable is a debug mode setting: boot normally */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CM33_CTL, 0x00000000);

	ret = rzv2h_cm33_release_reset(pdata);
	if (ret)
		goto err;

	dev_info(dev, "CM33 released from reset\n");
	return 0;

err:
	dev_err(dev, "CM33 CPG timeout\n");
	return ret;
}

/*
 * Leave the CM33 in a state from which the next start works.
 *
 * The CM33 cold reset is to be applied while the CM33 is in system MPU sleep
 * (HW manual Table 2.2-29). A firmware that is stopped while running violates
 * this, and the next reset release then does not start execution: every other
 * start fails. So run a stub from the boot vector that only executes WFI, and
 * reset the core again while it waits there.
 *
 * The stub must not enter the CM33 sleep mode proper (SCR.SLEEPDEEP). It does
 * set the sleep and deep standby status bits of CPG_LP_CM33CTL0 that the
 * manual asks for, but resetting the CM33 in that state without the rest of
 * the CM33 sleep mode procedure (CPG_LP_CTL1 CM33SLEEP_REQ/ACK handshake)
 * leads to a SoC hang or reset some seconds to minutes later.
 */
static void rzv2h_cm33_park(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct device *dev = rproc->dev.parent;
	u32 vtor = pdata->bootaddr[0];
	void __iomem *va;

	va = (void __iomem *)rz_rproc_da_to_va(rproc, vtor, 16, NULL);
	if (!va) {
		dev_warn(dev, "no memory at boot vector, CM33 not parked\n");
		return;
	}

	writel(vtor + 0x100, va + 0x0);     /* initial SP (unused) */
	writel((vtor + 0x8) | 1, va + 0x4); /* reset handler, Thumb */
	writel(0xBF30B672, va + 0x8);       /* cpsid i; wfi */
	writel(0x0000E7FD, va + 0xC);       /* b wfi */
	wmb();  /* stub in SRAM before the core fetches it */

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CM33_CTL, 0x00000000);
	if (rzv2h_cm33_release_reset(pdata))
		dev_warn(dev, "CM33 park: reset release timeout\n");
	usleep_range(1000, 2000);
	rzv2h_cm33_assert_reset(pdata);

	memset_io(va, 0, 16);
}

static int rzv2h_stop_cm33(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	int ret;

	/*
	 * Its carveouts are not cleared: cm33_rsc_table is the OpenAMP window
	 * shared with the CR8 cores.
	 */
	ret = rzv2h_cm33_assert_reset(pdata);
	if (ret)
		return ret;

	rzv2h_cm33_park(rproc);

	return rzv2h_cm33_clk_off(pdata);
}

/* ================================================================== */
/* RZ/V2H CR8 startup / shutdown                                      */
/* ================================================================== */

/*
 * Cold reset release, HW manual Table 2.2-45, with both CPUs held by nCPUHALT
 * so that the TCMs can be loaded (step 11) before a core is started.
 */
static int rzv2h_cr8_cluster_up(struct rz_rproc_pdata *pdata)
{
	int ret;

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_0, 0xE000E000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00030003);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_CLKMON_0, 0x0003E000, 0x0003E000);
	if (ret)
		return ret;

	/* Clocks are supplied: clear MSTOP of the CR8 TCM bus */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_BUS_10_MSTOP,
			 RZV2H_MSTOP_CLEAR(RZV2H_MSTOP_CR8_TCM));

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x1FFF0000);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_RSTMON_0, 0xFFF00000, 0xFFF00000);
	if (!ret)
		ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_RSTMON_1, 0x1, 0x1);
	if (ret)
		return ret;

	/* Debug mode setting (step 2), keeps the CR8 debuggable over JTAG */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL3, 0x003F0000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1, 0x00000000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x10001000);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_LP_CR8_CTL4,
				 RZV2H_RESET_CTRL_READY, RZV2H_RESET_CTRL_READY);
	if (ret)
		return ret;

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL4, 0x00000020);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_LP_CR8_CTL4,
				 RZV2H_RESET_RELEASEREQ, RZV2H_RESET_RELEASEREQ);
	if (ret)
		return ret;

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x1FFF1FFF);
	ret = rzv2h_cpg_poll(pdata, RZV2H_CPG_LP_CR8_CTL4, RZV2H_POWERUP_ACT, 0);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL4, 0x00000000);

	return ret;
}

/* Cold reset (HW manual Table 2.2-44), then module stop of the cluster */
static void rzv2h_cr8_cluster_down(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	if (rzv2h_cpg_poll(pdata, RZV2H_CPG_CR8_CORESTATUS,
			   RZV2H_CR8_STANDBYWFI, RZV2H_CR8_STANDBYWFI))
		dev_dbg(rproc->dev.parent, "CR8 cores not in WFI, resetting\n");

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x1FFF0000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_BUS_10_MSTOP,
			 RZV2H_MSTOP_SET(RZV2H_MSTOP_CR8_TCM));
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00030000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_0, 0xE0000000);
}

/*
 * Take a cluster reference for this core. The first user brings the cluster
 * up (clocks on, resets released) with both CPUs held by nCPUHALT, so the
 * TCMs can be loaded; the core itself is released later in .start.
 */
static int rzv2h_cr8_cluster_get(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	int ret;

	guard(mutex)(&rzv2h_cr8_cluster_lock);

	if (pdata->cr8_cluster_held)
		return 0;

	if (rzv2h_cr8_cluster_refcnt) {
		/* Cluster already brought up by the other core */
		if (rzv2h_cr8_ran_mask & BIT(pdata->core)) {
			dev_err(rproc->dev.parent,
				"CR8 core %u cannot be restarted while the other CR8 core runs (no per-core reset); stop both cores first\n",
				pdata->core - RZV2H_CR8_CORE0_NUMBER);
			return -EBUSY;
		}
		pdata->cr8_cluster_held = true;
		rzv2h_cr8_cluster_refcnt++;
		return 0;
	}
	ret = rzv2h_cr8_cluster_up(pdata);
	if (ret) {
		dev_err(rproc->dev.parent, "CR8 cluster bring-up timeout\n");
		rzv2h_cr8_cluster_down(rproc);
		return ret;
	}

	pdata->cr8_cluster_held = true;
	rzv2h_cr8_cluster_refcnt = 1;
	rzv2h_cr8_ran_mask = 0;

	return 0;
}

/*
 * Drop this core's cluster reference. The CR8 does not support per-core
 * reset/clock gating, so only the last user tears the shared cluster down.
 */
static void rzv2h_cr8_cluster_put(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	guard(mutex)(&rzv2h_cr8_cluster_lock);

	if (!pdata->cr8_cluster_held)
		return;
	pdata->cr8_cluster_held = false;

	if (WARN_ON(rzv2h_cr8_cluster_refcnt == 0))
		return;
	if (--rzv2h_cr8_cluster_refcnt)
		return;

	rzv2h_cr8_cluster_down(rproc);
}

static u32 rzv2h_cr8_halt_bit(struct rz_rproc_pdata *pdata)
{
	return pdata->core == RZV2H_CR8_CORE0_NUMBER ? BIT(0) : BIT(1);
}

static int rzv2h_start_cr8(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	/* The cluster is up (taken in .prepare); release this core's nCPUHALT */
	guard(mutex)(&rzv2h_cr8_cluster_lock);
	rzv2h_cr8_ran_mask |= BIT(pdata->core);
	regmap_update_bits(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1,
			   rzv2h_cr8_halt_bit(pdata), rzv2h_cr8_halt_bit(pdata));

	return 0;
}

static int rzv2h_stop_cr8(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *carveout;
	void *marker = NULL;

	/*
	 * Per-core control is limited to nCPUHALT (CR8_CONFIG1). The shared
	 * cluster is released in .unprepare, once the core is quiesced.
	 */
	scoped_guard(mutex, &rzv2h_cr8_cluster_lock)
		regmap_update_bits(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1,
				   rzv2h_cr8_halt_bit(pdata), 0);

	/*
	 * Only clear the "running" marker checked at probe (first word of
	 * cr8_ddr), not the whole carveouts: they are not private to this core
	 * (cr8_sram3 of core1 includes core0's init-wait word, and the OpenAMP
	 * window cr8_rsc_table is shared by all remote cores). Wiping them would
	 * corrupt a core that is still running. They may also cover the TF-A
	 * secure DDR (cm33_rsc_table reaches 0x44dfffff, BL31 is at
	 * 0x44000000): wiping BL31 hangs the next PSCI call (e.g. SYSTEM_RESET
	 * on reboot).
	 */
	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if (carveout->va && !strcmp(carveout->name, "cr8_ddr")) {
			marker = carveout->va;
			break;
		}
	}
	if (marker)
		writel(0, (void __iomem *)marker);

	return 0;
}

/* ================================================================== */
/* RZ/V2H start / stop dispatch (by core)                             */
/* ================================================================== */

static int rzv2h_rproc_start(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct device *dev = rproc->dev.parent;
	int ret;

	switch (pdata->core) {
	case RZV2H_CM33_CORE_NUMBER:
		ret = rzv2h_cm33_startup(rproc);
		if (ret)
			dev_err(dev, "CM33 startup failed: %d\n", ret);
		return ret;

	case RZV2H_CR8_CORE0_NUMBER:
	case RZV2H_CR8_CORE1_NUMBER:
		return rzv2h_start_cr8(rproc);

	default:
		dev_err(dev, "Unsupported core id: %d\n", pdata->core);
		return -EOPNOTSUPP;
	}
}

static int rzv2h_rproc_stop(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct device *dev = rproc->dev.parent;

	switch (pdata->core) {
	case RZV2H_CM33_CORE_NUMBER:
		return rzv2h_stop_cm33(rproc);

	case RZV2H_CR8_CORE0_NUMBER:
	case RZV2H_CR8_CORE1_NUMBER:
		return rzv2h_stop_cr8(rproc);

	default:
		dev_err(dev, "Unsupported core id: %d\n", pdata->core);
		return -EOPNOTSUPP;
	}
}

static int rzv2h_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret && ret != -EINVAL && ret != -ENOENT)
		dev_warn(&rproc->dev, "failed to load resource table: %d\n", ret);

	return 0;
}

/* ================================================================== */
/* RZ/G2L (RZ/V2L) start / stop                                       */
/* ================================================================== */

static int rzg2l_rproc_start(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	u32 val;
	int ret;

	regmap_read(pdata->cpg_regmap, RZG2L_CPG_SIPLL3_MON, &val);
	if ((val & RZG2L_PLL3_RESET) == 0x1) {
		/* Normal mode */
		regmap_write(pdata->sysc_regmap, RZG2L_SYS_CM33_CFG0, 0x01003CE5);
		regmap_write(pdata->sysc_regmap, RZG2L_SYS_CM33_CFG1, 0x01003CE5);
	} else {
		/* Standby mode */
		regmap_write(pdata->sysc_regmap, RZG2L_SYS_CM33_CFG0, 0x00003D08);
		regmap_write(pdata->sysc_regmap, RZG2L_SYS_CM33_CFG1, 0x00003D08);
	}

	regmap_write(pdata->sysc_regmap, RZG2L_SYS_CM33_CFG2, pdata->bootaddr[0]);
	regmap_write(pdata->sysc_regmap, RZG2L_SYS_CM33_CFG3, pdata->bootaddr[1]);

	regmap_write(pdata->cpg_regmap, RZG2L_CPG_CLKON_CM33, 0x00010001);
	ret = regmap_read_poll_timeout(pdata->cpg_regmap, RZG2L_CPG_CLKMON_CM33,
					   val, val & RZG2L_CPG_CLKON_CM33_CLK0_ON_MASK,
					   10, RZ_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	regmap_write(pdata->cpg_regmap, RZG2L_CPG_RST_CM33, 0x00040004);
	regmap_write(pdata->cpg_regmap, RZG2L_CPG_RST_CM33, 0x00070007);
	return regmap_read_poll_timeout(pdata->cpg_regmap, RZG2L_CPG_RSTMON_CM33,
					val, !(val & 0x00000007), 10,
					RZ_POLL_TIMEOUT_US);
}

static int rzg2l_rproc_stop(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *carveout;
	int i, ret;

	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if (!carveout->va)
			continue;
		memset(carveout->va, 0, carveout->len);
	}

	for (i = 0; i < pdata->data->num_resets; i++) {
		ret = reset_control_assert(pdata->resets[i]);
		if (ret) {
			dev_err(dev, "failed to assert %s\n",
				pdata->data->reset_names[i]);
			return ret;
		}
	}

	pm_runtime_put(dev);

	return 0;
}

static int rzg2l_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret)
		dev_warn(&rproc->dev, "no resource table found for this firmware\n");

	return 0;
}

/* ================================================================== */
/* rproc ops (dispatch to variant callbacks)                          */
/* ================================================================== */

static int rz_rproc_start(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	return pdata->data->start(rproc);
}

static int rz_rproc_stop(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	return pdata->data->stop(rproc);
}

static int rz_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	struct rz_rproc_pdata *pdata = rproc->priv;

	return pdata->data->parse_fw(rproc, fw);
}
static const struct rproc_ops rz_rproc_ops = {
	.prepare		= rz_rproc_prepare,
	.unprepare		= rz_rproc_unprepare,
	.start			= rz_rproc_start,
	.stop			= rz_rproc_stop,
	.attach			= rz_rproc_attach,
	.kick			= rz_rproc_kick,
	.da_to_va		= rz_rproc_da_to_va,
	.parse_fw		= rz_rproc_parse_fw,
	.find_loaded_rsc_table	= rproc_elf_find_loaded_rsc_table,
	.load			= rproc_elf_load_segments,
	.sanity_check		= rproc_elf_sanity_check,
	.get_boot_addr		= rproc_elf_get_boot_addr,
};

/* ================================================================== */
/* Variant descriptors                                                */
/* ================================================================== */

/*
 * Reset line names must match the "reset-names" DT property of each SoC's
 * binding. They intentionally differ between RZ/V2H and RZ/G2L and must not
 * be unified, to preserve devicetree ABI compatibility.
 */
static const char * const rzv2h_cm33_reset_names[] = {
	"cm33reset0", "cm33reset1", "cm33reset2",
};

static const char * const rzg2l_cm33_reset_names[] = {
	"nporeset", "nsysreset", "miscresetn",
};

static const struct rz_rproc_data rzv2h_cm33_rproc_data = {
	.variant		= RZ_VARIANT_RZV2H,
	.start			= rzv2h_rproc_start,
	.stop			= rzv2h_rproc_stop,
	.parse_fw		= rzv2h_rproc_parse_fw,
	.reset_names		= rzv2h_cm33_reset_names,
	.num_resets		= ARRAY_SIZE(rzv2h_cm33_reset_names),
	.rstmon_reg		= RZV2H_CPG_RSTMON_0,
	.rstmon_mask		= 0x000E0000,
	.needs_mem_region_request = false,
	.detach_on_boot		= false,
};

/* CR8 has no reset controls in DT — num_resets = 0 */
static const struct rz_rproc_data rzv2h_cr8_rproc_data = {
	.variant		= RZ_VARIANT_RZV2H,
	.start			= rzv2h_rproc_start,
	.stop			= rzv2h_rproc_stop,
	.parse_fw		= rzv2h_rproc_parse_fw,
	.reset_names		= NULL,
	.num_resets		= 0,
	.rstmon_reg		= RZV2H_CPG_RSTMON_0,
	.rstmon_mask		= 0x000E0000,
	.needs_mem_region_request = false,
	.detach_on_boot		= false,
};

static const struct rz_rproc_data rzg2l_cm33_rproc_data = {
	.variant		= RZ_VARIANT_RZG2L,
	.start			= rzg2l_rproc_start,
	.stop			= rzg2l_rproc_stop,
	.parse_fw		= rzg2l_rproc_parse_fw,
	.reset_names		= rzg2l_cm33_reset_names,
	.num_resets		= ARRAY_SIZE(rzg2l_cm33_reset_names),
	.rstmon_reg		= RZG2L_CPG_RSTMON_CM33,
	.rstmon_mask		= 0x00000007,  /* bits[2:0]: nporeset, nsysreset, miscresetn */
	.needs_mem_region_request = true,
	.detach_on_boot		= false,
};

/* ================================================================== */
/* Detach / rsc-table helpers                                         */
/* ================================================================== */

static void rz_rproc_attach_rsc_table(struct device *dev, struct rproc *rproc)
{
	struct device_node *np = dev->of_node;
	struct resource_table *rsc_table;
	void __iomem *rsc_va;
	u32 rsc_pa;

	if (of_property_read_u32_index(np, "renesas,rz-rsctbl", 0, &rsc_pa)) {
		dev_warn(dev, "detached firmware has no resource table\n");
		return;
	}

	rsc_va = devm_ioremap_wc(dev, rsc_pa, RSC_TBL_SIZE);
	if (!rsc_va) {
		dev_err(dev, "unable to map memory region: %pa+%zx\n",
			&rsc_pa, (size_t)RSC_TBL_SIZE);
		return;
	}

	rsc_table = (struct resource_table *)rsc_va;
	if (rsc_table->ver != 1) {
		devm_iounmap(dev, rsc_va);
		dev_warn(dev, "detached firmware has no resource table\n");
		return;
	}

	rproc->table_ptr = rsc_table;
	rproc->table_sz = RSC_TBL_SIZE;
}

/*
 * Returns true only if the remote core is truly executing.
 *
 * For RZ/V2H CM33, U-Boot may leave the core with reset deasserted but
 * fetch disabled (CPG_CM33_CTL bit[0] = 1) as a handoff convention — the
 * core is NOT executing in that state. Only report "running" when both
 * reset is deasserted (RSTMON bits clear) AND fetch is enabled
 * (CM33_CTL bit[0] == 0).
 */
static int rz_rproc_check_running(struct platform_device *pdev,
				  struct rz_rproc_pdata *pdata,
				  bool *running)
{
	struct device *dev = &pdev->dev;
	const struct rz_rproc_data *data = pdata->data;
	void __iomem *ddr_cr8_base;
	struct resource *res;
	u32 rstmon, cm33ctl;

	*running = false;

	/* RZ/V2H CR8: check the CR8 DDR marker written by firmware */
	if (data->variant == RZ_VARIANT_RZV2H &&
		(pdata->core == RZV2H_CR8_CORE0_NUMBER ||
		 pdata->core == RZV2H_CR8_CORE1_NUMBER)) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   "cr8_ddr");
		if (!res) {
			dev_err(dev, "cannot get cr8_ddr\n");
			return -EINVAL;
		}

		ddr_cr8_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(ddr_cr8_base))
			return PTR_ERR(ddr_cr8_base);

		if (ioread32(ddr_cr8_base) != 0) {
			*running = true;
			/* The running core holds a cluster reference until stopped */
			guard(mutex)(&rzv2h_cr8_cluster_lock);
			if (!pdata->cr8_cluster_held) {
				pdata->cr8_cluster_held = true;
				rzv2h_cr8_cluster_refcnt++;
				rzv2h_cr8_ran_mask |= BIT(pdata->core);
			}
		}
		return 0;
	}

	/*
	 * RZ/V2H CM33: check both RSTMON and CM33_CTL.
	 * U-Boot handoff leaves: RSTMON=0 (reset deasserted) + CM33_CTL=1
	 * (fetch disabled). That is NOT running: Linux loads the firmware and
	 * restarts the core.
	 */
	if (data->variant == RZ_VARIANT_RZV2H &&
		pdata->core == RZV2H_CM33_CORE_NUMBER) {
		regmap_read(pdata->cpg_regmap, data->rstmon_reg, &rstmon);
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_CM33_CTL, &cm33ctl);
		*running = !(rstmon & data->rstmon_mask) && !(cm33ctl & BIT(0));
		return 0;
	}

	/* RZ/G2L: RSTMON-only check is sufficient */
	regmap_read(pdata->cpg_regmap, data->rstmon_reg, &rstmon);
	*running = !(rstmon & data->rstmon_mask);

	return 0;
}

/* ================================================================== */
/* Probe / remove                                                     */
/* ================================================================== */

static int rz_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct rz_rproc_data *data;
	struct rz_rproc_pdata *pdata;
	struct rproc *rproc;
	struct resource *res;
	bool running = false;
	int ret, i;

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "no match data\n");
		return -ENODEV;
	}

	rproc = devm_rproc_alloc(dev, np->name, &rz_rproc_ops, NULL,
				 sizeof(*pdata));
	if (!rproc)
		return -ENOMEM;

	pdata = rproc->priv;
	pdata->data = data;

	/* RZ/V2H: read core id (defaults to CM33 core 0 if absent) */
	if (data->variant == RZ_VARIANT_RZV2H)
		of_property_read_u32_index(np, "renesas,rz-core", 0,
					   &pdata->core);

	/* RZ/G2L requests memory regions explicitly */
	if (data->needs_mem_region_request) {
		for (i = 0; i < pdev->num_resources; i++) {
			res = platform_get_resource(pdev, IORESOURCE_MEM, i);
			if (!res)
				continue;
			if (!devm_request_mem_region(dev, res->start,
							 resource_size(res),
							 dev_name(dev))) {
				dev_err(dev, "unable to request memory region\n");
				return -EBUSY;
			}
		}
	}

	pdata->cpg_regmap = syscon_regmap_lookup_by_phandle(np, "renesas,rz-cpg");
	if (IS_ERR(pdata->cpg_regmap)) {
		dev_err(dev, "failed to lookup cpg regmap\n");
		return PTR_ERR(pdata->cpg_regmap);
	}

	pdata->sysc_regmap = syscon_regmap_lookup_by_phandle(np, "renesas,rz-sysc");
	if (IS_ERR(pdata->sysc_regmap)) {
		dev_err(dev, "failed to lookup sysc regmap\n");
		return PTR_ERR(pdata->sysc_regmap);
	}

	/* Acquire reset controls (CM33 variants only) */
	for (i = 0; i < data->num_resets; i++) {
		pdata->resets[i] = devm_reset_control_get_exclusive(dev,
						data->reset_names[i]);
		if (IS_ERR(pdata->resets[i])) {
			dev_err(dev, "failed to acquire %s\n",
				data->reset_names[i]);
			return PTR_ERR(pdata->resets[i]);
		}
	}

	/* CM33 core needs boot vector addresses */
	if (pdata->core == RZV2H_CM33_CORE_NUMBER) {
		for (i = 0; i < 2; i++) {
			if (of_property_read_u32_index(np, "renesas,rz-bootaddrs",
							   i, &pdata->bootaddr[i])) {
				dev_err(dev, "invalid boot address\n");
				return -EINVAL;
			}
		}
	}

	rproc->auto_boot = of_property_read_bool(np, "renesas,rz-autoboot");

	pm_runtime_enable(dev);

	/* Detect whether the remote processor is truly running */
	ret = rz_rproc_check_running(pdev, pdata, &running);
	if (ret)
		goto err_pm_disable;

	if (running) {
		/*
		 * Core is fully running (fetch enabled) — started by a prior
		 * boot stage. Attach without reloading firmware.
		 */
		dev_info(dev, "remote core released from reset by bootloader\n");
		rproc->state = RPROC_DETACHED;
		pm_runtime_get_sync(dev);
		rz_rproc_attach_rsc_table(dev, rproc);
	}
	/*
	 * running == false covers two cases for RZ/V2H CM33:
	 *   1. Core never touched — fully offline.
	 *   2. U-Boot handoff: reset deasserted, fetch disabled (CM33_CTL=1).
	 * In both cases state stays RPROC_OFFLINE. Linux loads the ELF and
	 * .start cold-boots the core.
	 */

	platform_set_drvdata(pdev, rproc);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "failed to register rproc\n");
		goto err_pm_put;
	}

	dev_info(dev, "probed (core %u)\n", pdata->core);

	return 0;

err_pm_put:
	if (running)
		pm_runtime_put_sync(dev);
err_pm_disable:
	pm_runtime_disable(dev);

	return ret;
}

static void rz_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id rz_rproc_of_match[] = {
	{ .compatible = "renesas,rzv2h-cm33", .data = &rzv2h_cm33_rproc_data },
	{ .compatible = "renesas,rzv2h-cr8",  .data = &rzv2h_cr8_rproc_data },
	{ .compatible = "renesas,rzg2l-cm33", .data = &rzg2l_cm33_rproc_data },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, rz_rproc_of_match);
static struct platform_driver rz_rproc_driver = {
	.probe  = rz_rproc_probe,
	.remove = rz_rproc_remove,
	.driver = {
		.name = "rz-rproc",
		.of_match_table = rz_rproc_of_match,
	},
};
module_platform_driver(rz_rproc_driver);

MODULE_AUTHOR("Tu Duong <tu.duong.zy@renesas.com>");
MODULE_DESCRIPTION("Renesas RZ remote processor control driver");
MODULE_LICENSE("GPL v2");
