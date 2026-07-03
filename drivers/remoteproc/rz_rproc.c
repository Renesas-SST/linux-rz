// SPDX-License-Identifier: GPL-2.0

#include <linux/cleanup.h>
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
#define RZV2H_CPG_LP_CM33_CTL0		0xD2C
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

/* RZ/V2H CR8 TCM mapping */
#define RZV2H_CR8_CORE0_ITCM_AXI_START	0x12040000
#define RZV2H_CR8_CORE1_ITCM_AXI_START	0x12080000
#define RZV2H_CR8_CORE_TCM_MAP_SIZE	0x00040000
#define RZV2H_RESET_CTRL_READY		BIT(4)
#define RZV2H_RESET_RELEASEREQ		BIT(3)

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
};

/*
 * RZ/V2H CR8 cluster state.
 *
 * The two CR8 cores share the same cluster hardware (clocks and reset). Per
 * the hardware manual, "The CR8 does not support per-core reset" - reset and
 * clock control are cluster-level. The only per-core control is nCPUHALT
 * (CR8_CONFIG1 BIT(0)/BIT(1)).
 *
 * A refcount tracks how many CR8 cores are running so the shared cluster is
 * brought up on the first core start (0 -> 1) and torn down only on the last
 * core stop (1 -> 0). It is protected by a mutex to avoid races when the two
 * cores are started/stopped concurrently.
 */
static DEFINE_MUTEX(rzv2h_cr8_cluster_lock);
static unsigned int rzv2h_cr8_cluster_refcnt;

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

		if (strcmp(it.node->name, "vdev0buffer")) {
			mem = rproc_mem_entry_init(dev, NULL, rmem->base,
						   rmem->size, da,
						   rz_rproc_mem_alloc,
						   rz_rproc_mem_release,
						   it.node->name);
		} else {
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   rmem->base,
							   it.node->name);
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
	int ret;

	ret = rz_rproc_add_carveouts(rproc);
	if (ret)
		return ret;

	/* If the remote core is already running (DETACHED), skip startup */
	if (rproc->state == RPROC_DETACHED)
		dev_info(dev, "remote core already running, skip startup\n");

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

static int rzv2h_cm33_startup(struct rproc *rproc)
{
	struct device *dev = rproc->dev.parent;
	struct rz_rproc_pdata *pdata = rproc->priv;
	u32 clkmon, rstmon;

	/* Initialize SRAM/DDR configuration for CM33 */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CM33_CTL0, 0x02000000);

	/* Check CM33 clock status */
	regmap_read(pdata->cpg_regmap, RZV2H_CPG_CLKMON_0, &clkmon);

	/* Ensure CM33 is in reset */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380000);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_RSTMON_0, &rstmon);
	} while ((rstmon & 0x000E0000) != 0x000E0000);

	/* If clock was already on, disable it first to ensure clean reset */
	if (clkmon & RZV2H_CPG_CLKON_1_CLK2_ON_MASK) {
		regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00040000);
		do {
			regmap_read(pdata->cpg_regmap, RZV2H_CPG_CLKMON_0, &clkmon);
		} while (clkmon & RZV2H_CPG_CLKON_1_CLK2_ON_MASK);
		dev_info(dev, "CM33 clock disabled for clean initialization\n");
	}

	regmap_write(pdata->sysc_regmap, RZV2H_SYS_MCPU_CFG2, pdata->bootaddr[0]);
	regmap_write(pdata->sysc_regmap, RZV2H_SYS_MCPU_CFG3, pdata->bootaddr[1]);
	dev_info(dev, "CM33 bootaddr secure=0x%08x non-secure=0x%08x\n",
		 pdata->bootaddr[0], pdata->bootaddr[1]);

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00040004);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_CLKMON_0, &clkmon);
	} while ((clkmon & RZV2H_CPG_CLKON_1_CLK2_ON_MASK) == 0);

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CM33_CTL1, 0x00003100);

	/*
	 * Disable fetch (CM33_CTL bit[0] = 1) before releasing reset.
	 * CM33 will be out of reset but not executing — the remoteproc
	 * framework loads the ELF after start() returns. Fetch is released
	 * in rz_rproc_loaded() only after the ELF is fully in memory.
	 *
	 * Without this, CM33 starts fetching immediately after reset release
	 * before the ELF is loaded → falls into SCIF download mode.
	 */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CM33_CTL, 0x00000001);

	/* Two-step reset release sequence */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380008);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_RSTMON_0, &rstmon);
	} while ((rstmon & 0x000E0000) != 0x000C0000);

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380038);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_RSTMON_0, &rstmon);
	} while (rstmon & 0x000E0000);

	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CM33_CTL, 0x00000000);
	dev_info(dev, "CM33 fetch enabled, core is running\n");

	return 0;
}

static int rzv2h_stop_cm33(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *carveout;
	u32 rstmon, clkmon;

	/* Put CM33 back into reset before gating its clock */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_1, 0x00380000);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_RSTMON_0, &rstmon);
	} while ((rstmon & 0x000E0000) != 0x000E0000);

	/* Disable fetch */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CM33_CTL, 0x00000001);

	/* Gate CM33 clock */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00040000);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_CLKMON_0, &clkmon);
	} while (clkmon & RZV2H_CPG_CLKON_1_CLK2_ON_MASK);

	/* Clear registered carveouts after the core is quiesced */
	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if (!carveout->va)
			continue;
		memset(carveout->va, 0, carveout->len);
	}

	return 0;
}

/* ================================================================== */
/* RZ/V2H CR8 startup / shutdown                                      */
/* ================================================================== */

static int rzv2h_cr8_startup(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	u32 val;

	guard(mutex)(&rzv2h_cr8_cluster_lock);

	/* Cluster already brought up by the other core */
	if (rzv2h_cr8_cluster_refcnt++)
		return 0;

	/* Assert MSTOP for CR8 bus */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_BUS_10_MSTOP, 0x04000000);

	/* Set CR8 Clock to ON */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_0, 0xE000E000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00030003);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_CLKMON_0, &val);
	} while ((val & 0x0003E000) == 0);

	/* Reset all CR8 resets */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x1FFF0000);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_RSTMON_0, &val);
	} while ((val & 0xFFF00000) != 0xFFF00000);
	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_RSTMON_1, &val);
	} while ((val & 0x1) != 0x1);

	/* Configure debug mode for CR8 */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL3, 0x003F0000);

	/* Set nCPUHALT to 00b (halt both CR8 CPUs at cluster bring-up) */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1, 0x00000000);

	/* Release cold reset for CR8 */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x10001000);

	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL4, &val);
	} while (!(val & RZV2H_RESET_CTRL_READY));

	/* Trigger reset release sequence */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL4, 0x00000020);

	do {
		regmap_read(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL4, &val);
	} while (!(val & RZV2H_RESET_RELEASEREQ));

	/* Release all CR8 resets */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x1FFF1FFF);

	/* Clear the RESET_TRIG */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_LP_CR8_CTL4, 0x00000000);

	return 0;
}

static int rzv2h_cr8_startup_and_config(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct device *dev = rproc->dev.parent;
	u32 val;
	int ret;

	ret = rzv2h_cr8_startup(rproc);
	if (ret) {
		dev_err(dev, "CR8 startup failed: %d\n", ret);
		return ret;
	}

	/* Set nCPUHALT to run this specific CR8 core (per-core control) */
	regmap_read(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1, &val);
	if (pdata->core == RZV2H_CR8_CORE0_NUMBER)
		val |= BIT(0);
	else if (pdata->core == RZV2H_CR8_CORE1_NUMBER)
		val |= BIT(1);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1, val);

	return 0;
}

static int rzv2h_stop_cr8(struct rproc *rproc)
{
	struct rz_rproc_pdata *pdata = rproc->priv;
	struct rproc_mem_entry *carveout;

	guard(mutex)(&rzv2h_cr8_cluster_lock);

	if (WARN_ON(rzv2h_cr8_cluster_refcnt == 0))
		return 0;

	/*
	 * Per-core control is limited to nCPUHALT (CR8_CONFIG1). The CR8 does
	 * not support per-core reset/clock gating, so halt only this core's
	 * CPU and leave the shared cluster running if the other core is up.
	 */
	if (pdata->core == RZV2H_CR8_CORE0_NUMBER)
		regmap_update_bits(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1,
				   BIT(0), 0);
	else if (pdata->core == RZV2H_CR8_CORE1_NUMBER)
		regmap_update_bits(pdata->cpg_regmap, RZV2H_CPG_CR8_CONFIG1,
				   BIT(1), 0);

	/* Clear this core's carveouts (keep shared/TCM regions intact) */
	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if (!carveout->va)
			continue;
		if (strstr(carveout->name, "tcm"))
			continue;
		memset(carveout->va, 0, carveout->len);
	}

	/*
	 * Reset/clock are cluster-level, so only the last core tears down
	 * the shared cluster.
	 */
	if (--rzv2h_cr8_cluster_refcnt)
		return 0;

	/* Assert cluster reset, then gate cluster clocks */
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_RST_2, 0x1FFF0000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_1, 0x00030000);
	regmap_write(pdata->cpg_regmap, RZV2H_CPG_CLKON_0, 0xE0000000);

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
		return rzv2h_cr8_startup_and_config(rproc);

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
	do {
		regmap_read(pdata->cpg_regmap, RZG2L_CPG_CLKMON_CM33, &val);
	} while ((val & RZG2L_CPG_CLKON_CM33_CLK0_ON_MASK) == 0);

	regmap_write(pdata->cpg_regmap, RZG2L_CPG_RST_CM33, 0x00040004);
	regmap_write(pdata->cpg_regmap, RZG2L_CPG_RST_CM33, 0x00070007);
	do {
		regmap_read(pdata->cpg_regmap, RZG2L_CPG_RSTMON_CM33, &val);
	} while (val & 0x00000007);

	return 0;
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
			rzv2h_cr8_cluster_refcnt = 1;
		}
		return 0;
	}

	/*
	 * RZ/V2H CM33: check both RSTMON and CM33_CTL.
	 * U-Boot handoff leaves: RSTMON=0 (reset deasserted) + CM33_CTL=1
	 * (fetch disabled). That is NOT running — Linux must load firmware
	 * and release fetch via rz_rproc_loaded().
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
	 * rz_rproc_loaded() releases fetch after load completes.
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
	.probe	= rz_rproc_probe,
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
