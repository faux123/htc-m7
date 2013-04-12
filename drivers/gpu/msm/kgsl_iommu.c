/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/genalloc.h>
#include <linux/slab.h>
#include <linux/iommu.h>
#include <linux/msm_kgsl.h>
#include <mach/socinfo.h>
#include <mach/msm_iomap.h>
#include <mach/board.h>
#include <stddef.h>

#include "kgsl.h"
#include "kgsl_device.h"
#include "kgsl_mmu.h"
#include "kgsl_sharedmem.h"
#include "kgsl_iommu.h"
#include "adreno_pm4types.h"
#include "adreno.h"
#include "kgsl_trace.h"
#include "z180.h"


struct remote_iommu_petersons_spinlock kgsl_iommu_sync_lock_vars;

static struct kgsl_iommu_unit *get_iommu_unit(struct device *dev)
{
	int i, j, k;

	for (i = 0; i < KGSL_DEVICE_MAX; i++) {
		struct kgsl_mmu *mmu;
		struct kgsl_iommu *iommu;

		if (kgsl_driver.devp[i] == NULL)
			continue;

		mmu = kgsl_get_mmu(kgsl_driver.devp[i]);
		if (mmu == NULL || mmu->priv == NULL)
			continue;

		iommu = mmu->priv;

		for (j = 0; j < iommu->unit_count; j++) {
			struct kgsl_iommu_unit *iommu_unit =
				&iommu->iommu_units[j];
			for (k = 0; k < iommu_unit->dev_count; k++) {
				if (iommu_unit->dev[k].dev == dev)
					return iommu_unit;
			}
		}
	}

	return NULL;
}

static struct kgsl_iommu_device *get_iommu_device(struct kgsl_iommu_unit *unit,
		struct device *dev)
{
	int k;

	for (k = 0; unit && k < unit->dev_count; k++) {
		if (unit->dev[k].dev == dev)
			return &(unit->dev[k]);
	}

	return NULL;
}

static int kgsl_iommu_fault_handler(struct iommu_domain *domain,
	struct device *dev, unsigned long addr, int flags)
{
	struct kgsl_iommu_unit *iommu_unit = get_iommu_unit(dev);
	struct kgsl_iommu_device *iommu_dev = get_iommu_device(iommu_unit, dev);
	unsigned int ptbase, fsr;
	static unsigned long last_pagefault_jiffies;
	static int last_pid;
	int current_pid;
	unsigned long wait_time_jiff = 0;

	if (!iommu_dev) {
		KGSL_CORE_ERR("Invalid IOMMU device %p\n", dev);
		return -ENOSYS;
	}

	wait_time_jiff = last_pagefault_jiffies + msecs_to_jiffies(500);
	last_pagefault_jiffies = jiffies;

	ptbase = KGSL_IOMMU_GET_IOMMU_REG(iommu_unit->reg_map.hostptr,
			iommu_dev->ctx_id, TTBR0);
	current_pid = kgsl_mmu_get_ptname_from_ptbase(ptbase);

	if ((last_pid != current_pid) ||
	    (time_after(jiffies, wait_time_jiff))
	   ) {
		fsr = KGSL_IOMMU_GET_IOMMU_REG(iommu_unit->reg_map.hostptr,
			iommu_dev->ctx_id, FSR);

		KGSL_MEM_CRIT(iommu_dev->kgsldev,
			"GPU PAGE FAULT: addr = %lX pid = %d\n",
			addr, kgsl_mmu_get_ptname_from_ptbase(ptbase));
		KGSL_MEM_CRIT(iommu_dev->kgsldev, "context = %d FSR = %X\n",
			iommu_dev->ctx_id, fsr);

		last_pid = current_pid;
	}

	trace_kgsl_mmu_pagefault(iommu_dev->kgsldev, addr,
			kgsl_mmu_get_ptname_from_ptbase(ptbase), 0);

	return 0;
}

static void kgsl_iommu_disable_clk(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	struct msm_iommu_drvdata *iommu_drvdata;
	int i, j;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (!iommu_unit->dev[j].clk_enabled)
				continue;
			iommu_drvdata = dev_get_drvdata(
					iommu_unit->dev[j].dev->parent);
			if (iommu_drvdata->clk)
				clk_disable_unprepare(iommu_drvdata->clk);
			clk_disable_unprepare(iommu_drvdata->pclk);
			iommu_unit->dev[j].clk_enabled = false;
		}
	}
}

static void kgsl_iommu_clk_disable_event(struct kgsl_device *device, void *data,
					unsigned int id, unsigned int ts)
{
	struct kgsl_mmu *mmu = data;
	struct kgsl_iommu *iommu = mmu->priv;

	if (!iommu->clk_event_queued) {
		if (0 > timestamp_cmp(ts, iommu->iommu_last_cmd_ts))
			KGSL_DRV_ERR(device,
			"IOMMU disable clock event being cancelled, "
			"iommu_last_cmd_ts: %x, retired ts: %x\n",
			iommu->iommu_last_cmd_ts, ts);
		return;
	}

	if (0 <= timestamp_cmp(ts, iommu->iommu_last_cmd_ts)) {
		kgsl_iommu_disable_clk(mmu);
		iommu->clk_event_queued = false;
	} else {
		if (kgsl_add_event(device, id, iommu->iommu_last_cmd_ts,
			kgsl_iommu_clk_disable_event, mmu, mmu)) {
				KGSL_DRV_ERR(device,
				"Failed to add IOMMU disable clk event\n");
				iommu->clk_event_queued = false;
		}
	}
}

static void
kgsl_iommu_disable_clk_on_ts(struct kgsl_mmu *mmu, unsigned int ts,
				bool ts_valid)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (iommu->clk_event_queued) {
		if (ts_valid && (0 <
			timestamp_cmp(ts, iommu->iommu_last_cmd_ts)))
			iommu->iommu_last_cmd_ts = ts;
	} else {
		if (ts_valid) {
			iommu->iommu_last_cmd_ts = ts;
			iommu->clk_event_queued = true;
			if (kgsl_add_event(mmu->device, KGSL_MEMSTORE_GLOBAL,
				ts, kgsl_iommu_clk_disable_event, mmu, mmu)) {
				KGSL_DRV_ERR(mmu->device,
				"Failed to add IOMMU disable clk event\n");
				iommu->clk_event_queued = false;
			}
		} else {
			kgsl_iommu_disable_clk(mmu);
		}
	}
}

static int kgsl_iommu_enable_clk(struct kgsl_mmu *mmu,
				int ctx_id)
{
	int ret = 0;
	int i, j;
	struct kgsl_iommu *iommu = mmu->priv;
	struct msm_iommu_drvdata *iommu_drvdata;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (iommu_unit->dev[j].clk_enabled ||
				ctx_id != iommu_unit->dev[j].ctx_id)
				continue;
			iommu_drvdata =
			dev_get_drvdata(iommu_unit->dev[j].dev->parent);
			ret = clk_prepare_enable(iommu_drvdata->pclk);
			if (ret)
				goto done;
			if (iommu_drvdata->clk) {
				ret = clk_prepare_enable(iommu_drvdata->clk);
				if (ret) {
					clk_disable_unprepare(
						iommu_drvdata->pclk);
					goto done;
				}
			}
			iommu_unit->dev[j].clk_enabled = true;
		}
	}
done:
	if (ret)
		kgsl_iommu_disable_clk(mmu);
	return ret;
}

static int kgsl_iommu_pt_equal(struct kgsl_pagetable *pt,
					unsigned int pt_base)
{
	struct kgsl_iommu_pt *iommu_pt = pt ? pt->priv : NULL;
	unsigned int domain_ptbase = iommu_pt ?
				iommu_get_pt_base_addr(iommu_pt->domain) : 0;
	
	domain_ptbase &= (KGSL_IOMMU_TTBR0_PA_MASK <<
				KGSL_IOMMU_TTBR0_PA_SHIFT);
	pt_base &= (KGSL_IOMMU_TTBR0_PA_MASK <<
				KGSL_IOMMU_TTBR0_PA_SHIFT);
	return domain_ptbase && pt_base &&
		(domain_ptbase == pt_base);
}

static void kgsl_iommu_destroy_pagetable(void *mmu_specific_pt)
{
	struct kgsl_iommu_pt *iommu_pt = mmu_specific_pt;
	if (iommu_pt->domain)
		iommu_domain_free(iommu_pt->domain);
	kfree(iommu_pt);
}

void *kgsl_iommu_create_pagetable(void)
{
	struct kgsl_iommu_pt *iommu_pt;

	iommu_pt = kzalloc(sizeof(struct kgsl_iommu_pt), GFP_KERNEL);
	if (!iommu_pt) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n",
				sizeof(struct kgsl_iommu_pt));
		return NULL;
	}
	iommu_pt->domain = iommu_domain_alloc(&platform_bus_type,
										  MSM_IOMMU_DOMAIN_PT_CACHEABLE);
	if (!iommu_pt->domain) {
		KGSL_CORE_ERR("Failed to create iommu domain\n");
		kfree(iommu_pt);
		return NULL;
	} else {
		iommu_set_fault_handler(iommu_pt->domain,
			kgsl_iommu_fault_handler);
	}

	return iommu_pt;
}

static void kgsl_detach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		iommu_pt = mmu->defaultpagetable->priv;
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (mmu->priv_bank_table &&
				(KGSL_IOMMU_CONTEXT_PRIV == j))
				iommu_pt = mmu->priv_bank_table->priv;
			if (iommu_unit->dev[j].attached) {
				iommu_detach_device(iommu_pt->domain,
						iommu_unit->dev[j].dev);
				iommu_unit->dev[j].attached = false;
				KGSL_MEM_INFO(mmu->device, "iommu %p detached "
					"from user dev of MMU: %p\n",
					iommu_pt->domain, mmu);
			}
		}
	}
}

static int kgsl_attach_pagetable_iommu_domain(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j, ret = 0;

	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		iommu_pt = mmu->defaultpagetable->priv;
		for (j = 0; j < iommu_unit->dev_count; j++) {
			if (mmu->priv_bank_table &&
				(KGSL_IOMMU_CONTEXT_PRIV == j))
				iommu_pt = mmu->priv_bank_table->priv;
			if (!iommu_unit->dev[j].attached) {
				ret = iommu_attach_device(iommu_pt->domain,
							iommu_unit->dev[j].dev);
				if (ret) {
					KGSL_MEM_ERR(mmu->device,
						"Failed to attach device, err %d\n",
						ret);
					goto done;
				}
				iommu_unit->dev[j].attached = true;
				KGSL_MEM_INFO(mmu->device,
				"iommu pt %p attached to dev %p, ctx_id %d\n",
				iommu_pt->domain, iommu_unit->dev[j].dev,
				iommu_unit->dev[j].ctx_id);
			}
		}
	}
done:
	return ret;
}

static int _get_iommu_ctxs(struct kgsl_mmu *mmu,
	struct kgsl_device_iommu_data *data, unsigned int unit_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[unit_id];
	int i;

	if (data->iommu_ctx_count > KGSL_IOMMU_MAX_DEVS_PER_UNIT) {
		KGSL_CORE_ERR("Too many iommu devices defined for an "
				"IOMMU unit\n");
		return -EINVAL;
	}

	for (i = 0; i < data->iommu_ctx_count; i++) {
		if (!data->iommu_ctxs[i].iommu_ctx_name)
			continue;

		iommu_unit->dev[iommu_unit->dev_count].dev =
			msm_iommu_get_ctx(data->iommu_ctxs[i].iommu_ctx_name);
		if (iommu_unit->dev[iommu_unit->dev_count].dev == NULL) {
			KGSL_CORE_ERR("Failed to get iommu dev handle for "
			"device %s\n", data->iommu_ctxs[i].iommu_ctx_name);
			return -EINVAL;
		}
		if (KGSL_IOMMU_CONTEXT_USER != data->iommu_ctxs[i].ctx_id &&
			KGSL_IOMMU_CONTEXT_PRIV != data->iommu_ctxs[i].ctx_id) {
			KGSL_CORE_ERR("Invalid context ID defined: %d\n",
					data->iommu_ctxs[i].ctx_id);
			return -EINVAL;
		}
		iommu_unit->dev[iommu_unit->dev_count].ctx_id =
						data->iommu_ctxs[i].ctx_id;
		iommu_unit->dev[iommu_unit->dev_count].kgsldev = mmu->device;

		KGSL_DRV_INFO(mmu->device,
				"Obtained dev handle %p for iommu context %s\n",
				iommu_unit->dev[iommu_unit->dev_count].dev,
				data->iommu_ctxs[i].iommu_ctx_name);

		iommu_unit->dev_count++;
	}

	return 0;
}

static int kgsl_iommu_init_sync_lock(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	int status = 0;
	struct kgsl_pagetable *pagetable = NULL;
	uint32_t lock_gpu_addr = 0;
	uint32_t lock_phy_addr = 0;
	uint32_t page_offset = 0;

	iommu->sync_lock_initialized = 0;

	if (!(mmu->flags & KGSL_MMU_FLAGS_IOMMU_SYNC)) {
		KGSL_DRV_ERR(mmu->device,
		"The GPU microcode does not support IOMMUv1 sync opcodes\n");
		return -ENXIO;
	}

	
	lock_phy_addr = (msm_iommu_lock_initialize()
			- MSM_SHARED_RAM_BASE + msm_shared_ram_phys);

	if (!lock_phy_addr) {
		KGSL_DRV_ERR(mmu->device,
				"GPU CPU sync lock is not supported by kernel\n");
		return -ENXIO;
	}

	
	page_offset = (lock_phy_addr & (PAGE_SIZE - 1));
	lock_phy_addr = (lock_phy_addr & ~(PAGE_SIZE - 1));
	iommu->sync_lock_desc.physaddr = (unsigned int)lock_phy_addr;

	iommu->sync_lock_desc.size =
				PAGE_ALIGN(sizeof(kgsl_iommu_sync_lock_vars));
	status =  memdesc_sg_phys(&iommu->sync_lock_desc,
				 iommu->sync_lock_desc.physaddr,
				 iommu->sync_lock_desc.size);

	if (status)
		return status;

	
	iommu->sync_lock_desc.priv |= KGSL_MEMFLAGS_GLOBAL;

	pagetable = mmu->priv_bank_table ? mmu->priv_bank_table :
				mmu->defaultpagetable;

	status = kgsl_mmu_map(pagetable, &iommu->sync_lock_desc,
				     GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);

	if (status) {
		kgsl_mmu_unmap(pagetable, &iommu->sync_lock_desc);
		iommu->sync_lock_desc.priv &= ~~KGSL_MEMFLAGS_GLOBAL;
		return status;
	}

	
	lock_gpu_addr = (iommu->sync_lock_desc.gpuaddr + page_offset);

	kgsl_iommu_sync_lock_vars.flag[PROC_APPS] = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock,
			flag[PROC_APPS])));
	kgsl_iommu_sync_lock_vars.flag[PROC_GPU] = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock,
			flag[PROC_GPU])));
	kgsl_iommu_sync_lock_vars.turn = (lock_gpu_addr +
		(offsetof(struct remote_iommu_petersons_spinlock, turn)));

	iommu->sync_lock_vars = &kgsl_iommu_sync_lock_vars;

	
	iommu->sync_lock_initialized = 1;

	return status;
}

inline unsigned int kgsl_iommu_sync_lock(struct kgsl_mmu *mmu,
						unsigned int *cmds)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct remote_iommu_petersons_spinlock *lock_vars =
					iommu->sync_lock_vars;
	unsigned int *start = cmds;

	if (!iommu->sync_lock_initialized)
		return 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 1;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x1;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->turn;
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x1;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	*cmds++ = cp_type3_packet(CP_TEST_TWO_MEMS, 3);
	*cmds++ = lock_vars->flag[PROC_APPS];
	*cmds++ = lock_vars->turn;
	*cmds++ = 0;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - start;
}

inline unsigned int kgsl_iommu_sync_unlock(struct kgsl_mmu *mmu,
					unsigned int *cmds)
{
	struct kgsl_device *device = mmu->device;
	struct adreno_device *adreno_dev = ADRENO_DEVICE(device);
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct remote_iommu_petersons_spinlock *lock_vars =
						iommu->sync_lock_vars;
	unsigned int *start = cmds;

	if (!iommu->sync_lock_initialized)
		return 0;

	*cmds++ = cp_type3_packet(CP_MEM_WRITE, 2);
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0;

	*cmds++ = cp_type3_packet(CP_WAIT_REG_MEM, 5);
	
	*cmds++ = 0x13;
	*cmds++ = lock_vars->flag[PROC_GPU];
	*cmds++ = 0x0;
	*cmds++ = 0x1;
	*cmds++ = 0x1;

	cmds += adreno_add_idle_cmds(adreno_dev, cmds);

	return cmds - start;
}

static int kgsl_get_iommu_ctxt(struct kgsl_mmu *mmu)
{
	struct platform_device *pdev =
		container_of(mmu->device->parentdev, struct platform_device,
				dev);
	struct kgsl_device_platform_data *pdata_dev = pdev->dev.platform_data;
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	int i, ret = 0;

	
	if (KGSL_IOMMU_MAX_UNITS < pdata_dev->iommu_count) {
		KGSL_CORE_ERR("Too many IOMMU units defined\n");
		ret = -EINVAL;
		goto  done;
	}

	for (i = 0; i < pdata_dev->iommu_count; i++) {
		ret = _get_iommu_ctxs(mmu, &pdata_dev->iommu_data[i], i);
		if (ret)
			break;
	}
	iommu->unit_count = pdata_dev->iommu_count;
done:
	return ret;
}

static int kgsl_set_register_map(struct kgsl_mmu *mmu)
{
	struct platform_device *pdev =
		container_of(mmu->device->parentdev, struct platform_device,
				dev);
	struct kgsl_device_platform_data *pdata_dev = pdev->dev.platform_data;
	struct kgsl_iommu *iommu = mmu->device->mmu.priv;
	struct kgsl_iommu_unit *iommu_unit;
	int i = 0, ret = 0;

	for (; i < pdata_dev->iommu_count; i++) {
		struct kgsl_device_iommu_data data = pdata_dev->iommu_data[i];
		iommu_unit = &iommu->iommu_units[i];
		
		if (!data.physstart || !data.physend) {
			KGSL_CORE_ERR("The register range for IOMMU unit not"
					" specified\n");
			ret = -EINVAL;
			goto err;
		}
		iommu_unit->reg_map.hostptr = ioremap(data.physstart,
					data.physend - data.physstart + 1);
		if (!iommu_unit->reg_map.hostptr) {
			KGSL_CORE_ERR("Failed to map SMMU register address "
				"space from %x to %x\n", data.physstart,
				data.physend - data.physstart + 1);
			ret = -ENOMEM;
			i--;
			goto err;
		}
		iommu_unit->reg_map.size = data.physend - data.physstart + 1;
		iommu_unit->reg_map.physaddr = data.physstart;
		memdesc_sg_phys(&iommu_unit->reg_map, data.physstart,
				iommu_unit->reg_map.size);
	}
	iommu->unit_count = pdata_dev->iommu_count;
	return ret;
err:
	
	for (; i >= 0; i--) {
		iommu_unit = &iommu->iommu_units[i];
		iounmap(iommu_unit->reg_map.hostptr);
		iommu_unit->reg_map.size = 0;
		iommu_unit->reg_map.physaddr = 0;
	}
	return ret;
}

static unsigned int kgsl_iommu_pt_get_base_addr(struct kgsl_pagetable *pt)
{
	struct kgsl_iommu_pt *iommu_pt = pt->priv;
	return iommu_get_pt_base_addr(iommu_pt->domain);
}

static int kgsl_iommu_get_pt_lsb(struct kgsl_mmu *mmu,
				unsigned int unit_id,
				enum kgsl_iommu_context_id ctx_id)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++)
			if (unit_id == i &&
				ctx_id == iommu_unit->dev[j].ctx_id)
				return iommu_unit->dev[j].pt_lsb;
	}
	return 0;
}

static void kgsl_iommu_setstate(struct kgsl_mmu *mmu,
				struct kgsl_pagetable *pagetable,
				unsigned int context_id)
{
	if (mmu->flags & KGSL_FLAGS_STARTED) {
		if (mmu->hwpagetable != pagetable) {
			unsigned int flags = 0;
			mmu->hwpagetable = pagetable;
			flags |= kgsl_mmu_pt_get_flags(mmu->hwpagetable,
							mmu->device->id) |
							KGSL_MMUFLAGS_TLBFLUSH;
			kgsl_setstate(mmu, context_id,
				KGSL_MMUFLAGS_PTUPDATE | flags);
		}
	}
}

static int kgsl_iommu_init(struct kgsl_mmu *mmu)
{
	int status = 0;
	struct kgsl_iommu *iommu;

	iommu = kzalloc(sizeof(struct kgsl_iommu), GFP_KERNEL);
	if (!iommu) {
		KGSL_CORE_ERR("kzalloc(%d) failed\n",
				sizeof(struct kgsl_iommu));
		return -ENOMEM;
	}

	mmu->priv = iommu;
	status = kgsl_get_iommu_ctxt(mmu);
	if (status)
		goto done;
	status = kgsl_set_register_map(mmu);
	if (status)
		goto done;

	kgsl_sharedmem_writel(&mmu->setstate_memory,
				KGSL_IOMMU_SETSTATE_NOP_OFFSET,
				cp_nop_packet(1));

	dev_info(mmu->device->dev, "|%s| MMU type set for device is IOMMU\n",
			__func__);
done:
	if (status) {
		kfree(iommu);
		mmu->priv = NULL;
	}
	return status;
}

static int kgsl_iommu_setup_defaultpagetable(struct kgsl_mmu *mmu)
{
	int status = 0;
	int i = 0;
	struct kgsl_iommu *iommu = mmu->priv;
	struct kgsl_iommu_pt *iommu_pt;
	struct kgsl_pagetable *pagetable = NULL;

	if (!cpu_is_msm8960()) {
		mmu->priv_bank_table =
			kgsl_mmu_getpagetable(KGSL_MMU_PRIV_BANK_TABLE_NAME);
		if (mmu->priv_bank_table == NULL) {
			status = -ENOMEM;
			goto err;
		}
		iommu_pt = mmu->priv_bank_table->priv;
	}
	mmu->defaultpagetable = kgsl_mmu_getpagetable(KGSL_MMU_GLOBAL_PT);
	
	if (mmu->defaultpagetable == NULL) {
		status = -ENOMEM;
		goto err;
	}
	pagetable = mmu->priv_bank_table ? mmu->priv_bank_table :
				mmu->defaultpagetable;
	
	for (i = 0; i < iommu->unit_count; i++) {
		iommu->iommu_units[i].reg_map.priv |= KGSL_MEMFLAGS_GLOBAL;
		status = kgsl_mmu_map(pagetable,
			&(iommu->iommu_units[i].reg_map),
			GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);
		if (status) {
			iommu->iommu_units[i].reg_map.priv &=
							~KGSL_MEMFLAGS_GLOBAL;
			goto err;
		}
	}
	return status;
err:
	for (i--; i >= 0; i--) {
		kgsl_mmu_unmap(pagetable,
				&(iommu->iommu_units[i].reg_map));
		iommu->iommu_units[i].reg_map.priv &= ~KGSL_MEMFLAGS_GLOBAL;
	}
	if (mmu->priv_bank_table) {
		kgsl_mmu_putpagetable(mmu->priv_bank_table);
		mmu->priv_bank_table = NULL;
	}
	if (mmu->defaultpagetable) {
		kgsl_mmu_putpagetable(mmu->defaultpagetable);
		mmu->defaultpagetable = NULL;
	}
	return status;
}

static int kgsl_iommu_start(struct kgsl_mmu *mmu)
{
	struct kgsl_device *device = mmu->device;
	int status;
	struct kgsl_iommu *iommu = mmu->priv;
	int i, j;

	if (mmu->flags & KGSL_FLAGS_STARTED)
		return 0;

	if (mmu->defaultpagetable == NULL) {
		status = kgsl_iommu_setup_defaultpagetable(mmu);
		if (status)
			return -ENOMEM;

		
		if (msm_soc_version_supports_iommu_v1() &&
			(device->id == KGSL_DEVICE_3D0))
				kgsl_iommu_init_sync_lock(mmu);
	}

	if (cpu_is_msm8960()) {
		struct kgsl_mh *mh = &(mmu->device->mh);
		kgsl_regwrite(mmu->device, MH_MMU_CONFIG, 0x00000001);
		kgsl_regwrite(mmu->device, MH_MMU_MPU_END,
			mh->mpu_base +
			iommu->iommu_units
				[iommu->unit_count - 1].reg_map.gpuaddr -
				PAGE_SIZE);
	} else {
		kgsl_regwrite(mmu->device, MH_MMU_CONFIG, 0x00000000);
	}

	mmu->hwpagetable = mmu->defaultpagetable;

	status = kgsl_attach_pagetable_iommu_domain(mmu);
	if (status) {
		mmu->hwpagetable = NULL;
		goto done;
	}
	status = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	if (status) {
		KGSL_CORE_ERR("clk enable failed\n");
		goto done;
	}
	status = kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_PRIV);
	if (status) {
		KGSL_CORE_ERR("clk enable failed\n");
		goto done;
	}
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_iommu_unit *iommu_unit = &iommu->iommu_units[i];
		for (j = 0; j < iommu_unit->dev_count; j++)
			iommu_unit->dev[j].pt_lsb = KGSL_IOMMMU_PT_LSB(
						KGSL_IOMMU_GET_IOMMU_REG(
						iommu_unit->reg_map.hostptr,
						iommu_unit->dev[j].ctx_id,
						TTBR0));
	}

	kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
	mmu->flags |= KGSL_FLAGS_STARTED;

done:
	if (status) {
		kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
		kgsl_detach_pagetable_iommu_domain(mmu);
	}
	return status;
}

static int
kgsl_iommu_unmap(void *mmu_specific_pt,
		struct kgsl_memdesc *memdesc,
		unsigned int *tlb_flags)
{
	int ret;
	unsigned int range = kgsl_sg_size(memdesc->sg, memdesc->sglen);
	struct kgsl_iommu_pt *iommu_pt = mmu_specific_pt;


	unsigned int gpuaddr = memdesc->gpuaddr &  KGSL_MMU_ALIGN_MASK;

	if (range == 0 || gpuaddr == 0)
		return 0;

	ret = iommu_unmap_range(iommu_pt->domain, gpuaddr, range);
	if (ret)
		KGSL_CORE_ERR("iommu_unmap_range(%p, %x, %d) failed "
			"with err: %d\n", iommu_pt->domain, gpuaddr,
			range, ret);

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	if (!ret)
		*tlb_flags = UINT_MAX;
#endif
	return 0;
}

static int
kgsl_iommu_map(void *mmu_specific_pt,
			struct kgsl_memdesc *memdesc,
			unsigned int protflags,
			unsigned int *tlb_flags)
{
	int ret;
	unsigned int iommu_virt_addr;
	struct kgsl_iommu_pt *iommu_pt = mmu_specific_pt;
	int size = kgsl_sg_size(memdesc->sg, memdesc->sglen);

	BUG_ON(NULL == iommu_pt);


	iommu_virt_addr = memdesc->gpuaddr;

	ret = iommu_map_range(iommu_pt->domain, iommu_virt_addr, memdesc->sg,
				size, (IOMMU_READ | IOMMU_WRITE));
	if (ret) {
		KGSL_CORE_ERR("iommu_map_range(%p, %x, %p, %d, %d) "
				"failed with err: %d\n", iommu_pt->domain,
				iommu_virt_addr, memdesc->sg, size,
				(IOMMU_READ | IOMMU_WRITE), ret);
		return ret;
	}

	return ret;
}

static void kgsl_iommu_stop(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;

	if (mmu->flags & KGSL_FLAGS_STARTED) {
		kgsl_regwrite(mmu->device, MH_MMU_CONFIG, 0x00000000);
		
		kgsl_detach_pagetable_iommu_domain(mmu);
		mmu->hwpagetable = NULL;

		mmu->flags &= ~KGSL_FLAGS_STARTED;
	}

	
	iommu->clk_event_queued = false;
	kgsl_cancel_events(mmu->device, mmu);
	kgsl_iommu_disable_clk(mmu);
}

static int kgsl_iommu_close(struct kgsl_mmu *mmu)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int i;
	for (i = 0; i < iommu->unit_count; i++) {
		struct kgsl_pagetable *pagetable = (mmu->priv_bank_table ?
			mmu->priv_bank_table : mmu->defaultpagetable);
		if (iommu->iommu_units[i].reg_map.gpuaddr)
			kgsl_mmu_unmap(pagetable,
			&(iommu->iommu_units[i].reg_map));
		if (iommu->iommu_units[i].reg_map.hostptr)
			iounmap(iommu->iommu_units[i].reg_map.hostptr);
		kgsl_sg_free(iommu->iommu_units[i].reg_map.sg,
				iommu->iommu_units[i].reg_map.sglen);
	}

	if (mmu->priv_bank_table)
		kgsl_mmu_putpagetable(mmu->priv_bank_table);
	if (mmu->defaultpagetable)
		kgsl_mmu_putpagetable(mmu->defaultpagetable);
	kfree(iommu);

	return 0;
}

static unsigned int
kgsl_iommu_get_current_ptbase(struct kgsl_mmu *mmu)
{
	unsigned int pt_base;
	struct kgsl_iommu *iommu = mmu->priv;
	if (in_interrupt())
		return 0;
	
	kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER);
	pt_base = readl_relaxed(iommu->iommu_units[0].reg_map.hostptr +
			(KGSL_IOMMU_CONTEXT_USER << KGSL_IOMMU_CTX_SHIFT) +
			KGSL_IOMMU_TTBR0);
	kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
	return pt_base & (KGSL_IOMMU_TTBR0_PA_MASK <<
				KGSL_IOMMU_TTBR0_PA_SHIFT);
}

static void kgsl_iommu_default_setstate(struct kgsl_mmu *mmu,
					uint32_t flags)
{
	struct kgsl_iommu *iommu = mmu->priv;
	int temp;
	int i;
	unsigned int pt_base = kgsl_iommu_pt_get_base_addr(
					mmu->hwpagetable);
	unsigned int pt_val;

	if (kgsl_iommu_enable_clk(mmu, KGSL_IOMMU_CONTEXT_USER)) {
		KGSL_DRV_ERR(mmu->device, "Failed to enable iommu clocks\n");
		return;
	}
	
	pt_base &= (KGSL_IOMMU_TTBR0_PA_MASK << KGSL_IOMMU_TTBR0_PA_SHIFT);

	
	if (msm_soc_version_supports_iommu_v1())
		kgsl_idle(mmu->device);

	
	msm_iommu_lock();

	if (flags & KGSL_MMUFLAGS_PTUPDATE) {
		if (!msm_soc_version_supports_iommu_v1())
			kgsl_idle(mmu->device);
		for (i = 0; i < iommu->unit_count; i++) {
			pt_val = kgsl_iommu_get_pt_lsb(mmu, i,
						KGSL_IOMMU_CONTEXT_USER);
			pt_val += pt_base;

			KGSL_IOMMU_SET_IOMMU_REG(
				iommu->iommu_units[i].reg_map.hostptr,
				KGSL_IOMMU_CONTEXT_USER, TTBR0, pt_val);

			mb();
			temp = KGSL_IOMMU_GET_IOMMU_REG(
				iommu->iommu_units[i].reg_map.hostptr,
				KGSL_IOMMU_CONTEXT_USER, TTBR0);
		}
	}
	
	if (flags & KGSL_MMUFLAGS_TLBFLUSH) {
		for (i = 0; i < iommu->unit_count; i++) {
			KGSL_IOMMU_SET_IOMMU_REG(
				iommu->iommu_units[i].reg_map.hostptr,
				KGSL_IOMMU_CONTEXT_USER, CTX_TLBIALL,
				1);
			mb();
		}
	}

	
	msm_iommu_unlock();

	
	kgsl_iommu_disable_clk_on_ts(mmu, 0, false);
}

static int kgsl_iommu_get_reg_map_desc(struct kgsl_mmu *mmu,
					void **reg_map_desc)
{
	struct kgsl_iommu *iommu = mmu->priv;
	void **reg_desc_ptr;
	int i;

	reg_desc_ptr = kmalloc(iommu->unit_count *
			sizeof(struct kgsl_memdesc *), GFP_KERNEL);
	if (!reg_desc_ptr) {
		KGSL_CORE_ERR("Failed to kmalloc(%d)\n",
			iommu->unit_count * sizeof(struct kgsl_memdesc *));
		return -ENOMEM;
	}

	for (i = 0; i < iommu->unit_count; i++)
		reg_desc_ptr[i] = &(iommu->iommu_units[i].reg_map);

	*reg_map_desc = reg_desc_ptr;
	return i;
}

struct kgsl_mmu_ops iommu_ops = {
	.mmu_init = kgsl_iommu_init,
	.mmu_close = kgsl_iommu_close,
	.mmu_start = kgsl_iommu_start,
	.mmu_stop = kgsl_iommu_stop,
	.mmu_setstate = kgsl_iommu_setstate,
	.mmu_device_setstate = kgsl_iommu_default_setstate,
	.mmu_pagefault = NULL,
	.mmu_get_current_ptbase = kgsl_iommu_get_current_ptbase,
	.mmu_enable_clk = kgsl_iommu_enable_clk,
	.mmu_disable_clk_on_ts = kgsl_iommu_disable_clk_on_ts,
	.mmu_get_pt_lsb = kgsl_iommu_get_pt_lsb,
	.mmu_get_reg_map_desc = kgsl_iommu_get_reg_map_desc,
	.mmu_sync_lock = kgsl_iommu_sync_lock,
	.mmu_sync_unlock = kgsl_iommu_sync_unlock,
};

struct kgsl_mmu_pt_ops iommu_pt_ops = {
	.mmu_map = kgsl_iommu_map,
	.mmu_unmap = kgsl_iommu_unmap,
	.mmu_create_pagetable = kgsl_iommu_create_pagetable,
	.mmu_destroy_pagetable = kgsl_iommu_destroy_pagetable,
	.mmu_pt_equal = kgsl_iommu_pt_equal,
	.mmu_pt_get_base_addr = kgsl_iommu_pt_get_base_addr,
};
