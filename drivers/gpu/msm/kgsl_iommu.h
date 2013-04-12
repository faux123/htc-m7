/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
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
#ifndef __KGSL_IOMMU_H
#define __KGSL_IOMMU_H

#include <mach/iommu.h>

#define KGSL_IOMMU_TTBR0			0x10
#define KGSL_IOMMU_TTBR1			0x14
#define KGSL_IOMMU_FSR				0x20

#define KGSL_IOMMU_TTBR0_PA_MASK		0x0003FFFF
#define KGSL_IOMMU_TTBR0_PA_SHIFT		14
#define KGSL_IOMMU_CTX_TLBIALL			0x800
#define KGSL_IOMMU_CTX_SHIFT			12

#define KGSL_IOMMU_MAX_UNITS 2

#define KGSL_IOMMU_MAX_DEVS_PER_UNIT 2

#define KGSL_IOMMU_SET_IOMMU_REG(base_addr, ctx, REG, val)		\
		writel_relaxed(val, base_addr +				\
				(ctx << KGSL_IOMMU_CTX_SHIFT) +		\
				KGSL_IOMMU_##REG)

#define KGSL_IOMMU_GET_IOMMU_REG(base_addr, ctx, REG)			\
		readl_relaxed(base_addr +				\
			(ctx << KGSL_IOMMU_CTX_SHIFT) +			\
			KGSL_IOMMU_##REG)

#define KGSL_IOMMMU_PT_LSB(pt_val)					\
		(pt_val & ~(KGSL_IOMMU_TTBR0_PA_MASK <<			\
				KGSL_IOMMU_TTBR0_PA_SHIFT))

#define KGSL_IOMMU_SETSTATE_NOP_OFFSET	1024

struct kgsl_iommu_device {
	struct device *dev;
	bool attached;
	unsigned int pt_lsb;
	enum kgsl_iommu_context_id ctx_id;
	bool clk_enabled;
	struct kgsl_device *kgsldev;
};

struct kgsl_iommu_unit {
	struct kgsl_iommu_device dev[KGSL_IOMMU_MAX_DEVS_PER_UNIT];
	unsigned int dev_count;
	struct kgsl_memdesc reg_map;
};

struct kgsl_iommu {
	struct kgsl_iommu_unit iommu_units[KGSL_IOMMU_MAX_UNITS];
	unsigned int unit_count;
	unsigned int iommu_last_cmd_ts;
	bool clk_event_queued;
	struct kgsl_device *device;
	struct remote_iommu_petersons_spinlock *sync_lock_vars;
	struct kgsl_memdesc sync_lock_desc;
	bool sync_lock_initialized;
};

struct kgsl_iommu_pt {
	struct iommu_domain *domain;
	struct kgsl_iommu *iommu;
};

#endif
