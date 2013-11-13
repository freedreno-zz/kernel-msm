/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "msm_drv.h"
#include "msm_mmu.h"
#include "msm_gpu.h"

// hmm, technically I think the MH_MMU_* registers are
// common between z180 and adreno..
#include "adreno/adreno_common.xml.h"

struct msm_gpummu {
	struct msm_mmu base;
	struct msm_gpu *gpu;
};
#define to_msm_gpummu(x) container_of(x, struct msm_gpummu, base)


static int msm_gpummu_attach(struct msm_mmu *mmu, const char **names, int cnt)
{
	return 0;
}

static int msm_gpummu_map(struct msm_mmu *mmu, uint32_t iova,
		struct sg_table *sgt, unsigned len, int prot)
{
	return -1;
}

static int msm_gpummu_unmap(struct msm_mmu *mmu, uint32_t iova,
		struct sg_table *sgt, unsigned len)
{
	return -1;
}

static void msm_gpummu_destroy(struct msm_mmu *mmu)
{
	struct msm_gpummu *gpummu = to_msm_gpummu(mmu);
	kfree(gpummu);
}

static const struct msm_mmu_funcs funcs = {
		.attach = msm_gpummu_attach,
		.map = msm_gpummu_map,
		.unmap = msm_gpummu_unmap,
		.destroy = msm_gpummu_destroy,
};

struct msm_mmu *msm_gpummu_new(struct drm_device *dev, struct msm_gpu *gpu)
{
	struct msm_gpummu *gpummu;

	gpummu = kzalloc(sizeof(*gpummu), GFP_KERNEL);
	if (!gpummu)
		return ERR_PTR(-ENOMEM);

	gpummu->gpu = gpu;
	msm_mmu_init(&gpummu->base, dev, &funcs);

	// XXX
	msm_gpu_pm_resume(gpu);
	{
		uint32_t val;
#define MH_INTERRUPT_MASK            0x0A42
#define MH_INTERRUPT_STATUS          0x0A43
#define MH_INTERRUPT_CLEAR           0x0A44
#define MH_AXI_ERROR                 0x0A45
#define MH_ARBITER_CONFIG            0x0A40
#define MH_DEBUG_CTRL                0x0A4E
#define MH_DEBUG_DATA                0x0A4F
#define MH_AXI_HALT_CONTROL          0x0A50
#define MH_CLNT_INTF_CTRL_CONFIG1    0x0A54
#define MH_CLNT_INTF_CTRL_CONFIG2    0x0A55
		val = gpu_read(gpu, REG_AXXX_MH_MMU_CONFIG);
		DBG("MH_MMU_CONFIG: %08x", val);

		gpu_write(gpu, REG_AXXX_MH_MMU_CONFIG, 1);
		val = gpu_read(gpu, REG_AXXX_MH_MMU_CONFIG);
		DBG("MH_MMU_CONFIG: %08x", val);

DBG("MH_INTERRUPT_MASK        : %08x", gpu_read(gpu, MH_INTERRUPT_MASK        ));
DBG("MH_INTERRUPT_STATUS      : %08x", gpu_read(gpu, MH_INTERRUPT_STATUS      ));
DBG("MH_INTERRUPT_CLEAR       : %08x", gpu_read(gpu, MH_INTERRUPT_CLEAR       ));
DBG("MH_AXI_ERROR             : %08x", gpu_read(gpu, MH_AXI_ERROR             ));
DBG("MH_ARBITER_CONFIG        : %08x", gpu_read(gpu, MH_ARBITER_CONFIG        ));
DBG("MH_DEBUG_CTRL            : %08x", gpu_read(gpu, MH_DEBUG_CTRL            ));
DBG("MH_DEBUG_DATA            : %08x", gpu_read(gpu, MH_DEBUG_DATA            ));
DBG("MH_AXI_HALT_CONTROL      : %08x", gpu_read(gpu, MH_AXI_HALT_CONTROL      ));
DBG("MH_CLNT_INTF_CTRL_CONFIG1: %08x", gpu_read(gpu, MH_CLNT_INTF_CTRL_CONFIG1));
DBG("MH_CLNT_INTF_CTRL_CONFIG2: %08x", gpu_read(gpu, MH_CLNT_INTF_CTRL_CONFIG2));

	}
	msm_gpu_pm_suspend(gpu);

	return &gpummu->base;
}
