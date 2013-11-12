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
	return ERR_PTR(-EINVAL);
}
