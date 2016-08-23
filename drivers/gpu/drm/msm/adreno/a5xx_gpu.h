/*
 * Copyright (C) 2016 Red Hat
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

#ifndef __A5XX_GPU_H__
#define __A5XX_GPU_H__

#include "adreno_gpu.h"

/* arrg, somehow fb.h is getting pulled in: */
#undef ROP_COPY
#undef ROP_XOR

#include "a5xx.xml.h"

struct a5xx_gpu {
	struct adreno_gpu base;
	struct platform_device *pdev;

	/* pfp/pm4 fw runs out of external memory, so we've got to stick 'em
	 * in BOs.
	 */
	struct drm_gem_object *pfp_bo, *pm4_bo;
	uint64_t pfp_iova, pm4_iova;
};
#define to_a5xx_gpu(x) container_of(x, struct a5xx_gpu, base)

#endif /* __A5XX_GPU_H__ */
