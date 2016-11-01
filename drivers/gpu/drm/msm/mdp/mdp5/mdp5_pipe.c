/*
 * Copyright (C) 2014-2015 The Linux Foundation. All rights reserved.
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

#include "mdp5_kms.h"

struct mdp5_hw_pipe *mdp5_pipe_assign(struct mdp5_kms *mdp5_kms,
		struct drm_plane *plane, uint32_t caps)
{
	struct mdp5_hw_pipe *hwpipe = NULL;
	int i;

	for (i = 0; i < mdp5_kms->num_hwpipes; i++) {
		struct mdp5_hw_pipe *cur = mdp5_kms->hwpipes[i];

		/* skip if already in-use: */
		if (cur->plane)
			continue;

		/* skip if doesn't support some required caps: */
		if (caps & ~cur->caps)
			continue;

		/* possible candidate, take the one with the
		 * fewest unneeded caps bits set:
		 */
		if (!hwpipe || (hweight_long(cur->caps & ~caps) <
				hweight_long(hwpipe->caps & ~caps)))
			hwpipe = cur;
	}

	if (hwpipe) {
		DBG("%s: assign to plane %s for caps %x",
				hwpipe->name, plane->name, caps);
		hwpipe->plane = plane;
	}

	return hwpipe;
}

void mdp5_pipe_release(struct mdp5_hw_pipe *hwpipe)
{
	if (!hwpipe)
		return;

	if (WARN_ON(!hwpipe->plane))
		return;

	DBG("%s: release from plane %s", hwpipe->name, hwpipe->plane->name);

	hwpipe->plane = NULL;
}

void mdp5_pipe_destroy(struct mdp5_hw_pipe *hwpipe)
{
	WARN_ON(hwpipe->plane);
	kfree(hwpipe);
}

struct mdp5_hw_pipe *mdp5_pipe_init(enum mdp5_pipe pipe,
		uint32_t reg_offset, uint32_t caps)
{
	struct mdp5_hw_pipe *hwpipe;

	hwpipe = kzalloc(sizeof(*hwpipe), GFP_KERNEL);
	if (!hwpipe)
		return ERR_PTR(-ENOMEM);

	hwpipe->name = pipe2name(pipe);
	hwpipe->pipe = pipe;
	hwpipe->reg_offset = reg_offset;
	hwpipe->caps = caps;
	hwpipe->flush_mask = mdp_ctl_flush_mask_pipe(pipe);

	spin_lock_init(&hwpipe->pipe_lock);

	return hwpipe;
}
