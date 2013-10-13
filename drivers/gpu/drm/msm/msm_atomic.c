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
#include "msm_gem.h"

struct msm_async_commit {
	struct drm_atomic_helper_state *state;
	uint32_t fence;
	struct msm_fence_cb fence_cb;
};

static void fence_cb(struct msm_fence_cb *cb);
static int commit_sync(struct drm_device *dev, void *state);

static struct msm_async_commit *new_commit(struct drm_atomic_helper_state *state)
{
	struct msm_async_commit *c = kzalloc(sizeof(*c), GFP_KERNEL);

	if (!c)
		return NULL;

	drm_atomic_helper_state_reference(state);
	c->state = state;
	INIT_FENCE_CB(&c->fence_cb, fence_cb);

	return c;
}
static void free_commit(struct msm_async_commit *c)
{
	drm_atomic_helper_state_unreference(c->state);
	kfree(c);
}

static void fence_cb(struct msm_fence_cb *cb)
{
	struct msm_async_commit *c =
			container_of(cb, struct msm_async_commit, fence_cb);
	commit_sync(c->state->dev, c->state);
	free_commit(c);
}

static void add_fb(struct msm_async_commit *c, struct drm_crtc *crtc,
		struct drm_framebuffer *fb)
{
	struct drm_gem_object *obj = msm_framebuffer_bo(fb, 0);
	c->fence = max(c->fence, msm_gem_fence(to_msm_bo(obj), MSM_PREP_READ));
}

static int wait_fb(struct drm_crtc *crtc, struct drm_framebuffer *fb)
{
	// XXX TODO wait..
	return 0;
}

#define pending_fb(state) ((state) && (state)->fb && (state)->new_fb)

static int commit_sync(struct drm_device *dev, void *state)
{
	struct drm_atomic_helper_state *a = state;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_kms *kms = priv->kms;
	int ncrtcs = dev->mode_config.num_crtc;
	uint32_t pending_crtcs = 0;
	int i, ret;

	for (i = 0; i < ncrtcs; i++)
		if (a->crtcs[i])
			pending_crtcs |= (1 << a->crtcs[i]->id);

	mutex_lock(&dev->struct_mutex);
	WARN_ON(priv->pending_crtcs & pending_crtcs);
	priv->pending_crtcs |= pending_crtcs;
	mutex_unlock(&dev->struct_mutex);

	ret = drm_atomic_helper_commit(dev, state);

	mutex_lock(&dev->struct_mutex);
	priv->pending_crtcs &= ~pending_crtcs;
	mutex_unlock(&dev->struct_mutex);

	if (ret)
		return ret;

	for (i = 0; i < ncrtcs; i++)
		if (a->crtcs[i])
			kms->funcs->flush(kms, a->crtcs[i]);

	return 0;
}

int msm_atomic_commit(struct drm_device *dev, void *state)
{
	struct drm_atomic_helper_state *a = state;
	int nplanes = dev->mode_config.num_plane;
	int ncrtcs = dev->mode_config.num_crtc;
	int i;

	if (a->flags & DRM_MODE_ATOMIC_NONBLOCK) {
		/* non-block mode: defer commit until fb's are ready */
		struct msm_async_commit *c = new_commit(state);

		if (!c)
			return -ENOMEM;

		for (i = 0; i < nplanes; i++)
			if (pending_fb(a->pstates[i]))
				add_fb(c, a->pstates[i]->crtc, a->pstates[i]->fb);

		for (i = 0; i < ncrtcs; i++)
			if (pending_fb(a->cstates[i]))
				add_fb(c, a->crtcs[i], a->cstates[i]->fb);

		return msm_queue_fence_cb(dev, &c->fence_cb, c->fence);
	} else {
		/* blocking mode: wait until fb's are ready */
		int ret = 0;

		for (i = 0; i < nplanes && !ret; i++)
			if (pending_fb(a->pstates[i]))
				ret = wait_fb(a->pstates[i]->crtc, a->pstates[i]->fb);

		for (i = 0; i < ncrtcs && !ret; i++)
			if (pending_fb(a->cstates[i]))
				ret = wait_fb(a->crtcs[i], a->cstates[i]->fb);

		if (ret)
			return ret;
	}

	return commit_sync(dev, state);
}
