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

#include "mdp5_kms.h"


struct mdp5_plane {
	struct drm_plane base;
	const char *name;

	enum mdp5_pipe pipe;

	uint32_t nformats;
	uint32_t formats[32];

	bool enabled;
};
#define to_mdp5_plane(x) container_of(x, struct mdp5_plane, base)

struct mdp5_plane_state {
	struct drm_plane_state base;
	bool modeset;
	bool pending;
};
#define to_mdp5_plane_state(x) \
	container_of(x, struct mdp5_plane_state, base)

static void mdp5_plane_set_scanout(struct drm_plane *plane,
		struct drm_framebuffer *fb);
static int mdp5_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h);

static struct mdp5_kms *get_kms(struct drm_plane *plane)
{
	struct msm_drm_private *priv = plane->dev->dev_private;
	return to_mdp5_kms(to_mdp_kms(priv->kms));
}

static bool plane_enabled(struct drm_plane_state *state)
{
	return state->fb && state->crtc;
}

static int mdp5_plane_disable(struct drm_plane *plane)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);
	enum mdp5_pipe pipe = mdp5_plane->pipe;
	int i;

	DBG("%s: disable", mdp5_plane->name);

	/* update our SMP request to zero (release all our blks): */
	for (i = 0; i < pipe2nclients(pipe); i++)
		mdp5_smp_request(mdp5_kms, pipe2client(pipe, i), 0);

	return 0;
}

static void mdp5_plane_destroy(struct drm_plane *plane)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);

	drm_plane_helper_disable(plane);
	drm_plane_cleanup(plane);

	kfree(mdp5_plane);
}

/* helper to install properties which are common to planes and crtcs */
void mdp5_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj)
{
	// XXX
}

int mdp5_plane_set_property(struct drm_plane *plane,
		struct drm_property *property, uint64_t val)
{
	// XXX
	return -EINVAL;
}

static void mdp5_plane_reset(struct drm_plane *plane)
{
	struct mdp5_plane_state *mdp5_state;

	if (plane->state && plane->state->fb)
		drm_framebuffer_unreference(plane->state->fb);

	kfree(to_mdp5_plane_state(plane->state));
	mdp5_state = kzalloc(sizeof(*mdp5_state), GFP_KERNEL);
	plane->state = &mdp5_state->base;
}

static struct drm_plane_state *
mdp5_plane_duplicate_state(struct drm_plane *plane)
{
	struct mdp5_plane_state *mdp5_state;

	if (WARN_ON(!plane->state))
		return NULL;

	mdp5_state = kmemdup(to_mdp5_plane_state(plane->state),
			sizeof(*mdp5_state), GFP_KERNEL);

	if (mdp5_state && mdp5_state->base.fb)
		drm_framebuffer_reference(mdp5_state->base.fb);

	return &mdp5_state->base;
}

static void mdp5_plane_destroy_state(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	if (state->fb)
		drm_framebuffer_unreference(state->fb);

	kfree(to_mdp5_plane_state(state));
}

static const struct drm_plane_funcs mdp5_plane_funcs = {
		.update_plane = drm_atomic_helper_update_plane,
		.disable_plane = drm_atomic_helper_disable_plane,
		.destroy = mdp5_plane_destroy,
		.set_property = mdp5_plane_set_property,
		.reset = mdp5_plane_reset,
		.atomic_duplicate_state = mdp5_plane_duplicate_state,
		.atomic_destroy_state = mdp5_plane_destroy_state,
};

static int mdp5_plane_prepare_fb(struct drm_plane *plane,
		struct drm_framebuffer *fb)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);

	DBG("%s: prepare: FB[%u]", mdp5_plane->name, fb->base.id);
	return msm_framebuffer_prepare(fb, mdp5_kms->id);
}

static void mdp5_plane_cleanup_fb(struct drm_plane *plane,
		struct drm_framebuffer *fb)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);

	DBG("%s: cleanup: FB[%u]", mdp5_plane->name, fb->base.id);
	msm_framebuffer_cleanup(fb, mdp5_kms->id);
}

static int mdp5_plane_atomic_check(struct drm_plane *plane,
		struct drm_plane_state *state)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct drm_plane_state *old_state = plane->state;

	DBG("%s: check", mdp5_plane->name);

	if (plane_enabled(state) && plane_enabled(old_state)) {
		/* we cannot change SMP block configuration during scanout: */
		if (state->fb->pixel_format != old_state->fb->pixel_format) {
			DBG("%s: pixel_format change!", mdp5_plane->name);
			return -EBUSY;
		}
		if (state->src_w != old_state->src_w) {
			DBG("%s: src_w change!", mdp5_plane->name);
			return -EBUSY;
		}
		if (to_mdp5_plane_state(old_state)->pending) {
			DBG("%s: still pending!", mdp5_plane->name);
			return -EBUSY;
		}
	} else {
		to_mdp5_plane_state(state)->modeset = true;
	}

	return 0;
}

static void mdp5_plane_atomic_update(struct drm_plane *plane)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct drm_plane_state *state = plane->state;

	DBG("%s: update", mdp5_plane->name);

	if (!plane_enabled(state)) {
		to_mdp5_plane_state(state)->pending = true;
		mdp5_plane_disable(plane);
	} else if (to_mdp5_plane_state(state)->modeset) {
		int ret;
		to_mdp5_plane_state(state)->pending = true;
		ret = mdp5_plane_mode_set(plane,
				state->crtc, state->fb,
				state->crtc_x, state->crtc_y,
				state->crtc_w, state->crtc_h,
				state->src_x,  state->src_y,
				state->src_w, state->src_h);
		/* atomic_check should have ensured that this doesn't fail */
		WARN_ON(ret < 0);
	} else {
		mdp5_plane_set_scanout(plane, state->fb);
	}
}

static const struct drm_plane_helper_funcs mdp5_plane_helper_funcs = {
		.prepare_fb = mdp5_plane_prepare_fb,
		.cleanup_fb = mdp5_plane_cleanup_fb,
		.atomic_check = mdp5_plane_atomic_check,
		.atomic_update = mdp5_plane_atomic_update,
};

static void mdp5_plane_set_scanout(struct drm_plane *plane,
		struct drm_framebuffer *fb)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);
	enum mdp5_pipe pipe = mdp5_plane->pipe;
	uint32_t nplanes = drm_format_num_planes(fb->pixel_format);
	uint32_t iova[4];
	int i;

	for (i = 0; i < nplanes; i++) {
		struct drm_gem_object *bo = msm_framebuffer_bo(fb, i);
		msm_gem_get_iova(bo, mdp5_kms->id, &iova[i]);
	}
	for (; i < 4; i++)
		iova[i] = 0;

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_STRIDE_A(pipe),
			MDP5_PIPE_SRC_STRIDE_A_P0(fb->pitches[0]) |
			MDP5_PIPE_SRC_STRIDE_A_P1(fb->pitches[1]));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_STRIDE_B(pipe),
			MDP5_PIPE_SRC_STRIDE_B_P2(fb->pitches[2]) |
			MDP5_PIPE_SRC_STRIDE_B_P3(fb->pitches[3]));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC0_ADDR(pipe), iova[0]);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC1_ADDR(pipe), iova[1]);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC2_ADDR(pipe), iova[2]);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC3_ADDR(pipe), iova[3]);

	plane->fb = fb;
}

/* NOTE: looks like if horizontal decimation is used (if we supported that)
 * then the width used to calculate SMP block requirements is the post-
 * decimated width.  Ie. SMP buffering sits downstream of decimation (which
 * presumably happens during the dma from scanout buffer).
 */
static int request_smp_blocks(struct drm_plane *plane, uint32_t format,
		uint32_t nplanes, uint32_t width)
{
	struct drm_device *dev = plane->dev;
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);
	enum mdp5_pipe pipe = mdp5_plane->pipe;
	int i, hsub, nlines, nblks, ret;

	hsub = drm_format_horz_chroma_subsampling(format);

	/* different if BWC (compressed framebuffer?) enabled: */
	nlines = 2;

	for (i = 0, nblks = 0; i < nplanes; i++) {
		int n, fetch_stride, cpp;

		cpp = drm_format_plane_cpp(format, i);
		fetch_stride = width * cpp / (i ? hsub : 1);

		n = DIV_ROUND_UP(fetch_stride * nlines, SMP_BLK_SIZE);

		/* for hw rev v1.00 */
		if (mdp5_kms->rev == 0)
			n = roundup_pow_of_two(n);

		DBG("%s[%d]: request %d SMP blocks", mdp5_plane->name, i, n);
		ret = mdp5_smp_request(mdp5_kms, pipe2client(pipe, i), n);
		if (ret) {
			dev_err(dev->dev, "Could not allocate %d SMP blocks: %d\n",
					n, ret);
			return ret;
		}

		nblks += n;
	}

	/* in success case, return total # of blocks allocated: */
	return nblks;
}

static void set_fifo_thresholds(struct drm_plane *plane, int nblks)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);
	enum mdp5_pipe pipe = mdp5_plane->pipe;
	uint32_t val;

	/* 1/4 of SMP pool that is being fetched */
	val = (nblks * SMP_ENTRIES_PER_BLK) / 4;

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_REQPRIO_FIFO_WM_0(pipe), val * 1);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_REQPRIO_FIFO_WM_1(pipe), val * 2);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_REQPRIO_FIFO_WM_2(pipe), val * 3);

}

static int mdp5_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	struct mdp5_kms *mdp5_kms = get_kms(plane);
	enum mdp5_pipe pipe = mdp5_plane->pipe;
	const struct mdp_format *format;
	uint32_t nplanes, config = 0;
	uint32_t phasex_step = 0, phasey_step = 0;
	uint32_t hdecm = 0, vdecm = 0;
	int i, nblks;

	nplanes = drm_format_num_planes(fb->pixel_format);

	/* bad formats should already be rejected: */
	if (WARN_ON(nplanes > pipe2nclients(pipe)))
		return -EINVAL;

	/* src values are in Q16 fixed point, convert to integer: */
	src_x = src_x >> 16;
	src_y = src_y >> 16;
	src_w = src_w >> 16;
	src_h = src_h >> 16;

	DBG("%s: FB[%u] %u,%u,%u,%u -> CRTC[%u] %d,%d,%u,%u", mdp5_plane->name,
			fb->base.id, src_x, src_y, src_w, src_h,
			crtc->base.id, crtc_x, crtc_y, crtc_w, crtc_h);

	/*
	 * Calculate and request required # of smp blocks:
	 */
	nblks = request_smp_blocks(plane, fb->pixel_format, nplanes, src_w);
	if (nblks < 0)
		return nblks;

	/*
	 * Currently we update the hw for allocations/requests immediately,
	 * but once atomic modeset/pageflip is in place, the allocation
	 * would move into atomic->check_plane_state(), while updating the
	 * hw would remain here:
	 */
	for (i = 0; i < pipe2nclients(pipe); i++)
		mdp5_smp_configure(mdp5_kms, pipe2client(pipe, i));

	if (src_w != crtc_w) {
		config |= MDP5_PIPE_SCALE_CONFIG_SCALEX_EN;
		/* TODO calc phasex_step, hdecm */
	}

	if (src_h != crtc_h) {
		config |= MDP5_PIPE_SCALE_CONFIG_SCALEY_EN;
		/* TODO calc phasey_step, vdecm */
	}

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_IMG_SIZE(pipe),
			MDP5_PIPE_SRC_IMG_SIZE_WIDTH(src_w) |
			MDP5_PIPE_SRC_IMG_SIZE_HEIGHT(src_h));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_SIZE(pipe),
			MDP5_PIPE_SRC_SIZE_WIDTH(src_w) |
			MDP5_PIPE_SRC_SIZE_HEIGHT(src_h));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_XY(pipe),
			MDP5_PIPE_SRC_XY_X(src_x) |
			MDP5_PIPE_SRC_XY_Y(src_y));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_OUT_SIZE(pipe),
			MDP5_PIPE_OUT_SIZE_WIDTH(crtc_w) |
			MDP5_PIPE_OUT_SIZE_HEIGHT(crtc_h));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_OUT_XY(pipe),
			MDP5_PIPE_OUT_XY_X(crtc_x) |
			MDP5_PIPE_OUT_XY_Y(crtc_y));

	mdp5_plane_set_scanout(plane, fb);

	format = to_mdp_format(msm_framebuffer_format(fb));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_FORMAT(pipe),
			MDP5_PIPE_SRC_FORMAT_A_BPC(format->bpc_a) |
			MDP5_PIPE_SRC_FORMAT_R_BPC(format->bpc_r) |
			MDP5_PIPE_SRC_FORMAT_G_BPC(format->bpc_g) |
			MDP5_PIPE_SRC_FORMAT_B_BPC(format->bpc_b) |
			COND(format->alpha_enable, MDP5_PIPE_SRC_FORMAT_ALPHA_ENABLE) |
			MDP5_PIPE_SRC_FORMAT_CPP(format->cpp - 1) |
			MDP5_PIPE_SRC_FORMAT_UNPACK_COUNT(format->unpack_count - 1) |
			COND(format->unpack_tight, MDP5_PIPE_SRC_FORMAT_UNPACK_TIGHT) |
			MDP5_PIPE_SRC_FORMAT_NUM_PLANES(nplanes - 1) |
			MDP5_PIPE_SRC_FORMAT_CHROMA_SAMP(CHROMA_RGB));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_UNPACK(pipe),
			MDP5_PIPE_SRC_UNPACK_ELEM0(format->unpack[0]) |
			MDP5_PIPE_SRC_UNPACK_ELEM1(format->unpack[1]) |
			MDP5_PIPE_SRC_UNPACK_ELEM2(format->unpack[2]) |
			MDP5_PIPE_SRC_UNPACK_ELEM3(format->unpack[3]));

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_OP_MODE(pipe),
			MDP5_PIPE_SRC_OP_MODE_BWC(BWC_LOSSLESS));

	/* not using secure mode: */
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SRC_ADDR_SW_STATUS(pipe), 0);

	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SCALE_PHASE_STEP_X(pipe), phasex_step);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SCALE_PHASE_STEP_Y(pipe), phasey_step);
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_DECIMATION(pipe),
			MDP5_PIPE_DECIMATION_VERT(vdecm) |
			MDP5_PIPE_DECIMATION_HORZ(hdecm));
	mdp5_write(mdp5_kms, REG_MDP5_PIPE_SCALE_CONFIG(pipe),
			MDP5_PIPE_SCALE_CONFIG_SCALEX_MIN_FILTER(SCALE_FILTER_NEAREST) |
			MDP5_PIPE_SCALE_CONFIG_SCALEY_MIN_FILTER(SCALE_FILTER_NEAREST) |
			MDP5_PIPE_SCALE_CONFIG_SCALEX_CR_FILTER(SCALE_FILTER_NEAREST) |
			MDP5_PIPE_SCALE_CONFIG_SCALEY_CR_FILTER(SCALE_FILTER_NEAREST) |
			MDP5_PIPE_SCALE_CONFIG_SCALEX_MAX_FILTER(SCALE_FILTER_NEAREST) |
			MDP5_PIPE_SCALE_CONFIG_SCALEY_MAX_FILTER(SCALE_FILTER_NEAREST));

	set_fifo_thresholds(plane, nblks);

	return 0;
}

void mdp5_plane_complete_flip(struct drm_plane *plane)
{
	struct mdp5_kms *mdp5_kms = get_kms(plane);
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	enum mdp5_pipe pipe = mdp5_plane->pipe;
	int i;

	DBG("%s: complete flip", mdp5_plane->name);

	for (i = 0; i < pipe2nclients(pipe); i++)
		mdp5_smp_commit(mdp5_kms, pipe2client(pipe, i));

	to_mdp5_plane_state(plane->state)->pending = false;
}

enum mdp5_pipe mdp5_plane_pipe(struct drm_plane *plane)
{
	struct mdp5_plane *mdp5_plane = to_mdp5_plane(plane);
	return mdp5_plane->pipe;
}

/* initialize plane */
struct drm_plane *mdp5_plane_init(struct drm_device *dev,
		enum mdp5_pipe pipe, bool private_plane)
{
	struct drm_plane *plane = NULL;
	struct mdp5_plane *mdp5_plane;
	int ret;
	enum drm_plane_type type;

	mdp5_plane = kzalloc(sizeof(*mdp5_plane), GFP_KERNEL);
	if (!mdp5_plane) {
		ret = -ENOMEM;
		goto fail;
	}

	plane = &mdp5_plane->base;

	mdp5_plane->pipe = pipe;
	mdp5_plane->name = pipe2name(pipe);

	mdp5_plane->nformats = mdp5_get_formats(pipe, mdp5_plane->formats,
			ARRAY_SIZE(mdp5_plane->formats));

	type = private_plane ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
	ret = drm_universal_plane_init(dev, plane, 0xff, &mdp5_plane_funcs,
				 mdp5_plane->formats, mdp5_plane->nformats,
				 type);
	if (ret)
		goto fail;

	drm_plane_helper_add(plane, &mdp5_plane_helper_funcs);

	mdp5_plane_install_properties(plane, &plane->base);

	return plane;

fail:
	if (plane)
		mdp5_plane_destroy(plane);

	return ERR_PTR(ret);
}
