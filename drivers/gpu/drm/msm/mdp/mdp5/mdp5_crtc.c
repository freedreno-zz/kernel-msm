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

#include <drm/drm_mode.h>
#include "drm_crtc.h"
#include "drm_crtc_helper.h"
#include "drm_flip_work.h"

struct mdp5_crtc {
	struct drm_crtc base;
	char name[8];
	int id;
	bool enabled;

	/* which mixer/encoder we route output to: */
	int mixer;

	/* if there is a pending flip, these will be non-null: */
	struct drm_pending_vblank_event *event;

#define PENDING_CURSOR 0x1
#define PENDING_FLIP   0x2
	atomic_t pending;

	struct mdp_irq vblank;
	struct mdp_irq err;
};
#define to_mdp5_crtc(x) container_of(x, struct mdp5_crtc, base)

static struct mdp5_kms *get_kms(struct drm_crtc *crtc)
{
	struct msm_drm_private *priv = crtc->dev->dev_private;
	return to_mdp5_kms(to_mdp_kms(priv->kms));
}

static void request_pending(struct drm_crtc *crtc, uint32_t pending)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);

	atomic_or(pending, &mdp5_crtc->pending);
	mdp_irq_register(&get_kms(crtc)->base, &mdp5_crtc->vblank);
}

static void crtc_flush(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct mdp5_kms *mdp5_kms = get_kms(crtc);
	int id = mdp5_crtc->id;
	struct drm_plane *plane;
	uint32_t flush = 0;

	for_each_plane_on_crtc(crtc, plane) {
		enum mdp5_pipe pipe = mdp5_plane_pipe(plane);
		flush |= pipe2flush(pipe);
	}

	flush |= mixer2flush(mdp5_crtc->id);
	flush |= MDP5_CTL_FLUSH_CTL;

	DBG("%s: flush=%08x", mdp5_crtc->name, flush);

	mdp5_write(mdp5_kms, REG_MDP5_CTL_FLUSH(id), flush);
}

/* if file!=NULL, this is preclose potential cancel-flip path */
static void complete_flip(struct drm_crtc *crtc, struct drm_file *file)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_pending_vblank_event *event;
	struct drm_plane *plane;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = mdp5_crtc->event;
	if (event) {
		/* if regular vblank case (!file) or if cancel-flip from
		 * preclose on file that requested flip, then send the
		 * event:
		 */
		if (!file || (event->base.file_priv == file)) {
			mdp5_crtc->event = NULL;
			DBG("%s: send event: %p", mdp5_crtc->name, event);
			drm_send_vblank_event(dev, mdp5_crtc->id, event);
		}
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);

	for_each_plane_on_crtc(crtc, plane)
		mdp5_plane_complete_flip(plane);
}

static void mdp5_crtc_destroy(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);

	drm_crtc_cleanup(crtc);

	kfree(mdp5_crtc);
}

static void mdp5_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct mdp5_kms *mdp5_kms = get_kms(crtc);
	bool enabled = (mode == DRM_MODE_DPMS_ON);

	DBG("%s: mode=%d", mdp5_crtc->name, mode);

	if (enabled != mdp5_crtc->enabled) {
		if (enabled) {
			mdp5_enable(mdp5_kms);
			mdp_irq_register(&mdp5_kms->base, &mdp5_crtc->err);
		} else {
			mdp_irq_unregister(&mdp5_kms->base, &mdp5_crtc->err);
			mdp5_disable(mdp5_kms);
		}
		mdp5_crtc->enabled = enabled;
	}
}

static bool mdp5_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void blend_setup(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct mdp5_kms *mdp5_kms = get_kms(crtc);
	int id = mdp5_crtc->id;

	/*
	 * Hard-coded setup for now until I figure out how the
	 * layer-mixer works
	 */

	/* LM[id]: */
	mdp5_write(mdp5_kms, REG_MDP5_LM_BLEND_COLOR_OUT(id),
			MDP5_LM_BLEND_COLOR_OUT_STAGE0_FG_ALPHA);
	mdp5_write(mdp5_kms, REG_MDP5_LM_BLEND_OP_MODE(id, 0),
			MDP5_LM_BLEND_OP_MODE_FG_ALPHA(FG_CONST) |
			MDP5_LM_BLEND_OP_MODE_BG_ALPHA(FG_PIXEL) |
			MDP5_LM_BLEND_OP_MODE_BG_INV_ALPHA);
	mdp5_write(mdp5_kms, REG_MDP5_LM_BLEND_FG_ALPHA(id, 0), 0xff);
	mdp5_write(mdp5_kms, REG_MDP5_LM_BLEND_BG_ALPHA(id, 0), 0x00);

	/* NOTE: seems that LM[n] and CTL[m], we do not need n==m.. but
	 * we want to be setting CTL[m].LAYER[n].  Not sure what the
	 * point of having CTL[m].LAYER[o] (for o!=n).. maybe that is
	 * used when chaining up mixers for high resolution displays?
	 */

	/* CTL[id]: */
	mdp5_write(mdp5_kms, REG_MDP5_CTL_LAYER_REG(id, 0),
			MDP5_CTL_LAYER_REG_RGB0(STAGE0) |
			MDP5_CTL_LAYER_REG_BORDER_COLOR);
	mdp5_write(mdp5_kms, REG_MDP5_CTL_LAYER_REG(id, 1), 0);
	mdp5_write(mdp5_kms, REG_MDP5_CTL_LAYER_REG(id, 2), 0);
	mdp5_write(mdp5_kms, REG_MDP5_CTL_LAYER_REG(id, 3), 0);
	mdp5_write(mdp5_kms, REG_MDP5_CTL_LAYER_REG(id, 4), 0);
}

static void mdp5_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct mdp5_kms *mdp5_kms = get_kms(crtc);

	struct drm_display_mode *mode;

	if (WARN_ON(!crtc->state))
		return;

	mode = &crtc->state->adjusted_mode;

	DBG("%s: set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
			mdp5_crtc->name, mode->base.id, mode->name,
			mode->vrefresh, mode->clock,
			mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal,
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal,
			mode->type, mode->flags);

	mdp5_write(mdp5_kms, REG_MDP5_LM_OUT_SIZE(mdp5_crtc->id),
			MDP5_LM_OUT_SIZE_WIDTH(mode->hdisplay) |
			MDP5_LM_OUT_SIZE_HEIGHT(mode->vdisplay));
}

static void mdp5_crtc_prepare(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	DBG("%s", mdp5_crtc->name);
	/* make sure we hold a ref to mdp clks while setting up mode: */
	mdp5_enable(get_kms(crtc));
	mdp5_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void mdp5_crtc_commit(struct drm_crtc *crtc)
{
	mdp5_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
	crtc_flush(crtc);
	/* drop the ref to mdp clk's that we got in prepare: */
	mdp5_disable(get_kms(crtc));
}

static void mdp5_crtc_load_lut(struct drm_crtc *crtc)
{
}

static int mdp5_crtc_atomic_check(struct drm_crtc *crtc,
		struct drm_crtc_state *state)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct drm_device *dev = crtc->dev;

	DBG("%s: check", mdp5_crtc->name);

	if (mdp5_crtc->event) {
		dev_err(dev->dev, "already pending flip!\n");
		return -EBUSY;
	}

	// TODO anything else to check?

	return 0;
}

static void mdp5_crtc_atomic_begin(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	DBG("%s: begin", mdp5_crtc->name);
}

static void mdp5_crtc_atomic_flush(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	DBG("%s: flush", mdp5_crtc->name);

	WARN_ON(mdp5_crtc->event);

	spin_lock_irqsave(&dev->event_lock, flags);
	mdp5_crtc->event = crtc->state->event;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	blend_setup(crtc);
	crtc_flush(crtc);
	if (crtc->enabled)
		request_pending(crtc, PENDING_FLIP);
}

static int mdp5_crtc_set_property(struct drm_crtc *crtc,
		struct drm_property *property, uint64_t val)
{
	// XXX
	return -EINVAL;
}

static const struct drm_crtc_funcs mdp5_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = mdp5_crtc_destroy,
	.page_flip = drm_atomic_helper_page_flip,
	.set_property = mdp5_crtc_set_property,
	.reset = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_crtc_helper_funcs mdp5_crtc_helper_funcs = {
	.dpms = mdp5_crtc_dpms,
	.mode_fixup = mdp5_crtc_mode_fixup,
	.mode_set_nofb = mdp5_crtc_mode_set_nofb,
	.mode_set = drm_helper_crtc_mode_set,
	.mode_set_base = drm_helper_crtc_mode_set_base,
	.prepare = mdp5_crtc_prepare,
	.commit = mdp5_crtc_commit,
	.load_lut = mdp5_crtc_load_lut,
	.atomic_check = mdp5_crtc_atomic_check,
	.atomic_begin = mdp5_crtc_atomic_begin,
	.atomic_flush = mdp5_crtc_atomic_flush,
};

static void mdp5_crtc_vblank_irq(struct mdp_irq *irq, uint32_t irqstatus)
{
	struct mdp5_crtc *mdp5_crtc = container_of(irq, struct mdp5_crtc, vblank);
	struct drm_crtc *crtc = &mdp5_crtc->base;
	unsigned pending;

	mdp_irq_unregister(&get_kms(crtc)->base, &mdp5_crtc->vblank);

	pending = atomic_xchg(&mdp5_crtc->pending, 0);

	if (pending & PENDING_FLIP) {
		complete_flip(crtc, NULL);
	}
}

static void mdp5_crtc_err_irq(struct mdp_irq *irq, uint32_t irqstatus)
{
	struct mdp5_crtc *mdp5_crtc = container_of(irq, struct mdp5_crtc, err);
	struct drm_crtc *crtc = &mdp5_crtc->base;
	DBG("%s: error: %08x", mdp5_crtc->name, irqstatus);
	crtc_flush(crtc);
}

uint32_t mdp5_crtc_vblank(struct drm_crtc *crtc)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	return mdp5_crtc->vblank.irqmask;
}

void mdp5_crtc_cancel_pending_flip(struct drm_crtc *crtc, struct drm_file *file)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	DBG("%s: cancel: %p", mdp5_crtc->name, file);
	complete_flip(crtc, file);
}

/* set interface for routing crtc->encoder: */
void mdp5_crtc_set_intf(struct drm_crtc *crtc, int intf,
		enum mdp5_intf intf_id)
{
	struct mdp5_crtc *mdp5_crtc = to_mdp5_crtc(crtc);
	struct mdp5_kms *mdp5_kms = get_kms(crtc);
	static const enum mdp5_intfnum intfnum[] = {
			INTF0, INTF1, INTF2, INTF3,
	};
	uint32_t intf_sel;

	/* now that we know what irq's we want: */
	mdp5_crtc->err.irqmask = intf2err(intf);
	mdp5_crtc->vblank.irqmask = intf2vblank(intf);

	/* when called from modeset_init(), skip the rest until later: */
	if (!mdp5_kms)
		return;

	intf_sel = mdp5_read(mdp5_kms, REG_MDP5_DISP_INTF_SEL);

	switch (intf) {
	case 0:
		intf_sel &= ~MDP5_DISP_INTF_SEL_INTF0__MASK;
		intf_sel |= MDP5_DISP_INTF_SEL_INTF0(intf_id);
		break;
	case 1:
		intf_sel &= ~MDP5_DISP_INTF_SEL_INTF1__MASK;
		intf_sel |= MDP5_DISP_INTF_SEL_INTF1(intf_id);
		break;
	case 2:
		intf_sel &= ~MDP5_DISP_INTF_SEL_INTF2__MASK;
		intf_sel |= MDP5_DISP_INTF_SEL_INTF2(intf_id);
		break;
	case 3:
		intf_sel &= ~MDP5_DISP_INTF_SEL_INTF3__MASK;
		intf_sel |= MDP5_DISP_INTF_SEL_INTF3(intf_id);
		break;
	default:
		BUG();
		break;
	}

	blend_setup(crtc);

	DBG("%s: intf_sel=%08x", mdp5_crtc->name, intf_sel);

	mdp5_write(mdp5_kms, REG_MDP5_DISP_INTF_SEL, intf_sel);
	mdp5_write(mdp5_kms, REG_MDP5_CTL_OP(mdp5_crtc->id),
			MDP5_CTL_OP_MODE(MODE_NONE) |
			MDP5_CTL_OP_INTF_NUM(intfnum[intf]));

	crtc_flush(crtc);
}

/* initialize crtc */
struct drm_crtc *mdp5_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, int id)
{
	struct drm_crtc *crtc = NULL;
	struct mdp5_crtc *mdp5_crtc;
	int ret;

	mdp5_crtc = kzalloc(sizeof(*mdp5_crtc), GFP_KERNEL);
	if (!mdp5_crtc) {
		ret = -ENOMEM;
		goto fail;
	}

	crtc = &mdp5_crtc->base;

	mdp5_crtc->id = id;

	mdp5_crtc->vblank.irq = mdp5_crtc_vblank_irq;
	mdp5_crtc->err.irq = mdp5_crtc_err_irq;

	snprintf(mdp5_crtc->name, sizeof(mdp5_crtc->name), "%s:%d",
			pipe2name(mdp5_plane_pipe(plane)), id);

	drm_crtc_init_with_planes(dev, crtc, plane, NULL, &mdp5_crtc_funcs);
	drm_crtc_helper_add(crtc, &mdp5_crtc_helper_funcs);
	plane->crtc = crtc;

	mdp5_plane_install_properties(plane, &crtc->base);

	return crtc;

fail:
	if (crtc)
		mdp5_crtc_destroy(crtc);

	return ERR_PTR(ret);
}
