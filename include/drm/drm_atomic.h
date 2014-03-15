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

#ifndef DRM_ATOMIC_HELPER_H_
#define DRM_ATOMIC_HELPER_H_

/**
 * DOC: atomic state helpers
 *
 * Base helper atomic state and functions.  Drivers are free to either
 * use these as-is, extend them, or completely replace them, in order
 * to implement the atomic KMS API.
 *
 * A naive driver, with no special constraints or hw support for atomic
 * updates may simply add the following to their driver struct:
 *
 *     .atomic_begin     = drm_atomic_begin,
 *     .atomic_set_event = drm_atomic_set_event,
 *     .atomic_check     = drm_atomic_check,
 *     .atomic_commit    = drm_atomic_commit,
 *     .atomic_end       = drm_atomic_end,
 *     .atomics   = &drm_atomic_funcs,
 *
 * In addition, if you're plane/crtc doesn't already have it's own custom
 * properties, then add to your plane/crtc_funcs:
 *
 *     .set_property     = drm_atomic_{plane,crtc}_set_property,
 *
 * Unlike the crtc helpers, it is intended that the atomic helpers can be
 * used piecemeal by the drivers, either using all or overriding parts as
 * needed.
 *
 * A driver which can have (for example) conflicting modes across multiple
 * crtcs (for example, bandwidth limitations or clock/pll configuration
 * restrictions), can simply wrap drm_atomic_check() with their own
 * driver specific .atomic_check() function.
 *
 * A driver which can support true atomic updates can wrap
 * drm_atomic_commit().
 *
 * A driver with custom properties should override the appropriate get_state(),
 * check_state(), and commit_state() functions in .atomics if it uses
 * the drm-atomic-helpers.  Otherwise it is free to use &drm_atomic_funcs
 * as-is.
 */

/**
 * struct drm_atomic_funcs - helper funcs used by the atomic helpers
 */
struct drm_atomic_funcs {
	int (*check_plane_state)(struct drm_plane *plane, struct drm_plane_state *pstate);
	int (*commit_plane_state)(struct drm_plane *plane, struct drm_plane_state *pstate);

	int (*check_crtc_state)(struct drm_crtc *crtc, struct drm_crtc_state *cstate);
	int (*commit_crtc_state)(struct drm_crtc *crtc, struct drm_crtc_state *cstate);
};

const extern struct drm_atomic_funcs drm_atomic_funcs;

void *drm_atomic_begin(struct drm_device *dev, uint32_t flags);
int drm_atomic_set_event(struct drm_device *dev,
		void *state, struct drm_mode_object *obj,
		struct drm_pending_vblank_event *event);
int drm_atomic_check(struct drm_device *dev, void *state);
int drm_atomic_commit(struct drm_device *dev, void *state);
int drm_atomic_commit_unlocked(struct drm_device *dev, void *state);
void drm_atomic_end(struct drm_device *dev, void *state);

int drm_atomic_plane_set_property(struct drm_plane *plane, void *state,
		struct drm_property *property, uint64_t val, void *blob_data);
struct drm_plane_state *drm_atomic_get_plane_state(struct drm_plane *plane,
		void *state);

static inline int
drm_atomic_check_plane_state(struct drm_plane *plane,
		struct drm_plane_state *pstate)
{
	const struct drm_atomic_funcs *funcs =
			plane->dev->driver->atomic_funcs;
	return funcs->check_plane_state(plane, pstate);
}

static inline int
drm_atomic_commit_plane_state(struct drm_plane *plane,
		struct drm_plane_state *pstate)
{
	const struct drm_atomic_funcs *funcs =
			plane->dev->driver->atomic_funcs;
	return funcs->commit_plane_state(plane, pstate);
}

int drm_atomic_crtc_set_property(struct drm_crtc *crtc, void *state,
		struct drm_property *property, uint64_t val, void *blob_data);
struct drm_crtc_state *drm_atomic_get_crtc_state(struct drm_crtc *crtc,
		void *state);

static inline int
drm_atomic_check_crtc_state(struct drm_crtc *crtc,
		struct drm_crtc_state *cstate)
{
	const struct drm_atomic_funcs *funcs =
			crtc->dev->driver->atomic_funcs;
	return funcs->check_crtc_state(crtc, cstate);
}

static inline int
drm_atomic_commit_crtc_state(struct drm_crtc *crtc,
		struct drm_crtc_state *cstate)
{
	const struct drm_atomic_funcs *funcs =
			crtc->dev->driver->atomic_funcs;
	return funcs->commit_crtc_state(crtc, cstate);
}

/**
 * struct drm_atomic_state - the state object used by atomic helpers
 */
struct drm_atomic_state {
	struct kref refcount;
	struct drm_device *dev;
	uint32_t flags;
	struct drm_plane **planes;
	struct drm_plane_state **pstates;
	struct drm_crtc **crtcs;
	struct drm_crtc_state **cstates;

	bool committed;
	bool checked;       /* just for debugging */

	struct ww_acquire_ctx ww_ctx;
	/* list of 'struct drm_modeset_lock': */
	struct list_head locked;

	/* currently simply for protecting against 'locked' list manipulation
	 * between original thread calling atomic->end() and driver thread
	 * calling back drm_atomic_commit_unlocked().
	 *
	 * Other spots are sufficiently synchronized by virtue of holding
	 * the lock's ww_mutex.  But during the lock/resource hand-over to the
	 * driver thread (drop_locks()/grab_locks()), we cannot rely on this.
	 */
	struct mutex mutex;
};

static inline void
drm_atomic_state_reference(struct drm_atomic_state *state)
{
	kref_get(&state->refcount);
}

static inline void
drm_atomic_state_unreference(struct drm_atomic_state *state)
{
	void _drm_atomic_state_free(struct kref *kref);
	kref_put(&state->refcount, _drm_atomic_state_free);
}

#endif /* DRM_ATOMIC_HELPER_H_ */
