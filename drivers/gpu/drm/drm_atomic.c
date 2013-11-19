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


#include <drm/drmP.h>
#include <drm/drm_atomic.h>

/**
 * drm_atomic_begin - start a sequence of atomic updates
 * @dev: DRM device
 * @flags: the modifier flags that userspace has requested
 *
 * Begin a sequence of atomic property sets.  Returns a driver
 * private state object that is passed back into the various
 * object's set_property() fxns, and into the remainder of the
 * atomic funcs.  The state object should accumulate the changes
 * from one o more set_property()'s.  At the end, the state can
 * be checked, and optionally committed.
 *
 * RETURNS
 *   a driver private state object, which is passed back in to
 *   the various other atomic fxns, or error (such as -EBUSY if
 *   there is still a pending async update)
 */
void *drm_atomic_begin(struct drm_device *dev, uint32_t flags)
{
	struct drm_atomic_state *state;
	int sz;
	void *ptr;

	sz = sizeof(*state);

	ptr = kzalloc(sz, GFP_KERNEL);

	state = ptr;
	ptr = &state[1];

	ww_acquire_init(&state->ww_ctx, &crtc_ww_class);
	INIT_LIST_HEAD(&state->locked);

	mutex_init(&state->mutex);
	kref_init(&state->refcount);
	state->dev = dev;
	state->flags = flags;

	return state;
}
EXPORT_SYMBOL(drm_atomic_begin);

/**
 * drm_atomic_set_event - set a pending event on mode object
 * @dev: DRM device
 * @state: the driver private state object
 * @obj: the object to set the event on
 * @event: the event to send back
 *
 * Set pending event for an update on the specified object.  The
 * event is to be sent back to userspace after the update completes.
 */
int drm_atomic_set_event(struct drm_device *dev,
		void *state, struct drm_mode_object *obj,
		struct drm_pending_vblank_event *event)
{
	return -EINVAL;  /* for now */
}
EXPORT_SYMBOL(drm_atomic_set_event);

/**
 * drm_atomic_check - validate state object
 * @dev: DRM device
 * @state: the driver private state object
 *
 * Check the state object to see if the requested state is
 * physically possible.
 *
 * RETURNS
 * Zero for success or -errno
 */
int drm_atomic_check(struct drm_device *dev, void *state)
{
	a->checked = true;
	return 0;  /* for now */
}
EXPORT_SYMBOL(drm_atomic_check);

/* Note that we drop and re-acquire the locks w/ ww_mutex directly,
 * since we keep the crtc in our list with in_atomic == true.
 */

static void drop_locks(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	struct drm_modeset_lock *lock;

	mutex_lock(&a->mutex);
	list_for_each_entry(lock, &a->locked, head)
		ww_mutex_unlock(&lock->mutex);
	mutex_unlock(&a->mutex);

	ww_acquire_fini(ww_ctx);
}

static void grab_locks(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	struct drm_modeset_lock *lock, *slow_locked, *contended;
	int ret;

	lock = slow_locked = contended = NULL;


	ww_acquire_init(ww_ctx, &crtc_ww_class);

	/*
	 * We need to do proper rain^Hww dance.. another context
	 * could sneak in a grab the lock in order to check
	 * crtc->in_atomic, and we get -EDEADLK.  But the winner
	 * will realize the mistake when it sees crtc->in_atomic
	 * already set, and then drop lock and return -EBUSY.
	 * So we just need to keep dancing until we win.
	 */
retry:
	ret = 0;
	list_for_each_entry(lock, &a->locked, head) {
		if (lock == slow_locked) {
			slow_locked = NULL;
			continue;
		}
		contended = lock;
		ret = ww_mutex_lock(&lock->mutex, ww_ctx);
		if (ret)
			goto fail;
	}

fail:
	if (ret == -EDEADLK) {
		/* we lost out in a seqno race, backoff, lock and retry.. */

		list_for_each_entry(lock, &a->locked, head) {
			if (lock == contended)
				break;
			ww_mutex_unlock(&lock->mutex);
		}

		if (slow_locked)
			ww_mutex_unlock(&slow_locked->mutex);

		ww_mutex_lock_slow(&contended->mutex, ww_ctx);
		slow_locked = contended;
		goto retry;
	}
	WARN_ON(ret);   /* if we get EALREADY then something is fubar */
}

static void commit_locks(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	/* and properly release them (clear in_atomic, remove from list): */
	mutex_lock(&a->mutex);
	while (!list_empty(&a->locked)) {
		struct drm_modeset_lock *lock;

		lock = list_first_entry(&a->locked,
				struct drm_modeset_lock, head);

		drm_modeset_unlock(lock);
	}
	mutex_unlock(&a->mutex);
	ww_acquire_fini(ww_ctx);
	a->committed = true;
}

static int atomic_commit(struct drm_atomic_state *a,
		struct ww_acquire_ctx *ww_ctx)
{
	int ret = 0;

	commit_locks(a, ww_ctx);

	return ret;
}

/**
 * drm_atomic_commit - commit state
 * @dev: DRM device
 * @state: the driver private state object
 *
 * Commit the state.  This will only be called if atomic_check()
 * succeeds.
 *
 * RETURNS
 * Zero for success or -errno
 */
int drm_atomic_commit(struct drm_device *dev, void *state)
{
	struct drm_atomic_state *a = state;
	return atomic_commit(a, &a->ww_ctx);
}
EXPORT_SYMBOL(drm_atomic_commit);

/**
 * drm_atomic_commit_unlocked - like drm_atomic_commit
 * but can be called back by driver in other thread.  Manages the lock
 * transfer from initiating thread.
 */
int drm_atomic_commit_unlocked(struct drm_device *dev, void *state)
{
	struct drm_atomic_state *a = state;
	struct ww_acquire_ctx ww_ctx;
	grab_locks(a, &ww_ctx);
	return atomic_commit(a, &ww_ctx);
}
EXPORT_SYMBOL(drm_atomic_commit_unlocked);

/**
 * drm_atomic_end - conclude the atomic update
 * @dev: DRM device
 * @state: the driver private state object
 *
 * Release resources associated with the state object.
 */
void drm_atomic_end(struct drm_device *dev, void *state)
{
	struct drm_atomic_state *a = state;

	/* if commit is happening from another thread, it will
	 * block grabbing locks until we drop (and not set
	 * a->committed until after), so this is not a race:
	 */
	if (!a->committed)
		drop_locks(a, &a->ww_ctx);

	drm_atomic_state_unreference(state);
}
EXPORT_SYMBOL(drm_atomic_end);

void _drm_atomic_state_free(struct kref *kref)
{
	struct drm_atomic_state *a =
		container_of(kref, struct drm_atomic_state, refcount);

	/* in case we haven't already: */
	if (!a->committed) {
		grab_locks(a, &a->ww_ctx);
		commit_locks(a, &a->ww_ctx);
	}

	mutex_destroy(&a->mutex);
	kfree(a);
}
EXPORT_SYMBOL(_drm_atomic_state_free);


const struct drm_atomic_funcs drm_atomic_funcs = {
};
EXPORT_SYMBOL(drm_atomic_funcs);
