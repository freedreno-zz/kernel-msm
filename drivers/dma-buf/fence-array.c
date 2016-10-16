/*
 * fence-array: aggregate fences to be waited together
 *
 * Copyright (C) 2016 Collabora Ltd
 * Copyright (C) 2016 Advanced Micro Devices, Inc.
 * Authors:
 *	Gustavo Padovan <gustavo@padovan.org>
 *	Christian König <christian.koenig@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/export.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/fence-array.h>

static struct workqueue_struct *fence_array_wq;

static void fence_array_cb_func(struct fence *f, struct fence_cb *cb);

static const char *fence_array_get_driver_name(struct fence *fence)
{
	return "fence_array";
}

static const char *fence_array_get_timeline_name(struct fence *fence)
{
	return "unbound";
}

static void signal_worker(struct work_struct *w)
{
	struct fence_array *array =
		container_of(w, struct fence_array, signal_worker);
	fence_signal(&array->base);
	fence_put(&array->base);
}

static void fence_array_cb_func(struct fence *f, struct fence_cb *cb)
{
	struct fence_array_cb *array_cb =
		container_of(cb, struct fence_array_cb, cb);
	struct fence_array *array = array_cb->array;

	if (atomic_dec_and_test(&array->num_pending)) {
		queue_work(fence_array_wq, &array->signal_worker);
	} else {
		/* we cannot drop final ref under lock, but we
		 * should not have final ref if we enter the
		 * 'else' case:
		 */
		WARN_ON(array->base.refcount.refcount.counter <= 1);
		fence_put(&array->base);
	}
}

static bool fence_array_enable_signaling(struct fence *fence)
{
	struct fence_array *array = to_fence_array(fence);
	struct fence_array_cb *cb = (void *)(&array[1]);
	unsigned i;

	for (i = 0; i < array->num_fences; ++i) {
		cb[i].array = array;
		/*
		 * As we may report that the fence is signaled before all
		 * callbacks are complete, we need to take an additional
		 * reference count on the array so that we do not free it too
		 * early. The core fence handling will only hold the reference
		 * until we signal the array as complete (but that is now
		 * insufficient).
		 */
		fence_get(&array->base);
		if (fence_add_callback(array->fences[i], &cb[i].cb,
				       fence_array_cb_func)) {
			fence_put(&array->base);
			if (atomic_dec_and_test(&array->num_pending))
				return false;
		}
	}

	return true;
}

static bool fence_array_signaled(struct fence *fence)
{
	struct fence_array *array = to_fence_array(fence);

	return atomic_read(&array->num_pending) <= 0;
}

static void fence_array_release(struct fence *fence)
{
	struct fence_array *array = to_fence_array(fence);
	unsigned i;

	for (i = 0; i < array->num_fences; ++i)
		fence_put(array->fences[i]);

	kfree(array->fences);
	fence_free(fence);
}

const struct fence_ops fence_array_ops = {
	.get_driver_name = fence_array_get_driver_name,
	.get_timeline_name = fence_array_get_timeline_name,
	.enable_signaling = fence_array_enable_signaling,
	.signaled = fence_array_signaled,
	.wait = fence_default_wait,
	.release = fence_array_release,
};
EXPORT_SYMBOL(fence_array_ops);

/**
 * fence_array_create - Create a custom fence array
 * @num_fences:		[in]	number of fences to add in the array
 * @fences:		[in]	array containing the fences
 * @context:		[in]	fence context to use
 * @seqno:		[in]	sequence number to use
 * @signal_on_any:	[in]	signal on any fence in the array
 *
 * Allocate a fence_array object and initialize the base fence with fence_init().
 * In case of error it returns NULL.
 *
 * The caller should allocate the fences array with num_fences size
 * and fill it with the fences it wants to add to the object. Ownership of this
 * array is taken and fence_put() is used on each fence on release.
 *
 * If @signal_on_any is true the fence array signals if any fence in the array
 * signals, otherwise it signals when all fences in the array signal.
 */
struct fence_array *fence_array_create(int num_fences, struct fence **fences,
				       u64 context, unsigned seqno,
				       bool signal_on_any)
{
	struct fence_array *array;
	size_t size = sizeof(*array);

	/* Allocate the callback structures behind the array. */
	size += num_fences * sizeof(struct fence_array_cb);
	array = kzalloc(size, GFP_KERNEL);
	if (!array)
		return NULL;

	spin_lock_init(&array->lock);
	fence_init(&array->base, &fence_array_ops, &array->lock,
		   context, seqno);

	array->num_fences = num_fences;
	atomic_set(&array->num_pending, signal_on_any ? 1 : num_fences);
	array->fences = fences;

	INIT_WORK(&array->signal_worker, signal_worker);

	return array;
}
EXPORT_SYMBOL(fence_array_create);

static int __init fence_array_init(void)
{
	fence_array_wq = alloc_ordered_workqueue("fence-array", 0);
	if (!fence_array_wq)
		return -ENOMEM;
	return 0;
}

static void __exit fence_array_exit(void)
{
	flush_workqueue(fence_array_wq);
	destroy_workqueue(fence_array_wq);
}

module_init(fence_array_init);
module_exit(fence_array_exit);
