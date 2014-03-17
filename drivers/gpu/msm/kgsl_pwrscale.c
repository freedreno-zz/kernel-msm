/* Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>

#include "kgsl.h"
#include "kgsl_pwrscale.h"
#include "kgsl_device.h"
#include "kgsl_trace.h"

#define FAST_BUS 1
#define SLOW_BUS -1

static void do_devfreq_suspend(struct work_struct *work);
static void do_devfreq_resume(struct work_struct *work);
static void do_devfreq_notify(struct work_struct *work);


static inline struct kgsl_device *todev(struct kgsl_pwrscale *pwrscale)
{
	return pwrscale->dev;
}

static void power_stats(struct kgsl_pwrscale *pwrscale,
		struct kgsl_power_stats *stats)
{
	struct kgsl_device *device = todev(pwrscale);
	device->ftbl->power_stats(device, stats);
}

/*
 * kgsl_pwrscale_sleep - notify governor that device is going off
 * @device: The device
 *
 * Called shortly after all pending work is completed.
 */
void kgsl_pwrscale_sleep(struct kgsl_pwrscale *pwrscale)
{
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));
	if (!pwrscale->enabled)
		return;
	pwrscale->time = pwrscale->on_time = 0;

	/* to call devfreq_suspend_device() from a kernel thread */
	queue_work(pwrscale->devfreq_wq,
		&pwrscale->devfreq_suspend_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_sleep);

/*
 * kgsl_pwrscale_wake - notify governor that device is going on
 * @device: The device
 *
 * Called when the device is returning to an active state.
 */
void kgsl_pwrscale_wake(struct kgsl_pwrscale *pwrscale)
{
	struct kgsl_power_stats stats;
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));

	if (!pwrscale->enabled)
		return;
	/* clear old stats before waking */
	memset(&pwrscale->accum_stats, 0,
		sizeof(pwrscale->accum_stats));

	/* and any hw activity from waking up*/
	power_stats(pwrscale, &stats);

	pwrscale->time = ktime_to_us(ktime_get());

	pwrscale->next_governor_call = 0;

	/* to call devfreq_resume_device() from a kernel thread */
	queue_work(pwrscale->devfreq_wq,
		&pwrscale->devfreq_resume_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_wake);

/*
 * kgsl_pwrscale_busy - update pwrscale state for new work
 * @device: The device
 *
 * Called when new work is submitted to the device.
 * This function must be called with the device mutex locked.
 */
void kgsl_pwrscale_busy(struct kgsl_pwrscale *pwrscale)
{
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));
	if (!pwrscale->enabled)
		return;
	if (pwrscale->on_time == 0)
		pwrscale->on_time = ktime_to_us(ktime_get());
}
EXPORT_SYMBOL(kgsl_pwrscale_busy);

/*
 * kgsl_pwrscale_update - update device busy statistics
 * @device: The device
 *
 * Read hardware busy counters when the device is likely to be
 * on and accumulate the results between devfreq get_dev_status
 * calls. This is limits the need to turn on clocks to read these
 * values for governors that run independently of hardware
 * activity (for example, by time based polling).
 */
void kgsl_pwrscale_update(struct kgsl_pwrscale *pwrscale)
{
	struct kgsl_power_stats stats;
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));

	if (!pwrscale->enabled)
		return;

	if (pwrscale->next_governor_call == 0)
		pwrscale->next_governor_call = jiffies;

	if (time_before(jiffies, pwrscale->next_governor_call))
		return;

	pwrscale->next_governor_call = jiffies
			+ msecs_to_jiffies(KGSL_GOVERNOR_CALL_INTERVAL);

	if (todev(pwrscale)->state == KGSL_STATE_ACTIVE) {
		power_stats(pwrscale, &stats);
		pwrscale->accum_stats.busy_time += stats.busy_time;
		pwrscale->accum_stats.ram_time += stats.ram_time;
		pwrscale->accum_stats.ram_wait += stats.ram_wait;
	}

	/* to call srcu_notifier_call_chain() from a kernel thread */
	if (todev(pwrscale)->requested_state != KGSL_STATE_SLUMBER)
		queue_work(pwrscale->devfreq_wq,
			&pwrscale->devfreq_notify_ws);
}
EXPORT_SYMBOL(kgsl_pwrscale_update);

/*
 * kgsl_pwrscale_disable - temporarily disable the governor
 * @device: The device
 *
 * Temporarily disable the governor, to prevent interference
 * with profiling tools that expect a fixed clock frequency.
 * This function must be called with the device mutex locked.
 */
void kgsl_pwrscale_disable(struct kgsl_pwrscale *pwrscale)
{
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));

	if (pwrscale->enabled) {
		queue_work(pwrscale->devfreq_wq,
			&pwrscale->devfreq_suspend_ws);
		pwrscale->enabled = false;
		kgsl_pwrctrl_pwrlevel_change(pwrscale->pwrctrl, KGSL_PWRLEVEL_TURBO);
	}
}
EXPORT_SYMBOL(kgsl_pwrscale_disable);

/*
 * kgsl_pwrscale_enable - re-enable the governor
 * @device: The device
 *
 * Reenable the governor after a kgsl_pwrscale_disable() call.
 * This function must be called with the device mutex locked.
 */
void kgsl_pwrscale_enable(struct kgsl_pwrscale *pwrscale)
{
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));

	if (!pwrscale->enabled) {
		pwrscale->enabled = true;
		queue_work(pwrscale->devfreq_wq,
			&pwrscale->devfreq_resume_ws);
	}
}
EXPORT_SYMBOL(kgsl_pwrscale_enable);

/*
 * kgsl_devfreq_target - devfreq_dev_profile.target callback
 * @dev: see devfreq.h
 * @freq: see devfreq.h
 * @flags: see devfreq.h
 *
 * This function expects the device mutex to be unlocked.
 */
int kgsl_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct kgsl_pwrctrl *pwr = get_pwrctrl(dev);
	struct kgsl_pwrscale *pwrscale = get_pwrscale(dev);
	struct kgsl_pwrlevel *pwr_level;
	int level, i, b;
	unsigned long cur_freq;

	if (pwrscale == NULL)
		return -ENODEV;
	if (freq == NULL)
		return -EINVAL;
	if (!pwrscale->enabled)
		return 0;

	if (flags & DEVFREQ_FLAG_WAKEUP_MAXFREQ) {
		/*
		 * The GPU is about to get suspended,
		 * but it needs to be at the max power level when waking up
		*/
		pwr->wakeup_maxpwrlevel = 1;
		return 0;
	}

	mutex_lock(&todev(pwrscale)->mutex);
	cur_freq = kgsl_pwrctrl_active_freq(pwr);
	level = pwr->active_pwrlevel;
	pwr_level = &pwr->pwrlevels[level];

	if (*freq != cur_freq) {
		level = pwr->max_pwrlevel;
		for (i = pwr->min_pwrlevel; i >= pwr->max_pwrlevel; i--)
			if (*freq <= pwr->pwrlevels[i].gpu_freq) {
				level = i;
				break;
			}
	} else if (flags && pwr->bus_control) {
		/*
		 * Signal for faster or slower bus.  If KGSL isn't already
		 * running at the desired speed for the given level, modify
		 * its vote.
		 */
		b = pwr->bus_mod;
		if ((flags & DEVFREQ_FLAG_FAST_HINT) &&
			((pwr_level->bus_freq + pwr->bus_mod)
				< pwr_level->bus_max))
			pwr->bus_mod++;
		else if ((flags & DEVFREQ_FLAG_SLOW_HINT) &&
			((pwr_level->bus_freq + pwr->bus_mod)
				> pwr_level->bus_min))
			pwr->bus_mod--;
		if (pwr->bus_mod != b)
			kgsl_pwrctrl_buslevel_update(pwr, true);
	}

	/*
	 * The power constraints need an entire interval to do their magic, so
	 * skip changing the powerlevel if the time hasn't expired yet  and the
	 * new level is less than the constraint
	 */
	if ((pwr->constraint.type != KGSL_CONSTRAINT_NONE) &&
		(!time_after(jiffies, pwr->constraint.expires)) &&
		(level >= pwr->constraint.hint.pwrlevel.level))
			*freq = cur_freq;
	else {
		/* Change the power level */
		kgsl_pwrctrl_pwrlevel_change(pwr, level);

		/*Invalidate the constraint set */
		pwr->constraint.type = KGSL_CONSTRAINT_NONE;
		pwr->constraint.expires = 0;

		*freq = kgsl_pwrctrl_active_freq(pwr);
	}

	mutex_unlock(&todev(pwrscale)->mutex);
	return 0;
}
EXPORT_SYMBOL(kgsl_devfreq_target);

/*
 * kgsl_devfreq_get_dev_status - devfreq_dev_profile.get_dev_status callback
 * @dev: see devfreq.h
 * @freq: see devfreq.h
 * @flags: see devfreq.h
 *
 * This function expects the device mutex to be unlocked.
 */
int kgsl_devfreq_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	struct kgsl_pwrscale *pwrscale = get_pwrscale(dev);
	s64 tmp;

	if (pwrscale == NULL)
		return -ENODEV;
	if (stat == NULL)
		return -EINVAL;

	mutex_lock(&todev(pwrscale)->mutex);
	/* make sure we don't turn on clocks just to read stats */
	if (todev(pwrscale)->state == KGSL_STATE_ACTIVE) {
		struct kgsl_power_stats extra;
		power_stats(pwrscale, &extra);
		pwrscale->accum_stats.busy_time += extra.busy_time;
		pwrscale->accum_stats.ram_time += extra.ram_time;
		pwrscale->accum_stats.ram_wait += extra.ram_wait;
	}

	tmp = ktime_to_us(ktime_get());
	stat->total_time = tmp - pwrscale->time;
	pwrscale->time = tmp;

	stat->busy_time = pwrscale->accum_stats.busy_time;

	stat->current_frequency = kgsl_pwrctrl_active_freq(pwrscale->pwrctrl);

	if (stat->private_data) {
		struct xstats *b = (struct xstats *)stat->private_data;
		b->ram_time = pwrscale->accum_stats.ram_time;
		b->ram_wait = pwrscale->accum_stats.ram_wait;
		b->mod = pwrscale->pwrctrl->bus_mod;
	}

	trace_kgsl_pwrstats(todev(pwrscale), stat->total_time, &pwrscale->accum_stats);
	memset(&pwrscale->accum_stats, 0, sizeof(pwrscale->accum_stats));

	mutex_unlock(&todev(pwrscale)->mutex);

	return 0;
}
EXPORT_SYMBOL(kgsl_devfreq_get_dev_status);

/*
 * kgsl_devfreq_get_cur_freq - devfreq_dev_profile.get_cur_freq callback
 * @dev: see devfreq.h
 * @freq: see devfreq.h
 * @flags: see devfreq.h
 *
 * This function expects the device mutex to be unlocked.
 */
int kgsl_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct kgsl_pwrscale *pwrscale = get_pwrscale(dev);

	if (pwrscale == NULL)
		return -ENODEV;
	if (freq == NULL)
		return -EINVAL;

	mutex_lock(&todev(pwrscale)->mutex);
	*freq = kgsl_pwrctrl_active_freq(pwrscale->pwrctrl);
	mutex_unlock(&todev(pwrscale)->mutex);

	return 0;
}
EXPORT_SYMBOL(kgsl_devfreq_get_cur_freq);

/*
 * kgsl_devfreq_add_notifier - add a fine grained notifier.
 * @dev: The device
 * @nb: Notifier block that will recieve updates.
 *
 * Add a notifier to recieve ADRENO_DEVFREQ_NOTIFY_* events
 * from the device.
 */
int kgsl_devfreq_add_notifier(struct device *dev,
		struct notifier_block *nb)
{
	struct kgsl_pwrscale *pwrscale = get_pwrscale(dev);

	if (pwrscale == NULL)
		return -ENODEV;

	if (nb == NULL)
		return -EINVAL;

	return srcu_notifier_chain_register(&pwrscale->nh, nb);
}
EXPORT_SYMBOL(kgsl_devfreq_add_notifier);

/*
 * kgsl_devfreq_del_notifier - remove a fine grained notifier.
 * @dev: The device
 * @nb: The notifier block.
 *
 * Remove a notifier registered with kgsl_devfreq_add_notifier().
 */
int kgsl_devfreq_del_notifier(struct device *dev, struct notifier_block *nb)
{
	struct kgsl_pwrscale *pwrscale = get_pwrscale(dev);

	if (pwrscale == NULL)
		return -ENODEV;

	if (nb == NULL)
		return -EINVAL;

	return srcu_notifier_chain_unregister(&pwrscale->nh, nb);
}
EXPORT_SYMBOL(kgsl_devfreq_del_notifier);

/*
 * kgsl_pwrscale_init - Initialize pwrscale.
 * @dev: The device
 * @governor: The initial governor to use.
 *
 * Initialize devfreq and any non-constant profile data.
 */
int kgsl_pwrscale_init(struct kgsl_pwrscale *pwrscale,
		struct kgsl_pwrctrl *pwr,
		struct device *dev, const char *governor)
{
	struct devfreq *devfreq;
	struct devfreq_dev_profile *profile;
	struct devfreq_msm_adreno_tz_data *data;
	int i, out = 0;
	int ret;

	profile = &pwrscale->profile;

	srcu_init_notifier_head(&pwrscale->nh);

	profile->initial_freq =
		pwr->pwrlevels[pwr->default_pwrlevel].gpu_freq;
	/* Let's start with 10 ms and tune in later */
	profile->polling_ms = 10;

	/* do not include the 'off' level or duplicate freq. levels */
	for (i = 0; i < (pwr->num_pwrlevels - 1); i++)
		pwrscale->freq_table[out++] = pwr->pwrlevels[i].gpu_freq;

	profile->max_state = out;
	/* link storage array to the devfreq profile pointer */
	profile->freq_table = pwrscale->freq_table;

	/* if there is only 1 freq, no point in running a governor */
	if (profile->max_state == 1)
		governor = "performance";

	/* initialize any governor specific data here */
	for (i = 0; i < profile->num_governor_data; i++) {
		if (strcmp("msm-adreno-tz",
				profile->governor_data[i].name) == 0) {
			data = (struct devfreq_msm_adreno_tz_data *)
				profile->governor_data[i].data;
			/*
			 * If there is a separate GX power rail, allow
			 * independent modification to its voltage through
			 * the bus bandwidth vote.
			 */
			if (pwr->bus_control) {
				out = 0;
				while (pwr->bus_ib[out]) {
					pwr->bus_ib[out] =
						pwr->bus_ib[out] >> 20;
					out++;
				}
				data->bus.num = out;
				data->bus.ib = &pwr->bus_ib[0];
				data->bus.index = &pwr->bus_index[0];
			} else {
				data->bus.num = 0;
			}
		}
	}

	devfreq = devfreq_add_device(dev, &pwrscale->profile, governor, NULL);
	if (IS_ERR(devfreq))
		return PTR_ERR(devfreq);

	pwrscale->devfreq = devfreq;

	ret = sysfs_create_link(&todev(pwrscale)->dev->kobj,
			&devfreq->dev.kobj, "devfreq");

	pwrscale->devfreq_wq = create_freezable_workqueue("kgsl_devfreq_wq");
	INIT_WORK(&pwrscale->devfreq_suspend_ws, do_devfreq_suspend);
	INIT_WORK(&pwrscale->devfreq_resume_ws, do_devfreq_resume);
	INIT_WORK(&pwrscale->devfreq_notify_ws, do_devfreq_notify);

	pwrscale->next_governor_call = 0;
	pwrscale->pwrctrl = pwr;
	pwrscale->dev = dev_get_drvdata(dev);

	return 0;
}
EXPORT_SYMBOL(kgsl_pwrscale_init);

/*
 * kgsl_pwrscale_close - clean up pwrscale
 * @device: the device
 *
 * This function should be called with the device mutex locked.
 */
void kgsl_pwrscale_close(struct kgsl_pwrscale *pwrscale)
{
	BUG_ON(!mutex_is_locked(&todev(pwrscale)->mutex));

	flush_workqueue(pwrscale->devfreq_wq);
	destroy_workqueue(pwrscale->devfreq_wq);
	devfreq_remove_device(pwrscale->devfreq);
	pwrscale->devfreq = NULL;
	srcu_cleanup_notifier_head(&pwrscale->nh);
}
EXPORT_SYMBOL(kgsl_pwrscale_close);

static void do_devfreq_suspend(struct work_struct *work)
{
	struct kgsl_pwrscale *pwrscale = container_of(work,
			struct kgsl_pwrscale, devfreq_suspend_ws);
	struct devfreq *devfreq = pwrscale->devfreq;

	devfreq_suspend_device(devfreq);
}

static void do_devfreq_resume(struct work_struct *work)
{
	struct kgsl_pwrscale *pwrscale = container_of(work,
			struct kgsl_pwrscale, devfreq_resume_ws);
	struct devfreq *devfreq = pwrscale->devfreq;

	devfreq_resume_device(devfreq);
}

static void do_devfreq_notify(struct work_struct *work)
{
	struct kgsl_pwrscale *pwrscale = container_of(work,
			struct kgsl_pwrscale, devfreq_notify_ws);
	struct devfreq *devfreq = pwrscale->devfreq;
	srcu_notifier_call_chain(&pwrscale->nh,
				 ADRENO_DEVFREQ_NOTIFY_RETIRE,
				 devfreq);
}
