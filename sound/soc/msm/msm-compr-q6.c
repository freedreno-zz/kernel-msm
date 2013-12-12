/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#include <linux/init.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/time.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/q6asm.h>
#include <sound/pcm_params.h>
#include <asm/dma.h>
#include <linux/dma-mapping.h>
#include <linux/android_pmem.h>
#include <sound/timer.h>
#include <sound/pcm.h>
#include <mach/qdsp6v2/q6core.h>

#include "msm-compr-q6.h"
#include "msm-pcm-routing.h"

#define COMPRE_CAPTURE_NUM_PERIODS	16
/* Allocate the worst case frame size for compressed audio */
#define COMPRE_CAPTURE_HEADER_SIZE	(sizeof(struct snd_compr_audio_info))
#define COMPRE_CAPTURE_MAX_FRAME_SIZE	(6144)
#define COMPRE_CAPTURE_PERIOD_SIZE	((COMPRE_CAPTURE_MAX_FRAME_SIZE + \
					  COMPRE_CAPTURE_HEADER_SIZE) * \
					  MAX_NUM_FRAMES_PER_BUFFER)
#define COMPRE_OUTPUT_METADATA_SIZE	(sizeof(struct output_meta_data_st))
#define PLAYBACK_MAX_NUM_PERIODS	1024
#define PLAYBACK_MIN_NUM_PERIODS	2
#define PLAYBACK_MAX_PERIOD_SIZE	(160 * 1024 * 6)
#define PLAYBACK_MIN_PERIOD_SIZE        320

static struct audio_locks the_locks;

static struct snd_pcm_hardware msm_compr_hardware_capture = {
	.info =		 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats =	      SNDRV_PCM_FMTBIT_S16_LE,
	.rates =		SNDRV_PCM_RATE_8000_48000,
	.rate_min =	     8000,
	.rate_max =	     48000,
	.channels_min =	 1,
	.channels_max =	 8,
	.buffer_bytes_max =
		COMPRE_CAPTURE_PERIOD_SIZE * COMPRE_CAPTURE_NUM_PERIODS ,
	.period_bytes_min =	COMPRE_CAPTURE_PERIOD_SIZE,
	.period_bytes_max = COMPRE_CAPTURE_PERIOD_SIZE,
	.periods_min =	  COMPRE_CAPTURE_NUM_PERIODS,
	.periods_max =	  COMPRE_CAPTURE_NUM_PERIODS,
	.fifo_size =	    0,
};

static struct snd_pcm_hardware msm_compr_hardware_playback = {
	.info =		 (SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_BLOCK_TRANSFER |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME),
	.formats =	      SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	.rates =	SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT,
	.rate_min =	     8000,
	.rate_max =	     192000,
	.channels_min =	 1,
	.channels_max =	 8,
	.buffer_bytes_max = PLAYBACK_MIN_NUM_PERIODS * PLAYBACK_MAX_PERIOD_SIZE,
	.period_bytes_min = PLAYBACK_MIN_PERIOD_SIZE,
	.period_bytes_max = PLAYBACK_MAX_PERIOD_SIZE,
	.periods_min = PLAYBACK_MIN_NUM_PERIODS,
	.periods_max = PLAYBACK_MAX_NUM_PERIODS,
	.fifo_size =	    0,
};

/* Conventional and unconventional sample rate supported */
static unsigned int supported_sample_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000,
	88200, 96000, 176400, 192000
};

static struct snd_pcm_hw_constraint_list constraints_sample_rates = {
	.count = ARRAY_SIZE(supported_sample_rates),
	.list = supported_sample_rates,
	.mask = 0,
};

static void compr_event_handler(uint32_t opcode,
		uint32_t token, uint32_t *payload, void *priv)
{
	struct compr_audio *compr = priv;
	struct msm_audio *prtd = &compr->prtd;
	struct snd_pcm_substream *substream = prtd->substream;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct audio_aio_write_param param;
	struct audio_aio_read_param read_param;
	struct audio_buffer *buf = NULL;
	struct output_meta_data_st output_meta_data;
	uint32_t *ptrmem = (uint32_t *)payload;
	int i = 0;
	int time_stamp_flag = 0;
	int buffer_length = 0;

	pr_debug("%s opcode =%08x\n", __func__, opcode);
	switch (opcode) {
	case ASM_DATA_EVENT_WRITE_DONE: {
		uint32_t *ptrmem = (uint32_t *)&param;
		pr_debug("ASM_DATA_EVENT_WRITE_DONE\n");
		pr_debug("Buffer Consumed = 0x%08x\n", *ptrmem);
		prtd->pcm_irq_pos += prtd->pcm_count;
		if (atomic_read(&prtd->start))
			snd_pcm_period_elapsed(substream);
		else
			if (substream->timer_running)
				snd_timer_interrupt(substream->timer, 1);
		atomic_inc(&prtd->out_count);
		wake_up(&the_locks.write_wait);
		if (!atomic_read(&prtd->start)) {
			atomic_set(&prtd->pending_buffer, 1);
			break;
		} else
			atomic_set(&prtd->pending_buffer, 0);
		if (!prtd->mmap_flag)
			break;
		if (runtime->status->hw_ptr >= runtime->control->appl_ptr) {
			atomic_set(&prtd->pending_buffer, 1);
			runtime->render_flag |= SNDRV_RENDER_STOPPED;
			atomic_set(&prtd->pending_buffer, 1);
			pr_debug("%s:compr driver underrun hw_ptr = %ld appl_ptr = %ld\n",
				__func__, runtime->status->hw_ptr,
				runtime->control->appl_ptr);
			break;
		}
		buf = prtd->audio_client->port[IN].buf;
		pr_debug("%s:writing %d bytes of buffer[%d] to dsp 2\n",
				__func__, prtd->pcm_count, prtd->out_head);
		pr_debug("%s:writing buffer[%d] from 0x%08x\n",
				__func__, prtd->out_head,
				((unsigned int)buf[0].phys
				+ (prtd->out_head * prtd->pcm_count)));

		if (runtime->tstamp_mode == SNDRV_PCM_TSTAMP_ENABLE)
			time_stamp_flag = SET_TIMESTAMP;
		else
			time_stamp_flag = NO_TIMESTAMP;
		memcpy(&output_meta_data, (char *)(buf->data +
			prtd->out_head * prtd->pcm_count),
			COMPRE_OUTPUT_METADATA_SIZE);

		buffer_length = output_meta_data.frame_size;
		pr_debug("meta_data_length: %d, frame_length: %d\n",
			 output_meta_data.meta_data_length,
			 output_meta_data.frame_size);
		pr_debug("timestamp_msw: %d, timestamp_lsw: %d\n",
			 output_meta_data.timestamp_msw,
			 output_meta_data.timestamp_lsw);
		if (buffer_length == 0) {
			pr_debug("Recieved a zero length buffer-break out");
			break;
		}
		param.paddr = (unsigned long)buf[0].phys
				+ (prtd->out_head * prtd->pcm_count)
				+ output_meta_data.meta_data_length;
		param.len = buffer_length;
		param.msw_ts = output_meta_data.timestamp_msw;
		param.lsw_ts = output_meta_data.timestamp_lsw;
		param.flags = time_stamp_flag;
		param.uid =  (unsigned long)buf[0].phys
				+ (prtd->out_head * prtd->pcm_count
				+ output_meta_data.meta_data_length);
		for (i = 0; i < sizeof(struct audio_aio_write_param)/4;
					i++, ++ptrmem)
			pr_debug("cmd[%d]=0x%08x\n", i, *ptrmem);
		if (q6asm_async_write(prtd->audio_client,
					&param) < 0)
			pr_err("%s:q6asm_async_write failed\n",
				__func__);
		else
			prtd->out_head =
				(prtd->out_head + 1) & (runtime->periods - 1);
		break;
	}
	case ASM_DATA_CMDRSP_EOS:
		pr_debug("ASM_DATA_CMDRSP_EOS\n");
		if (atomic_read(&prtd->eos)) {
			pr_debug("ASM_DATA_CMDRSP_EOS wake up\n");
			prtd->cmd_ack = 1;
			wake_up(&the_locks.eos_wait);
			atomic_set(&prtd->eos, 0);
		}
		atomic_set(&prtd->pending_buffer, 1);
		break;
	case ASM_DATA_EVENT_READ_DONE: {
		pr_debug("ASM_DATA_EVENT_READ_DONE\n");
		pr_debug("buf = %p, data = 0x%X, *data = %p,\n"
			 "prtd->pcm_irq_pos = %d\n",
				prtd->audio_client->port[OUT].buf,
			 *(uint32_t *)prtd->audio_client->port[OUT].buf->data,
				prtd->audio_client->port[OUT].buf->data,
				prtd->pcm_irq_pos);

		memcpy(prtd->audio_client->port[OUT].buf->data +
			   prtd->pcm_irq_pos, (ptrmem + 2),
			   COMPRE_CAPTURE_HEADER_SIZE);
		pr_debug("buf = %p, updated data = 0x%X, *data = %p\n",
				prtd->audio_client->port[OUT].buf,
			*(uint32_t *)(prtd->audio_client->port[OUT].buf->data +
				prtd->pcm_irq_pos),
				prtd->audio_client->port[OUT].buf->data);
		if (!atomic_read(&prtd->start))
			break;
		pr_debug("frame size=%d, buffer = 0x%X\n", ptrmem[2],
				ptrmem[1]);
		if (ptrmem[2] > COMPRE_CAPTURE_MAX_FRAME_SIZE) {
			pr_err("Frame length exceeded the max length");
			break;
		}
		buf = prtd->audio_client->port[OUT].buf;
		pr_debug("pcm_irq_pos=%d, buf[0].phys = 0x%X\n",
				prtd->pcm_irq_pos, (uint32_t)buf[0].phys);
		read_param.len = prtd->pcm_count - COMPRE_CAPTURE_HEADER_SIZE;
		read_param.paddr = (unsigned long)(buf[0].phys) +
			prtd->pcm_irq_pos + COMPRE_CAPTURE_HEADER_SIZE;
		prtd->pcm_irq_pos += prtd->pcm_count;

		if (atomic_read(&prtd->start))
			snd_pcm_period_elapsed(substream);

		q6asm_async_read(prtd->audio_client, &read_param);
		break;
	}
	case ASM_DATA_EVENT_READ_COMPRESSED_DONE: {
		pr_debug("ASM_DATA_EVENT_READ_COMPRESSED_DONE\n");
		pr_debug("buf = %p, data = 0x%X, *data = %p,\n"
			 "prtd->pcm_irq_pos = %d\n",
				prtd->audio_client->port[OUT].buf,
			 *(uint32_t *)prtd->audio_client->port[OUT].buf->data,
				prtd->audio_client->port[OUT].buf->data,
				prtd->pcm_irq_pos);

		if (!atomic_read(&prtd->start))
			break;
		buf = prtd->audio_client->port[OUT].buf;
		pr_debug("pcm_irq_pos=%d, buf[0].phys = 0x%X\n",
				prtd->pcm_irq_pos, (uint32_t)buf[0].phys);
		read_param.len = prtd->pcm_count;
		read_param.paddr = (unsigned long)(buf[0].phys) +
			prtd->pcm_irq_pos;
		prtd->pcm_irq_pos += prtd->pcm_count;

		if (atomic_read(&prtd->start))
			snd_pcm_period_elapsed(substream);

		q6asm_async_read_compressed(prtd->audio_client, &read_param);
		break;
	}
	case APR_BASIC_RSP_RESULT: {
		switch (payload[0]) {
		case ASM_SESSION_CMD_RUN:
			if (substream->stream
				!= SNDRV_PCM_STREAM_PLAYBACK) {
				atomic_set(&prtd->start, 1);
				break;
			}
			if (prtd->mmap_flag) {
				if (!atomic_read(&prtd->pending_buffer))
					break;
				pr_debug("%s:writing %d bytes of buffer[%d] to dsp\n",
					__func__, prtd->pcm_count,
					prtd->out_head);
				buf = prtd->audio_client->port[IN].buf;
				pr_debug("%s:writing buffer[%d] from 0x%08x\n",
					__func__, prtd->out_head,
					((unsigned int)buf[0].phys
					+ (prtd->out_head * prtd->pcm_count)));
				if (runtime->tstamp_mode ==
					SNDRV_PCM_TSTAMP_ENABLE)
					time_stamp_flag = SET_TIMESTAMP;
				else
					time_stamp_flag = NO_TIMESTAMP;
				memcpy(&output_meta_data,
					(char *)(buf->data +
					prtd->out_head * prtd->pcm_count),
					COMPRE_OUTPUT_METADATA_SIZE);
				buffer_length =
					output_meta_data.frame_size;
				pr_debug("meta_data_length: %d, frame_length: %d\n",
					 output_meta_data.meta_data_length,
					 output_meta_data.frame_size);
				pr_debug("timestamp_msw: %d, timestamp_lsw: %d\n",
					 output_meta_data.timestamp_msw,
					 output_meta_data.timestamp_lsw);
				param.paddr =
					(unsigned long)buf[prtd->out_head].phys
					+ output_meta_data.meta_data_length;
				param.len = buffer_length;
				param.msw_ts = output_meta_data.timestamp_msw;
				param.lsw_ts = output_meta_data.timestamp_lsw;
				param.flags = time_stamp_flag;
				param.uid =
					(unsigned long)buf[prtd->out_head].phys
					+ output_meta_data.meta_data_length;
				if (q6asm_async_write(prtd->audio_client,
							&param) < 0)
					pr_err("%s:q6asm_async_write failed\n",
						__func__);
				else
					prtd->out_head =
						(prtd->out_head + 1)
						& (runtime->periods - 1);
				atomic_set(&prtd->pending_buffer, 0);
			} else {
				snd_pcm_period_elapsed(substream);
				atomic_set(&prtd->start, 1);
			}
			break;
		case ASM_STREAM_CMD_FLUSH:
			pr_debug("ASM_STREAM_CMD_FLUSH\n");
			prtd->cmd_ack = 1;
			wake_up(&the_locks.eos_wait);
			wake_up(&the_locks.flush_wait);
			atomic_set(&prtd->flush, 0);
			break;
		default:
			break;
		}
		break;
	}
	default:
		pr_debug("Not Supported Event opcode[0x%x]\n", opcode);
		break;
	}
}

static int msm_compr_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	struct asm_aac_cfg aac_cfg;
	struct asm_wma_cfg wma_cfg;
	struct asm_wmapro_cfg wma_pro_cfg;
	struct asm_amrwbplus_cfg amrwb_cfg;
	int ret, i;
	short bit_width = 16;

	if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ||
		((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		 (prtd->mmap_flag))) {
		ret = q6asm_set_io_mode(prtd->audio_client, ASYNC_IO_MODE);
		if (ret < 0) {
			pr_err("%s: Set IO mode failed\n", __func__);
			return ret;
		}
	}
	pr_debug("compressed stream prepare\n");
	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;
	/* rate and channels are sent to audio driver */
	prtd->samp_rate = runtime->rate;
	prtd->channel_mode = runtime->channels;
	prtd->out_head = 0;
        prtd->delay = 0;

	if (prtd->enabled)
		return 0;

	switch (compr->info.codec_param.codec.id) {
	case SND_AUDIOCODEC_MP3:
		ret = q6asm_media_format_block(prtd->audio_client,
				compr->codec);
		if (ret < 0)
			pr_info("%s: CMD Format block failed\n", __func__);
		break;
	case SND_AUDIOCODEC_AAC:
		pr_debug("SND_AUDIOCODEC_AAC\n");
		memset(&aac_cfg, 0x0, sizeof(struct asm_aac_cfg));
		aac_cfg.aot = AAC_ENC_MODE_EAAC_P;
		aac_cfg.format = 0x03;
		aac_cfg.ch_cfg = runtime->channels;
		aac_cfg.sample_rate = runtime->rate;
		ret = q6asm_media_format_block_aac(prtd->audio_client,
					&aac_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		break;
	case SND_AUDIOCODEC_AC3_PASS_THROUGH:
	case SND_AUDIOCODEC_DTS_PASS_THROUGH:
	case SND_AUDIOCODEC_DTS_LBR_PASS_THROUGH:
	case SND_AUDIOCODEC_EAC3_PASS_THROUGH:
		pr_debug("compressd playback, no need to send decoder params");
		pr_debug("decoder id: %d\n",
			compr->info.codec_param.codec.id);
		break;
	case SND_AUDIOCODEC_WMA:
		pr_debug("SND_AUDIOCODEC_WMA\n");
		memset(&wma_cfg, 0x0, sizeof(struct asm_wma_cfg));
		wma_cfg.format_tag = compr->info.codec_param.codec.format;
		wma_cfg.ch_cfg = compr->info.codec_param.codec.ch_in;
		wma_cfg.sample_rate = compr->info.codec_param.codec.sample_rate;
		wma_cfg.avg_bytes_per_sec =
			compr->info.codec_param.codec.bit_rate/8;
		wma_cfg.block_align = compr->info.codec_param.codec.align;
		wma_cfg.valid_bits_per_sample =
		compr->info.codec_param.codec.options.wma.bits_per_sample;
		wma_cfg.ch_mask =
			compr->info.codec_param.codec.options.wma.channelmask;
		wma_cfg.encode_opt =
			compr->info.codec_param.codec.options.wma.encodeopt;
		ret = q6asm_media_format_block_wma(prtd->audio_client,
					&wma_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		break;
	case SND_AUDIOCODEC_WMA_PRO:
		pr_debug("SND_AUDIOCODEC_WMA_PRO\n");
		memset(&wma_pro_cfg, 0x0, sizeof(struct asm_wmapro_cfg));
		wma_pro_cfg.format_tag = compr->info.codec_param.codec.format;
		wma_pro_cfg.ch_cfg = compr->info.codec_param.codec.ch_in;
		wma_pro_cfg.sample_rate =
			compr->info.codec_param.codec.sample_rate;
		wma_pro_cfg.avg_bytes_per_sec =
			compr->info.codec_param.codec.bit_rate/8;
		wma_pro_cfg.block_align = compr->info.codec_param.codec.align;
		wma_pro_cfg.valid_bits_per_sample =
			compr->info.codec_param.codec\
				.options.wma.bits_per_sample;
		wma_pro_cfg.ch_mask =
			compr->info.codec_param.codec.options.wma.channelmask;
		wma_pro_cfg.encode_opt =
			compr->info.codec_param.codec.options.wma.encodeopt;
		wma_pro_cfg.adv_encode_opt =
			compr->info.codec_param.codec.options.wma.encodeopt1;
		wma_pro_cfg.adv_encode_opt2 =
			compr->info.codec_param.codec.options.wma.encodeopt2;
		ret = q6asm_media_format_block_wmapro(prtd->audio_client,
				&wma_pro_cfg);
		if (ret < 0)
			pr_err("%s: CMD Format block failed\n", __func__);
		break;
	case SND_AUDIOCODEC_DTS:
	case SND_AUDIOCODEC_DTS_LBR:
		pr_debug("SND_AUDIOCODEC_DTS\n");
		ret = q6asm_media_format_block(prtd->audio_client,
				compr->codec);
		if (ret < 0) {
			pr_err("%s: CMD Format block failed\n", __func__);
			return ret;
		}
		break;
	case SND_AUDIOCODEC_AMRWB:
		pr_debug("SND_AUDIOCODEC_AMRWB\n");
		ret = q6asm_media_format_block(prtd->audio_client,
					compr->codec);
		if (ret < 0) {
			pr_err("%s: CMD Format block failed\n", __func__);
			return ret;
		}
		break;
	case SND_AUDIOCODEC_AMRWBPLUS:
		pr_debug("SND_AUDIOCODEC_AMRWBPLUS\n");
		memset(&amrwb_cfg, 0x0, sizeof(struct asm_amrwbplus_cfg));
		amrwb_cfg.size_bytes = sizeof(struct asm_amrwbplus_cfg);
		pr_debug("calling q6asm_media_format_block_amrwbplus");
		ret = q6asm_media_format_block_amrwbplus(prtd->audio_client,
						&amrwb_cfg);
		if (ret < 0) {
			pr_err("%s: CMD Format block failed\n", __func__);
			return ret;
		}
		break;
	case SND_AUDIOCODEC_MP2:
		pr_debug("%s: SND_AUDIOCODEC_MP2\n", __func__);
		break;
	case SND_AUDIOCODEC_PCM:
		pr_debug("%s: SND_AUDIOCODEC_PCM\n", __func__);
		pr_debug("prtd->set_channel_map: %d", prtd->set_channel_map);
		if (!prtd->set_channel_map) {
			pr_debug("using default channel map");
			memset(prtd->channel_map, 0,
				PCM_FORMAT_MAX_NUM_CHANNEL);
			if (prtd->channel_mode == 1) {
				prtd->channel_map[0] = PCM_CHANNEL_FC;
			} else if (prtd->channel_mode == 2) {
				prtd->channel_map[0] = PCM_CHANNEL_FL;
				prtd->channel_map[1] = PCM_CHANNEL_FR;
			} else if (prtd->channel_mode == 4) {
				prtd->channel_map[0] = PCM_CHANNEL_FL;
				prtd->channel_map[1] = PCM_CHANNEL_FR;
				prtd->channel_map[2] = PCM_CHANNEL_LB;
				prtd->channel_map[3] = PCM_CHANNEL_RB;
			} else if (prtd->channel_mode == 6) {
				prtd->channel_map[0] = PCM_CHANNEL_FC;
				prtd->channel_map[1] = PCM_CHANNEL_FL;
				prtd->channel_map[2] = PCM_CHANNEL_FR;
				prtd->channel_map[3] = PCM_CHANNEL_LB;
				prtd->channel_map[4] = PCM_CHANNEL_RB;
				prtd->channel_map[5] = PCM_CHANNEL_LFE;
			} else if (prtd->channel_mode == 8) {
				prtd->channel_map[0] = PCM_CHANNEL_FC;
				prtd->channel_map[1] = PCM_CHANNEL_FL;
				prtd->channel_map[2] = PCM_CHANNEL_FR;
				prtd->channel_map[3] = PCM_CHANNEL_LB;
				prtd->channel_map[4] = PCM_CHANNEL_RB;
				prtd->channel_map[5] = PCM_CHANNEL_LFE;
				prtd->channel_map[6] = PCM_CHANNEL_FLC;
				prtd->channel_map[7] = PCM_CHANNEL_FRC;
			} else {
				pr_err("%s: ERROR.unsupported num_ch = %u\n",
					__func__, prtd->channel_mode);
			}
		}
		for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL; i++) {
			pr_debug("prtd->channel_map[%d]: %d", i,
				prtd->channel_map[i]);
		}
		if (runtime->format == SNDRV_PCM_FORMAT_S24_LE)
			bit_width = 24;

		ret = q6asm_media_format_block_multi_ch_pcm_format_support(
				prtd->audio_client, runtime->rate,
				runtime->channels, prtd->channel_map,
				bit_width);

		if (ret < 0) {
			pr_info("%s: CMD Format block failed\n", __func__);
			return ret;
		}
		break;
	default:
		return -EINVAL;
	}
	if (compr->info.codec_param.codec.transcode_dts) {
		msm_pcm_routing_reg_pseudo_stream(
			MSM_FRONTEND_DAI_PSEUDO,
			prtd->enc_audio_client->perf_mode,
			prtd->enc_audio_client->session,
			SNDRV_PCM_STREAM_CAPTURE,
			48000, runtime->channels > 6 ?
			6 : runtime->channels);
		pr_debug("%s: cmd: DTS ENCDEC CFG BLK\n", __func__);
		ret = q6asm_enc_cfg_blk_dts(prtd->enc_audio_client,
				DTS_ENC_SAMPLE_RATE48k,
				runtime->channels > 6 ?
				6 : runtime->channels);
		if (ret < 0)
			pr_err("%s: CMD: DTS ENCDEC CFG BLK failed\n",
				__func__);
	}
	atomic_set(&prtd->out_count, runtime->periods);
	prtd->enabled = 1;
	prtd->cmd_ack = 0;
	prtd->cmd_interrupt = 0;
	return 0;
}

static int msm_compr_capture_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct msm_audio *prtd = &compr->prtd;
	struct audio_buffer *buf = prtd->audio_client->port[OUT].buf;
	struct snd_codec *codec = &compr->info.codec_param.codec;
	struct audio_aio_read_param read_param;
	int ret = 0;
	int i;
	prtd->pcm_size = snd_pcm_lib_buffer_bytes(substream);
	prtd->pcm_count = snd_pcm_lib_period_bytes(substream);
	prtd->pcm_irq_pos = 0;

	/* rate and channels are sent to audio driver */
	prtd->samp_rate = runtime->rate;
	prtd->channel_mode = runtime->channels;

	if (prtd->enabled)
		return ret;
	read_param.len = prtd->pcm_count;

	switch (codec->id) {
	case SND_AUDIOCODEC_AMRWB:
		pr_debug("SND_AUDIOCODEC_AMRWB\n");
		ret = q6asm_enc_cfg_blk_amrwb(prtd->audio_client,
			MAX_NUM_FRAMES_PER_BUFFER,
			codec->options.generic.reserved[0] /*bitrate 0-8*/,
			codec->options.generic.reserved[1] /*dtx mode 0/1*/);
		if (ret < 0)
			pr_err("%s: CMD Format block" \
				"failed: %d\n", __func__, ret);
		break;
	case SND_AUDIOCODEC_PCM:
		pr_debug("SND_AUDIOCODEC_PCM\n");
		ret = q6asm_enc_cfg_blk_multi_ch_pcm(prtd->audio_client,
			 prtd->samp_rate, prtd->channel_mode);
		if (ret < 0)
			pr_info("%s: CMD Format block failed\n", __func__);
		break;
	default:
		pr_debug("No config for codec %d\n", codec->id);
	}
	pr_debug("%s: Samp_rate = %d, Channel = %d, pcm_size = %d,\n"
			 "pcm_count = %d, periods = %d\n",
			 __func__, prtd->samp_rate, prtd->channel_mode,
			 prtd->pcm_size, prtd->pcm_count, runtime->periods);

	for (i = 0; i < runtime->periods; i++) {
		read_param.uid = i;
		switch (codec->id) {
		case SND_AUDIOCODEC_AMRWB:
		case SND_AUDIOCODEC_PCM:
			read_param.len = prtd->pcm_count
					- COMPRE_CAPTURE_HEADER_SIZE;
			read_param.paddr = (unsigned long)(buf[i].phys)
					+ COMPRE_CAPTURE_HEADER_SIZE;
			pr_debug("Push buffer [%d] to DSP, "\
					"paddr: %p, vaddr: %p\n",
					i, (void *) read_param.paddr,
					buf[i].data);
			q6asm_async_read(prtd->audio_client, &read_param);
			break;
		case SND_AUDIOCODEC_PASS_THROUGH:
			read_param.paddr = (unsigned long)(buf[i].phys);
			q6asm_async_read_compressed(prtd->audio_client,
				&read_param);
			break;
		default:
			pr_err("Invalid format");
			ret = -EINVAL;
			break;
		}
	}
	prtd->periods = runtime->periods;

	prtd->enabled = 1;

	if (compr->info.codec_param.codec.id ==
			SND_AUDIOCODEC_PASS_THROUGH)
		msm_pcm_routing_reg_psthr_stream(
					soc_prtd->dai_link->be_id,
					prtd->session_id, substream->stream,
					1);

	return ret;
}

static int msm_compr_restart(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	struct audio_aio_write_param param;
	struct audio_buffer *buf = NULL;
	struct output_meta_data_st output_meta_data;
	int time_stamp_flag = 0;
	int buffer_length = 0;

	pr_err("msm_compr_restart\n");
	if (runtime->render_flag & SNDRV_RENDER_STOPPED) {
		buf = prtd->audio_client->port[IN].buf;
		pr_debug("%s:writing %d bytes of buffer[%d] to dsp 2\n",
				__func__, prtd->pcm_count, prtd->out_head);
		pr_debug("%s:writing buffer[%d] from 0x%08x\n",
				__func__, prtd->out_head,
				((unsigned int)buf[0].phys
				+ (prtd->out_head * prtd->pcm_count)));

		if (runtime->tstamp_mode == SNDRV_PCM_TSTAMP_ENABLE)
			time_stamp_flag = SET_TIMESTAMP;
		else
			time_stamp_flag = NO_TIMESTAMP;
		memcpy(&output_meta_data, (char *)(buf->data +
			prtd->out_head * prtd->pcm_count),
			COMPRE_OUTPUT_METADATA_SIZE);

		buffer_length = output_meta_data.frame_size;
		pr_debug("meta_data_length: %d, frame_length: %d\n",
			 output_meta_data.meta_data_length,
			 output_meta_data.frame_size);
		pr_debug("timestamp_msw: %d, timestamp_lsw: %d\n",
			 output_meta_data.timestamp_msw,
			 output_meta_data.timestamp_lsw);
		 if (buffer_length == 0) {
			pr_debug("Recieved a zero length buffer-break out");
			return -EINVAL;
		}
		param.paddr = (unsigned long)buf[0].phys
				+ (prtd->out_head * prtd->pcm_count)
				+ output_meta_data.meta_data_length;
		param.len = buffer_length;
		param.msw_ts = output_meta_data.timestamp_msw;
		param.lsw_ts = output_meta_data.timestamp_lsw;
		param.flags = time_stamp_flag;
		param.uid =  (unsigned long)buf[0].phys
				+ (prtd->out_head * prtd->pcm_count
				+ output_meta_data.meta_data_length);
		if (q6asm_async_write(prtd->audio_client,
					&param) < 0)
			pr_err("%s:q6asm_async_write failed\n",
				__func__);
		else
			prtd->out_head =
				(prtd->out_head + 1) & (runtime->periods - 1);

		runtime->render_flag &= ~SNDRV_RENDER_STOPPED;
		atomic_set(&prtd->pending_buffer, 0);
		return 0;
	}
	return 0;
}

static int msm_compr_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	uint64_t mask = 0xFFFFFFFF;

	pr_debug("%s\n", __func__);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->pcm_irq_pos = 0;
		/* intentional fall-through */
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("%s: Trigger start with delay %llu out_count %d\n",
				__func__, prtd->delay,
				atomic_read(&prtd->out_count));
		q6asm_run_nowait(prtd->audio_client, CMD_RUN_RELATIVE,
				prtd->delay & (mask << 32),
				prtd->delay & mask);
		if (prtd->enc_audio_client)
			q6asm_run_nowait(prtd->enc_audio_client, 0, 0, 0);
		if ((substream->stream == SNDRV_PCM_STREAM_CAPTURE) ||
			((substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
			 (prtd->mmap_flag)))
			atomic_set(&prtd->start, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		q6asm_cmd_nowait(prtd->audio_client, CMD_PAUSE);
		atomic_set(&prtd->start, 0);
		runtime->render_flag &= ~SNDRV_RENDER_STOPPED;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_debug("SNDRV_PCM_TRIGGER_PAUSE\n");
		q6asm_cmd_nowait(prtd->audio_client, CMD_PAUSE);
		if (prtd->enc_audio_client)
			q6asm_cmd_nowait(prtd->enc_audio_client, CMD_PAUSE);
		atomic_set(&prtd->start, 0);
		runtime->render_flag &= ~SNDRV_RENDER_STOPPED;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void populate_codec_list(struct compr_audio *compr,
		struct snd_pcm_runtime *runtime)
{
	pr_debug("%s\n", __func__);
	compr->info.compr_cap.num_codecs = 15;
	compr->info.compr_cap.min_fragment_size = runtime->hw.period_bytes_min;
	compr->info.compr_cap.max_fragment_size = runtime->hw.period_bytes_max;
	compr->info.compr_cap.min_fragments = runtime->hw.periods_min;
	compr->info.compr_cap.max_fragments = runtime->hw.periods_max;
	compr->info.compr_cap.codecs[0] = SND_AUDIOCODEC_MP3;
	compr->info.compr_cap.codecs[1] = SND_AUDIOCODEC_AAC;
	compr->info.compr_cap.codecs[2] = SND_AUDIOCODEC_AC3_PASS_THROUGH;
	compr->info.compr_cap.codecs[3] = SND_AUDIOCODEC_WMA;
	compr->info.compr_cap.codecs[4] = SND_AUDIOCODEC_WMA_PRO;
	compr->info.compr_cap.codecs[5] = SND_AUDIOCODEC_DTS;
	compr->info.compr_cap.codecs[6] = SND_AUDIOCODEC_DTS_LBR;
	compr->info.compr_cap.codecs[7] = SND_AUDIOCODEC_DTS_PASS_THROUGH;
	compr->info.compr_cap.codecs[8] = SND_AUDIOCODEC_AMRWB;
	compr->info.compr_cap.codecs[9] = SND_AUDIOCODEC_AMRWBPLUS;
	compr->info.compr_cap.codecs[10] = SND_AUDIOCODEC_PASS_THROUGH;
	compr->info.compr_cap.codecs[11] = SND_AUDIOCODEC_PCM;
	compr->info.compr_cap.codecs[12] = SND_AUDIOCODEC_MP2;
	compr->info.compr_cap.codecs[13] = SND_AUDIOCODEC_DTS_LBR_PASS_THROUGH;
	compr->info.compr_cap.codecs[14] = SND_AUDIOCODEC_EAC3_PASS_THROUGH;
	/* Add new codecs here and update num_codecs*/
}

static int compressed_set_volume(struct msm_audio *prtd, int volume)
{
	int rc = 0;
	struct compr_audio *compr = prtd->substream->runtime->private_data;
	if (prtd && compr) {
		switch (compr->info.codec_param.codec.id) {
		case SND_AUDIOCODEC_AC3_PASS_THROUGH:
		case SND_AUDIOCODEC_DTS_PASS_THROUGH:
		case SND_AUDIOCODEC_DTS_LBR_PASS_THROUGH:
		case SND_AUDIOCODEC_PASS_THROUGH:
		case SND_AUDIOCODEC_EAC3_PASS_THROUGH:
			pr_info("%s: called on passthrough handle\n", __func__);
			return rc;
		}
		rc = q6asm_set_volume(prtd->audio_client, volume);
		if (rc < 0) {
			pr_err("%s: Send Volume command failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		prtd->volume = volume;
	}
	return rc;
}

static int msm_compr_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr;
	struct msm_audio *prtd;
	int ret = 0;

	pr_debug("%s\n", __func__);
	compr = kzalloc(sizeof(struct compr_audio), GFP_KERNEL);
	if (compr == NULL) {
		pr_err("Failed to allocate memory for msm_audio\n");
		return -ENOMEM;
	}
	prtd = &compr->prtd;
	prtd->substream = substream;
	runtime->render_flag = SNDRV_DMA_MODE;
	prtd->audio_client = q6asm_audio_client_alloc(
				(app_cb)compr_event_handler, compr);
	if (!prtd->audio_client) {
		pr_info("%s: Could not allocate memory\n", __func__);
		kfree(prtd);
		return -ENOMEM;
	}
	prtd->audio_client->perf_mode = false;
	pr_info("%s: session ID %d\n", __func__, prtd->audio_client->session);

	prtd->session_id = prtd->audio_client->session;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw = msm_compr_hardware_playback;
		prtd->cmd_ack = 1;
	} else {
		runtime->hw = msm_compr_hardware_capture;
	}


	ret = snd_pcm_hw_constraint_list(runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_sample_rates);
	if (ret < 0)
		pr_err("snd_pcm_hw_constraint_list failed\n");
	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_minmax(runtime,
			SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
			PLAYBACK_MIN_NUM_PERIODS * PLAYBACK_MIN_PERIOD_SIZE,
			PLAYBACK_MIN_NUM_PERIODS * PLAYBACK_MAX_PERIOD_SIZE);
	if (ret < 0)
		pr_err("snd_pcm_hw_constraint buffer size failed\n");
	ret = snd_pcm_hw_constraint_minmax(runtime,
			    SNDRV_PCM_HW_PARAM_PERIODS,
			    PLAYBACK_MIN_NUM_PERIODS,
			    PLAYBACK_MAX_NUM_PERIODS);
	if (ret < 0)
		pr_info("snd_pcm_hw_constraint period cnt failed\n");

	prtd->dsp_cnt = 0;
	atomic_set(&prtd->pending_buffer, 1);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		compr->codec = FORMAT_MP3;
	populate_codec_list(compr, runtime);
	runtime->private_data = compr;
	prtd->audio_client->compr_passthr = false;
	atomic_set(&prtd->eos, 0);

	prtd->volume = 0x2000; /* unity gain */
	return 0;
}


static int msm_compr_playback_copy(struct snd_pcm_substream *substream, int a,
	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	int ret = 0;
	int fbytes = 0;
	int xfer = 0;
	char *bufptr = NULL;
	void *data = NULL;
	uint32_t idx = 0;
	uint32_t size = 0;

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct msm_audio *prtd = runtime->private_data;
	struct output_meta_data_st output_meta_data;
	int time_stamp_flag = 0;
	int buffer_length = 0;

	fbytes = frames_to_bytes(runtime, frames);
	pr_debug("%s: prtd->out_count = %d\n",
		__func__, atomic_read(&prtd->out_count));
	ret = wait_event_timeout(the_locks.write_wait,
		(atomic_read(&prtd->out_count)
		|| atomic_read(&prtd->flush) ||
		atomic_read(&prtd->close)), 5 * HZ);
	if (ret < 0) {
		pr_err("%s: wait_event_timeout failed\n", __func__);
		goto fail;
	}

	if (!atomic_read(&prtd->out_count)) {
		pr_err("%s: pcm stopped out_count 0\n", __func__);
		return 0;
	}

	if (atomic_read(&prtd->flush)) {
		pr_err("%s: write returned due to flush\n", __func__);
		return -EBUSY;
	}

	if (atomic_read(&prtd->close)) {
		pr_err("%s: write returned due to close\n", __func__);
		return -EINVAL;
	}
	data = q6asm_is_cpu_buf_avail(IN, prtd->audio_client, &size, &idx);
	bufptr = data;
	if (bufptr) {
		pr_debug("%s:fbytes =%d: xfer=%d size=%d\n",
			__func__, fbytes, xfer, size);
		xfer = fbytes;
		if (copy_from_user(bufptr, buf, xfer)) {
			ret = -EFAULT;
			goto fail;
		}
		buf += xfer;
		fbytes -= xfer;
		pr_debug("%s:fbytes = %d: xfer=%d\n", __func__, fbytes, xfer);
		pr_debug("%s:writing %d bytes of buffer to dsp\n",
				__func__, xfer);
		if (runtime->tstamp_mode == SNDRV_PCM_TSTAMP_ENABLE)
			time_stamp_flag = SET_TIMESTAMP;
		else
			time_stamp_flag = NO_TIMESTAMP;
		memcpy(&output_meta_data, (char *)bufptr,
				COMPRE_OUTPUT_METADATA_SIZE);
		buffer_length = output_meta_data.frame_size;
		memmove((char *)bufptr, (char *)bufptr +
				COMPRE_OUTPUT_METADATA_SIZE,
				buffer_length);
		ret = q6asm_write(prtd->audio_client, buffer_length,
				output_meta_data.timestamp_msw,
				output_meta_data.timestamp_lsw,
				time_stamp_flag);
		if (ret < 0) {
			ret = -EFAULT;
			goto fail;
		}
		atomic_dec(&prtd->out_count);
	}
fail:
	return  ret;
}

static int msm_compr_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	int dir = 0;

	pr_debug("%s\n", __func__);

	dir = IN;
	atomic_set(&prtd->pending_buffer, 0);
	atomic_set(&prtd->close, 1);
	wake_up(&the_locks.write_wait);
	wake_up(&the_locks.eos_wait);
	prtd->pcm_irq_pos = 0;
	q6asm_cmd(prtd->audio_client, CMD_CLOSE);
	if (prtd->enc_audio_client)
		q6asm_cmd(prtd->enc_audio_client, CMD_CLOSE);
	q6asm_audio_client_buf_free_contiguous(dir,
				prtd->audio_client);
	msm_pcm_routing_dereg_phy_stream(soc_prtd->dai_link->be_id,
					SNDRV_PCM_STREAM_PLAYBACK);
	if (compr->info.codec_param.codec.transcode_dts
                                        && prtd->enc_audio_client) {
		msm_pcm_routing_dereg_pseudo_stream(MSM_FRONTEND_DAI_PSEUDO,
			prtd->enc_audio_client->session);
	}
	if (prtd->enc_audio_client)
		q6asm_audio_client_free(prtd->enc_audio_client);
	q6asm_audio_client_free(prtd->audio_client);
	kfree(prtd);
	return 0;
}

static int msm_compr_capture_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	int dir = OUT;

	pr_debug("%s\n", __func__);
	atomic_set(&prtd->pending_buffer, 0);
	q6asm_cmd(prtd->audio_client, CMD_CLOSE);
	q6asm_audio_client_buf_free_contiguous(dir,
				prtd->audio_client);
	if (compr->info.codec_param.codec.id ==
			SND_AUDIOCODEC_PASS_THROUGH)
		msm_pcm_routing_reg_psthr_stream(
					soc_prtd->dai_link->be_id,
					prtd->session_id, substream->stream,
					0);
	else
		msm_pcm_routing_dereg_phy_stream(soc_prtd->dai_link->be_id,
					SNDRV_PCM_STREAM_CAPTURE);
	q6asm_audio_client_free(prtd->audio_client);
	kfree(prtd);

	return 0;
}

static int msm_compr_close(struct snd_pcm_substream *substream)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_compr_playback_close(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = msm_compr_capture_close(substream);
	return ret;
}

static int msm_compr_copy(struct snd_pcm_substream *substream, int a,
	snd_pcm_uframes_t hwoff, void __user *buf, snd_pcm_uframes_t frames)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_compr_playback_copy(substream, a, hwoff, buf, frames);
	return ret;
}

static int msm_compr_prepare(struct snd_pcm_substream *substream)
{
	int ret = 0;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ret = msm_compr_playback_prepare(substream);
	else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		ret = msm_compr_capture_prepare(substream);
	return ret;
}

static snd_pcm_uframes_t msm_compr_pointer(struct snd_pcm_substream *substream)
{

	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;

	if (prtd->pcm_irq_pos >= prtd->pcm_size)
		prtd->pcm_irq_pos = 0;

	pr_debug("%s: pcm_irq_pos = %d, pcm_size = %d, sample_bits = %d,\n"
			 "frame_bits = %d\n", __func__, prtd->pcm_irq_pos,
			 prtd->pcm_size, runtime->sample_bits,
			 runtime->frame_bits);
	return bytes_to_frames(runtime, (prtd->pcm_irq_pos));
}

static int msm_compr_mmap(struct snd_pcm_substream *substream,
				struct vm_area_struct *vma)
{
	int result = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;

	pr_debug("%s\n", __func__);
	prtd->mmap_flag = 1;
	runtime->render_flag = SNDRV_NON_DMA_MODE;
	if (runtime->dma_addr && runtime->dma_bytes) {
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		result = remap_pfn_range(vma, vma->vm_start,
				runtime->dma_addr >> PAGE_SHIFT,
				runtime->dma_bytes,
				vma->vm_page_prot);
	} else {
		pr_err("Physical address or size of buf is NULL");
		return -EINVAL;
	}
	return result;
}

static int msm_compr_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
	struct audio_buffer *buf;
	int dir, ret;
	short bit_width = 16;

	struct asm_softpause_params softpause = {
		.enable = SOFT_PAUSE_ENABLE,
		.period = SOFT_PAUSE_PERIOD,
		.step = SOFT_PAUSE_STEP,
		.rampingcurve = SOFT_PAUSE_CURVE_LINEAR,
	};
	struct asm_softvolume_params softvol = {
		.period = SOFT_VOLUME_PERIOD,
		.step = SOFT_VOLUME_STEP,
		.rampingcurve = SOFT_VOLUME_CURVE_LINEAR,
	};

	pr_debug("%s\n", __func__);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dir = IN;
	else
		dir = OUT;

	if (params_format(params) == SNDRV_PCM_FORMAT_S24_LE)
		bit_width = 24;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		switch (compr->info.codec_param.codec.id) {
		case SND_AUDIOCODEC_AC3_PASS_THROUGH:
		case SND_AUDIOCODEC_DTS_PASS_THROUGH:
		case SND_AUDIOCODEC_DTS_LBR_PASS_THROUGH:
		case SND_AUDIOCODEC_EAC3_PASS_THROUGH:
			prtd->audio_client->compr_passthr = true;
			ret = q6asm_open_write_compressed(prtd->audio_client,
					compr->codec);

			if (ret < 0) {
				pr_err("%s: Session out open failed\n",
					__func__);
				return -ENOMEM;
			}
			msm_pcm_routing_reg_phy_compr_stream(
				soc_prtd->dai_link->be_id,
				prtd->audio_client->perf_mode,
				prtd->session_id,
				substream->stream,
				prtd->audio_client->compr_passthr);

			break;
		default:
			ret = q6asm_open_write_v2(prtd->audio_client,
					compr->codec, bit_width);
			if (ret < 0) {
				pr_err("%s: Session out open failed\n",
					__func__);
				return -ENOMEM;
			}
			msm_pcm_routing_reg_phy_stream(
				soc_prtd->dai_link->be_id,
				prtd->audio_client->perf_mode,
				prtd->session_id,
				substream->stream);

			ret = compressed_set_volume(prtd, prtd->volume);
			if (ret < 0)
				pr_err("%s : Set Volume failed : %d",
					__func__, ret);

			ret = q6asm_set_softpause(prtd->audio_client,
					&softpause);
			if (ret < 0)
				pr_err("%s: Send SoftPause Param failed ret=%d\n",
					__func__, ret);
			ret = q6asm_set_softvolume(prtd->audio_client,
					&softvol);
			if (ret < 0)
				pr_err("%s: Send SoftVolume Param failed ret=%d\n",
					__func__, ret);

			if (compr->info.codec_param.codec.transcode_dts) {
				prtd->enc_audio_client =
					q6asm_audio_client_alloc(
					(app_cb)compr_event_handler, compr);
				if (!prtd->enc_audio_client) {
					pr_err("%s: Could not allocate " \
							"memory\n", __func__);
					return -ENOMEM;
				}
				prtd->enc_audio_client->perf_mode = false;
				pr_debug("%s Setting up loopback path\n",
						__func__);
				ret = q6asm_open_transcode_loopback(
					prtd->enc_audio_client,
					params_channels(params));
				if (ret < 0) {
					pr_err("%s: Session transcode " \
						"loopback open failed\n",
						__func__);
					return -ENODEV;
				}
			}

			break;
		}
	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		switch (compr->info.codec_param.codec.id) {
		case SND_AUDIOCODEC_AMRWB:
			pr_debug("q6asm_open_read(FORMAT_AMRWB)\n");
			ret = q6asm_open_read(prtd->audio_client,
				FORMAT_AMRWB);
			if (ret < 0) {
				pr_err("%s: compressed Session out open failed\n",
					__func__);
				return -ENOMEM;
			}
			pr_debug("msm_pcm_routing_reg_phy_stream\n");
			msm_pcm_routing_reg_phy_stream(
					soc_prtd->dai_link->be_id,
					prtd->audio_client->perf_mode,
					prtd->session_id, substream->stream);
			break;
		case SND_AUDIOCODEC_PCM:
			pr_debug("q6asm_open_read(FORMAT_PCM)\n");
			ret = q6asm_open_read(prtd->audio_client,
				FORMAT_MULTI_CHANNEL_LINEAR_PCM);
			if (ret < 0) {
				pr_err("%s: compressed Session open failed\n",
					__func__);
				return -ENOMEM;
			}
			pr_debug("msm_pcm_routing_reg_phy_stream\n");
			msm_pcm_routing_reg_phy_stream(
					soc_prtd->dai_link->be_id,
					prtd->audio_client->perf_mode,
					prtd->session_id, substream->stream);
			break;
		case SND_AUDIOCODEC_PASS_THROUGH:
			pr_debug("q6asm_open_read_compressed(COMPRESSED_META_DATA_MODE)\n");
			ret = q6asm_open_read_compressed(prtd->audio_client,
				MAX_NUM_FRAMES_PER_BUFFER,
				COMPRESSED_META_DATA_MODE);
			break;
		default:
			pr_err("Invalid codec for compressed session open\n");
			return -EFAULT;
		}

		if (ret < 0) {
			pr_err("%s: compressed Session out open failed\n",
				__func__);
			return -ENOMEM;
		}
	}
	/* Modifying kernel hardware params based on userspace config */
	if (params_periods(params) > 0 &&
		(params_periods(params) <= runtime->hw.periods_max)) {
		runtime->hw.periods_max = params_periods(params);
	} else {
		pr_err("Audio Start: period max out of bounds\n");
		return -EINVAL;
	}
	if (params_period_bytes(params) > 0 &&
		(params_period_bytes(params) >= runtime->hw.period_bytes_min)) {
		runtime->hw.period_bytes_min = params_period_bytes(params);
	} else {
		pr_err("Audio Start: period bytes min out of bounds\n");
		return -EINVAL;
	}
	pr_debug("%s, periods_max: %d, period_bytes_min: %d\n", __func__,
		runtime->hw.periods_max, runtime->hw.period_bytes_min);
	runtime->hw.buffer_bytes_max =
			runtime->hw.period_bytes_min * runtime->hw.periods_max;
	ret = q6asm_audio_client_buf_alloc_contiguous(dir,
			prtd->audio_client,
			runtime->hw.period_bytes_min,
			runtime->hw.periods_max);
	if (ret < 0) {
		pr_err("Audio Start: Buffer Allocation failed "
					"rc = %d\n", ret);
		return -ENOMEM;
	}
	buf = prtd->audio_client->port[dir].buf;

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;
	dma_buf->area = buf[0].data;
	dma_buf->addr =  buf[0].phys;
	dma_buf->bytes = runtime->hw.buffer_bytes_max;

	pr_debug("%s: buf[%p]dma_buf->area[%p]dma_buf->addr[%p]\n"
		 "dma_buf->bytes[%d]\n", __func__,
		 (void *)buf, (void *)dma_buf->area,
		 (void *)dma_buf->addr, dma_buf->bytes);
	if (!dma_buf->area)
		return -ENOMEM;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	return 0;
}

static int msm_compr_ioctl(struct snd_pcm_substream *substream,
		unsigned int cmd, void *arg)
{
	int rc = 0;
	struct snd_compr_routing param;
	struct snd_soc_pcm_runtime *soc_prtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct compr_audio *compr = runtime->private_data;
	struct msm_audio *prtd = &compr->prtd;
	uint64_t timestamp;
	uint64_t temp;

	switch (cmd) {
	case SNDRV_COMPRESS_TSTAMP: {
		struct snd_compr_tstamp tstamp;
		pr_debug("SNDRV_COMPRESS_TSTAMP\n");

		memset(&tstamp, 0x0, sizeof(struct snd_compr_tstamp));
		rc = q6asm_get_session_time(prtd->audio_client, &timestamp);
		if (rc < 0) {
			pr_err("%s: fail to get session tstamp\n", __func__);
			return rc;
		}
		temp = (timestamp * 2 * runtime->channels);
		temp = temp * (runtime->rate/1000);
		temp = div_u64(temp, 1000);
		tstamp.sampling_rate = runtime->rate;
		tstamp.timestamp = timestamp;
		pr_debug("%s: bytes_consumed:,"
			"timestamp = %lld,\n", __func__,
			tstamp.timestamp);
		if (copy_to_user((void *) arg, &tstamp,
			sizeof(struct snd_compr_tstamp)))
				return -EFAULT;
		return 0;
	}
	case SNDRV_COMPRESS_SET_START_DELAY: {
		prtd->delay = *((uint64_t *) arg);
		pr_debug("resume delay %llu\n", prtd->delay);
		return 0;
	}

	case SNDRV_COMPRESS_GET_CAPS:
		pr_debug("SNDRV_COMPRESS_GET_CAPS\n");
		if (copy_to_user((void *) arg, &compr->info.compr_cap,
			sizeof(struct snd_compr_caps))) {
			rc = -EFAULT;
			pr_err("%s: ERROR: copy to user\n", __func__);
			return rc;
		}
		return 0;
	case SNDRV_COMPRESS_SET_PARAMS:
		pr_debug("SNDRV_COMPRESS_SET_PARAMS: ");
		if (copy_from_user(&compr->info.codec_param, (void *) arg,
			sizeof(struct snd_compr_params))) {
			rc = -EFAULT;
			pr_err("%s: ERROR: copy from user\n", __func__);
			return rc;
		}
		/*
		* DTS Security needed for the transcode path
		*/
		if (compr->info.codec_param.codec.transcode_dts) {
			char modelId[128];
			struct snd_dec_dts opt_dts =
				compr->info.codec_param.codec.dts;
			unsigned int modelIdLength = opt_dts.modelIdLength;
			if (modelIdLength >= sizeof(modelId)) {
				rc = -EINVAL;
				pr_err("%s: ERROR: modelIdLength is"
						"invalid\n", __func__);
				return rc;
			}
			if (copy_from_user(modelId, (void *)opt_dts.modelId,
				modelIdLength))
				pr_err("%s: ERROR: copy modelId\n", __func__);
			modelId[modelIdLength] = '\0';
			pr_debug("%s: Received modelId =%s,length=%d\n",
				__func__, modelId, modelIdLength);
			core_set_dts_model_id(modelIdLength, modelId);
		}
		switch (compr->info.codec_param.codec.id) {
		case SND_AUDIOCODEC_MP3:
			/* For MP3 we dont need any other parameter */
			pr_debug("SND_AUDIOCODEC_MP3\n");
			compr->codec = FORMAT_MP3;
			break;
		case SND_AUDIOCODEC_AAC:
			pr_debug("SND_AUDIOCODEC_AAC\n");
			compr->codec = FORMAT_MPEG4_AAC;
			break;
		case SND_AUDIOCODEC_AC3_PASS_THROUGH:
			pr_debug("SND_AUDIOCODEC_AC3_PASS_THROUGH\n");
			compr->codec = FORMAT_AC3;
			break;
		case SND_AUDIOCODEC_WMA:
			pr_debug("SND_AUDIOCODEC_WMA\n");
			compr->codec = FORMAT_WMA_V9;
			break;
		case SND_AUDIOCODEC_WMA_PRO:
			pr_debug("SND_AUDIOCODEC_WMA_PRO\n");
			compr->codec = FORMAT_WMA_V10PRO;
			break;
		case SND_AUDIOCODEC_DTS_PASS_THROUGH:
			pr_debug("SND_AUDIOCODEC_DTS_PASS_THROUGH\n");
			compr->codec = FORMAT_DTS;
			break;
		case SND_AUDIOCODEC_DTS_LBR_PASS_THROUGH:
			pr_debug("SND_AUDIOCODEC_DTS_LBR_PASS_THROUGH\n");
			compr->codec = FORMAT_DTS_LBR;
			break;
		case SND_AUDIOCODEC_DTS: {
			char modelId[128];
			struct snd_dec_dts opt_dts =
				compr->info.codec_param.codec.dts;
			unsigned int modelIdLength = opt_dts.modelIdLength;
			pr_debug("SND_AUDIOCODEC_DTS\n");
			if (modelIdLength >= sizeof(modelId)) {
				rc = -EINVAL;
				pr_err("%s: ERROR: modelIdLength is"
						"invalid\n", __func__);
				return rc;
			}
			if (copy_from_user(modelId, (void *)opt_dts.modelId,
				modelIdLength))
				pr_err("%s: ERROR: copy modelId\n", __func__);
			modelId[modelIdLength] = '\0';
			pr_debug("%s: Received modelId =%s,length=%d\n",
				__func__, modelId, modelIdLength);
			core_set_dts_model_id(modelIdLength, modelId);
			compr->codec = FORMAT_DTS;
			}
			break;
		case SND_AUDIOCODEC_DTS_LBR:{
			char modelId[128];
			struct snd_dec_dts opt_dts =
				compr->info.codec_param.codec.dts;
			unsigned int modelIdLength = opt_dts.modelIdLength;
			pr_debug("SND_AUDIOCODEC_DTS_LBR\n");
			if (modelIdLength >= sizeof(modelId)) {
				rc = -EINVAL;
				pr_err("%s: ERROR: modelIdLength is"
						"invalid\n", __func__);
				return rc;
			}
			if (copy_from_user(modelId, (void *)opt_dts.modelId,
					modelIdLength))
				pr_err("%s: ERROR: copy modelId\n", __func__);
			modelId[modelIdLength] = '\0';
			pr_debug("%s: Received modelId =%s,length=%d\n",
				__func__, modelId, modelIdLength);
			core_set_dts_model_id(modelIdLength, modelId);
			compr->codec = FORMAT_DTS_LBR;
			}
			break;
		case SND_AUDIOCODEC_AMRWB:
			pr_debug("msm_compr_ioctl SND_AUDIOCODEC_AMRWB\n");
			compr->codec = FORMAT_AMRWB;
			break;
		case SND_AUDIOCODEC_AMRWBPLUS:
			pr_debug("msm_compr_ioctl SND_AUDIOCODEC_AMRWBPLUS\n");
			compr->codec = FORMAT_AMR_WB_PLUS;
			break;
		case SND_AUDIOCODEC_PASS_THROUGH:
			/* format pass through is used for HDMI IN compressed
			   where the decoder format is indicated by LPASS */
			pr_debug("msm_compr_ioctl SND_AUDIOCODEC_PASSTHROUGH\n");
			compr->codec = FORMAT_PASS_THROUGH;
			break;
		case SND_AUDIOCODEC_PCM:
			pr_debug("msm_compr_ioctl SND_AUDIOCODEC_PCM\n");
			compr->codec = FORMAT_MULTI_CHANNEL_LINEAR_PCM;
			break;
		case SND_AUDIOCODEC_MP2:
			pr_debug("SND_AUDIOCODEC_MP2\n");
			compr->codec = FORMAT_MP2;
			break;
		case SND_AUDIOCODEC_EAC3_PASS_THROUGH:
			pr_debug("SND_AUDIOCODEC_EAC3_PASS_THROUGH\n");
			compr->codec = FORMAT_EAC3;
			break;
		default:
			pr_err("msm_compr_ioctl failed..unknown codec\n");
			return -EFAULT;
		}
		return 0;
	case SNDRV_PCM_IOCTL1_RESET:
		pr_debug("SNDRV_PCM_IOCTL1_RESET\n");
		/* Flush only when session is started during CAPTURE,
		   while PLAYBACK has no such restriction. */
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK ||
			  (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
						atomic_read(&prtd->start))) {
			if (atomic_read(&prtd->eos)) {
				prtd->cmd_interrupt = 1;
				wake_up(&the_locks.eos_wait);
				atomic_set(&prtd->eos, 0);
				atomic_set(&prtd->pending_buffer, 1);
			}
			atomic_set(&prtd->flush, 1);
			wake_up(&the_locks.write_wait);
			/* A unlikely race condition possible with FLUSH
			   DRAIN if ack is set by flush and reset by drain */
			prtd->cmd_ack = 0;
			rc = q6asm_cmd(prtd->audio_client, CMD_FLUSH);
			if (rc < 0) {
				pr_err("%s: flush cmd failed rc=%d\n",
					__func__, rc);
				return rc;
			}
			rc = wait_event_timeout(the_locks.flush_wait,
				prtd->cmd_ack, 5 * HZ);
			if (rc < 0) {
				pr_err("Flush cmd timeout\n");
				atomic_set(&prtd->flush, 0);
			}
			prtd->pcm_irq_pos = 0;
		}
		break;
	case SNDRV_COMPRESS_DRAIN:
		pr_debug("%s: SNDRV_COMPRESS_DRAIN\n", __func__);
		if (prtd->mmap_flag) {
			if (atomic_read(&prtd->pending_buffer)) {
				pr_debug("%s: no pending writes, drain would block\n",
				__func__);
				return -EWOULDBLOCK;
			}

			atomic_set(&prtd->eos, 1);
			atomic_set(&prtd->pending_buffer, 0);
			prtd->cmd_ack = 0;
			q6asm_cmd_nowait(prtd->audio_client, CMD_EOS);
			/* Wait indefinitely for  DRAIN.
			   Flush can also signal this*/
			rc = wait_event_interruptible(the_locks.eos_wait,
				(prtd->cmd_ack || prtd->cmd_interrupt));
			if (rc < 0)
				pr_err("EOS cmd interrupted\n");
			pr_debug("%s: SNDRV_COMPRESS_DRAIN  out of wait\n",
				__func__);

			if (prtd->cmd_interrupt)
				rc = -EINTR;

			prtd->cmd_interrupt = 0;
		} else {
			if (!atomic_read(&prtd->start))
				return 0;
			atomic_set(&prtd->eos, 1);
			prtd->cmd_ack = 0;
			q6asm_cmd_nowait(prtd->audio_client, CMD_EOS);
			rc = wait_event_interruptible(
				the_locks.eos_wait, (prtd->cmd_ack ||
				atomic_read(&prtd->flush) ||
				atomic_read(&prtd->close)));
			if (rc < 0)
				pr_err("EOS cmd interrupted\n");
			pr_debug("%s: SNDRV_COMPRESS_DRAIN  out of wait\n",
				__func__);
			if (prtd->cmd_interrupt)
				rc = -EINTR;
			prtd->cmd_interrupt = 0;
		}
		return rc;
	case SNDRV_COMPRESS_SET_ROUTING:
		pr_debug("SNDRV_COMPRESS_SET_ROUTING: ");
		if (copy_from_user(&param, (void *) arg,
			sizeof(struct snd_compr_routing))) {
			rc = -EFAULT;
			pr_err("%s: ERROR: copy from user\n", __func__);
			return rc;
		}
		if (param.session_type == TRANSCODE_SESSION &&
			prtd->enc_audio_client) {
			if (param.operation == DISCONNECT_STREAM)
				msm_pcm_routing_dereg_pseudo_stream(
					MSM_FRONTEND_DAI_PSEUDO,
					prtd->enc_audio_client->session);
			else if (param.operation == CONNECT_STREAM)
				msm_pcm_routing_reg_pseudo_stream(
					MSM_FRONTEND_DAI_PSEUDO,
					prtd->enc_audio_client->perf_mode,
					prtd->enc_audio_client->session,
					SNDRV_PCM_STREAM_CAPTURE,
					48000, runtime->channels > 6 ?
					6 : runtime->channels);
		}
		if (param.session_type == PASSTHROUGH_SESSION) {
			if (param.operation == DISCONNECT_STREAM)
				msm_pcm_routing_dereg_phy_stream(
					soc_prtd->dai_link->be_id,
					SNDRV_PCM_STREAM_PLAYBACK);
			else if (param.operation == CONNECT_STREAM)
				msm_pcm_routing_reg_phy_compr_stream(
					soc_prtd->dai_link->be_id,
					prtd->audio_client->perf_mode,
					prtd->session_id, substream->stream,
					prtd->audio_client->compr_passthr);
		}
		return 0;

	default:
		break;
	}
	return snd_pcm_lib_ioctl(substream, cmd, arg);
}

static int msm_compr_chmap_ctl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int i;
	char channel_mapping[PCM_FORMAT_MAX_NUM_CHANNEL];
	struct snd_pcm_chmap *chmap = kcontrol->private_data;
	struct snd_pcm_substream *substream = chmap->pcm->streams[0].substream;
	struct msm_audio *prtd = substream->runtime->private_data;

	pr_debug("%s", __func__);
	for (i = 0; i < PCM_FORMAT_MAX_NUM_CHANNEL; i++)
		channel_mapping[i] = (char)(ucontrol->value.integer.value[i]);
	if (prtd) {
		prtd->set_channel_map = true;
		memcpy(prtd->channel_map, channel_mapping,
			PCM_FORMAT_MAX_NUM_CHANNEL);
	}
	return 0;
}

static int msm_compr_volume_ctl_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int rc = 0;
	struct snd_pcm_volume *vol = kcontrol->private_data;
	struct snd_pcm_substream *substream = vol->pcm->streams[0].substream;
	struct msm_audio *prtd = substream->runtime->private_data;
	int volume = ucontrol->value.integer.value[0];

	pr_debug("%s\n", __func__);
	rc = compressed_set_volume(prtd, volume);
	return rc;
}

static struct snd_pcm_ops msm_compr_ops = {
	.open	   = msm_compr_open,
	.copy	   = msm_compr_copy,
	.hw_params	= msm_compr_hw_params,
	.close	  = msm_compr_close,
	.ioctl	  = msm_compr_ioctl,
	.prepare	= msm_compr_prepare,
	.trigger	= msm_compr_trigger,
	.pointer	= msm_compr_pointer,
	.mmap		= msm_compr_mmap,
	.restart	= msm_compr_restart,
};

static int msm_compr_add_controls(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm = rtd->pcm->streams[0].pcm;
	struct snd_pcm_chmap *chmap_info;
	struct snd_pcm_volume *volume_info;
	struct snd_kcontrol *kctl;
	char device_num[3];
	int i, ret = 0;

	pr_debug("%s, Channel map cntrl add\n", __func__);
	ret = snd_pcm_add_chmap_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				NULL, PCM_FORMAT_MAX_NUM_CHANNEL,
				rtd->dai_link->be_id,
				&chmap_info);
	if (ret < 0)
		return ret;
	kctl = chmap_info->kctl;
	for (i = 0; i < kctl->count; i++)
		kctl->vd[i].access |= SNDRV_CTL_ELEM_ACCESS_WRITE;
	snprintf(device_num, sizeof(device_num), "%d", pcm->device);
	strlcat(kctl->id.name, device_num, sizeof(kctl->id.name));
	pr_debug("%s, Overwriting channel map control name to: %s",
			 __func__, kctl->id.name);
	kctl->put = msm_compr_chmap_ctl_put;

	pr_debug("%s, Volume cntrl add\n", __func__);
	ret = snd_pcm_add_volume_ctls(pcm, SNDRV_PCM_STREAM_PLAYBACK,
				NULL, 1,
				rtd->dai_link->be_id,
				&volume_info);
	if (ret < 0)
		return ret;
	kctl = volume_info->kctl;
	snprintf(device_num, sizeof(device_num), "%d", pcm->device);
	strlcat(kctl->id.name, device_num, sizeof(kctl->id.name));
	pr_debug("%s, Overwriting volume control name to: %s\n",
			 __func__, kctl->id.name);
	kctl->put = msm_compr_volume_ctl_put;
	return 0;
}

static int msm_asoc_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	int ret = 0;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	ret = msm_compr_add_controls(rtd);
	if (ret)
		pr_err("%s, kctl add failed\n", __func__);
	return ret;
}

static struct snd_soc_platform_driver msm_soc_platform = {
	.ops		= &msm_compr_ops,
	.pcm_new	= msm_asoc_pcm_new,
};

static __devinit int msm_compr_probe(struct platform_device *pdev)
{
	pr_info("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev,
				   &msm_soc_platform);
}

static int msm_compr_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver msm_compr_driver = {
	.driver = {
		.name = "msm-compr-dsp",
		.owner = THIS_MODULE,
	},
	.probe = msm_compr_probe,
	.remove = __devexit_p(msm_compr_remove),
};

static int __init msm_soc_platform_init(void)
{
	init_waitqueue_head(&the_locks.enable_wait);
	init_waitqueue_head(&the_locks.eos_wait);
	init_waitqueue_head(&the_locks.write_wait);
	init_waitqueue_head(&the_locks.read_wait);
	init_waitqueue_head(&the_locks.flush_wait);

	return platform_driver_register(&msm_compr_driver);
}
module_init(msm_soc_platform_init);

static void __exit msm_soc_platform_exit(void)
{
	platform_driver_unregister(&msm_compr_driver);
}
module_exit(msm_soc_platform_exit);

MODULE_DESCRIPTION("PCM module platform driver");
MODULE_LICENSE("GPL v2");
