/*
 * DIT4192 ASoC codec driver
 *
 * Copyright (C) 2013 Amazon.com inc. or its affiliates. All rights reserved.
 *
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/bitrev.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/pcm.h>
#include <sound/dit4192.h>
#include <sound/asoundef.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#define DEBUG

#define DIT4192_RATES  (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |\
			SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000 |\
			SNDRV_PCM_RATE_96000)

#define DIT4192_FORMATS (SNDRV_PCM_FMTBIT_S24_LE |\
			SNDRV_PCM_FORMAT_S16_LE |\
			SNDRV_PCM_FORMAT_S20_3LE)

struct dit4192_stream {
	struct snd_pcm_substream *substream;
	char hw_status[CHANNEL_STATUS_SIZE];		/* hardware status */
	char def_status[CHANNEL_STATUS_SIZE];		/* default status */
	char pcm_status[CHANNEL_STATUS_SIZE];		/* PCM private status */
	char hw_udata[32];
	struct snd_kcontrol *pcm_ctl;
};

struct dit4192 {
        struct mutex lock;
        struct spi_device *spi;
        unsigned int reset_pin;
        unsigned int buf_int;
        unsigned int ubit_pin;
        unsigned int bls_pin;
	struct dit4192_stream playback;
};

struct dit4192 *gl_chip;

static void dit4192_spi_completion_cb(void *arg)
{
        complete(arg);
}

static int dit4192_spi_write_device(struct dit4192 *dit4192, u8 header0, u8 *data, u32 bytes)
{
	int ret;
	u8 spi_data[DIT4192_MAX_DATA_SIZE];
	struct spi_transfer spi_transfer;
	struct spi_message spi_message;
	DECLARE_COMPLETION_ONSTACK(context);

	memset(&spi_transfer, 0, sizeof(struct spi_transfer));
	memset(&spi_message, 0, sizeof(struct spi_message));

	spi_data[DIT4192_HEADER_0] = header0;
	spi_data[DIT4192_HEADER_1] = 0;

	if(bytes > 0) {
		if( bytes <= (DIT4192_MAX_DATA_SIZE-2) ) {
			memcpy(&spi_data[2], data, bytes);
		} else {
			/* This should never happen. */
			pr_err("%s: SPI transfer error. Bad data size\n", __func__);
			return -1;
		}
	}

	spi_transfer.tx_buf = spi_data;
	spi_transfer.len = bytes+2;

	spi_setup(dit4192->spi);
	spi_message_init(&spi_message);
	spi_message_add_tail(&spi_transfer, &spi_message);
	spi_message.complete = dit4192_spi_completion_cb;
	spi_message.context = &context;

	/* must use spi_async in a context that may sleep */
	ret = spi_async(dit4192->spi, &spi_message);
	if (ret == 0) {
		wait_for_completion(&context);
		/* update ret to contain the number of bytes actually written */
		if (spi_message.status == 0) {
			ret = spi_transfer.len;
		} else 
			pr_err("%s: SPI transfer error, spi_message.status = %d\n", 
				__func__, spi_message.status);
	} else {
		pr_err("%s: Error calling spi_async, ret = %d\n", __func__, ret);
	}

	return 0;
}

static int dit4192_spi_read_device(struct dit4192 *dit4192, u8 reg, int bytes, u8 *buf)
{
	int ret;
	unsigned char header[2];
	struct spi_transfer spi_transfer_w;
	struct spi_transfer spi_transfer_r;
	struct spi_message spi_message;
	DECLARE_COMPLETION_ONSTACK(context);

	memset(&spi_transfer_w, 0, sizeof(struct spi_transfer));
	memset(&spi_transfer_r, 0, sizeof(struct spi_transfer));
	memset(&spi_message, 0, sizeof(struct spi_message));

	spi_setup(dit4192->spi);
	spi_message_init(&spi_message);

	header[DIT4192_HEADER_0] = DIT4192_CMD_R | DIT4192_IO_STEP_1 | reg; //0x80
	header[DIT4192_HEADER_1] = 0;

	spi_transfer_w.tx_buf = header;
	spi_transfer_w.len =  2;
	spi_message_add_tail(&spi_transfer_w, &spi_message);

	spi_transfer_r.rx_buf = buf;
	spi_transfer_r.len =  bytes;
	spi_message_add_tail(&spi_transfer_r, &spi_message);

	spi_message.complete = dit4192_spi_completion_cb;
	spi_message.context = &context;

	/* must use spi_async in a context that may sleep */
	ret = spi_async(dit4192->spi, &spi_message);
	if (ret == 0) {
		wait_for_completion(&context);

		if (spi_message.status == 0) {
			/* spi_message.actual_length should contain the number
			* of bytes actually read and should update ret to be
			* the actual length, but since our driver doesn't
			* support this, assume all count bytes were read.
			*/
			ret = bytes;
		}

		if (ret > 0) {
			ret = -EFAULT;
		}
	} else {
		pr_err("%s: Error calling spi_async, ret = %d\n", __func__, ret);
	}

	return ret;
}

static int dit4192_spi_sendbytes(struct dit4192 *chip, u8 reg_start, char *data, int bytes)
{
	u8 header;
	int i, ret;

	for (i = 0; i < bytes; i++) {
		header = DIT4192_CMD_W | DIT4192_IO_STEP_1 | (reg_start + i);
		ret = dit4192_spi_write_device(chip, header, &data[i], 1);
		if (ret < 0) {
			dev_err(&chip->spi->dev,
				"%s: failed to send the data to"
				" dit4192 chip\n", __func__);
			break;
		}
	}

	return i;
}

static int snd_dit4192_in_status_info(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 255;
	return 0;
}

static int snd_dit4192_in_status_get(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct dit4192 *chip = kcontrol->private_data;
	unsigned char val = 0;
	int err = 0;

	err = dit4192_spi_read_device(chip, kcontrol->private_value, 1, &val);
	if (err < 0)
		return err;
	ucontrol->value.integer.value[0] = val;
	return 0;
}

static int snd_dit4192_qsubcode_info(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;
	uinfo->count = 10;
	return 0;
}

static int snd_dit4192_qsubcode_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct dit4192 *chip = kcontrol->private_data;

	if (!chip) {
		pr_err("%s: invalid device info\n", __func__);
		return -ENODEV;
	}
	printk(KERN_INFO"%s(): not supported. Do nothing.\n" , __func__);

	return 0;
}

static int snd_dit4192_spdif_info(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}


static int snd_dit4192_send_csdata(struct dit4192 *obj,
				    int udata,
				    unsigned char *ndata,
				    int count)
{
	struct dit4192 *chip = obj;
	unsigned char data[CHANNEL_STATUS_SIZE*2];
	int i;

	memset(data, 0, sizeof(data));
	for (i=0; i<count; i++) {
		data[2*i] = ndata[i];
		data[2*i+1] = ndata[i];
	}

	dit4192_spi_sendbytes(chip, DIT4192_CS_START, data, count*2);

	return 0;
}

static int snd_dit4192_spdif_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct dit4192 *chip = kcontrol->private_data;

	if (!chip) {
		pr_err("%s: invalid device info\n", __func__);
		return -ENODEV;
	}

	memcpy(ucontrol->value.iec958.status,
			chip->playback.def_status, CHANNEL_STATUS_SIZE);
	return 0;
}

static int snd_dit4192_spdif_put(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct dit4192 *chip = kcontrol->private_data;
	unsigned char *status;
	int err, change;

	if (!chip) {
		pr_err("%s: invalid device info\n", __func__);
		return -ENODEV;
	}

	status = kcontrol->private_value ?
		chip->playback.pcm_status : chip->playback.def_status;

	change = memcmp(ucontrol->value.iec958.status, status, CHANNEL_STATUS_SIZE) != 0;

	memcpy(status, ucontrol->value.iec958.status, CHANNEL_STATUS_SIZE);
	err = snd_dit4192_send_csdata(chip, 0, status, CHANNEL_STATUS_SIZE);
	if (err < 0) change = err;

	return change;
}

static int snd_dit4192_spdif_mask_info(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_IEC958;
	uinfo->count = 1;
	return 0;
}

static int snd_dit4192_spdif_mask_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	memset(ucontrol->value.iec958.status, 0xff, CHANNEL_STATUS_SIZE);
	return 0;
}

static struct snd_kcontrol_new snd_dit4192_iec958_controls[] = {
	{
		.iface	=	SNDRV_CTL_ELEM_IFACE_PCM,
		.info	=	snd_dit4192_in_status_info,
		.name	=	"IEC958 DIT4192 Input Status",
		.access =	(SNDRV_CTL_ELEM_ACCESS_READ |
				 SNDRV_CTL_ELEM_ACCESS_VOLATILE),
		.get	=	snd_dit4192_in_status_get,
		.private_value = 15,
	},
	{
		.iface	=	SNDRV_CTL_ELEM_IFACE_PCM,
		.info	=	snd_dit4192_in_status_info,
		.name	=	"IEC958 DIT4192 Error Status",
		.access =	(SNDRV_CTL_ELEM_ACCESS_READ |
				 SNDRV_CTL_ELEM_ACCESS_VOLATILE),
		.get	=	snd_dit4192_in_status_get,
		.private_value = 16,
	},
	{
		.access	=	SNDRV_CTL_ELEM_ACCESS_READ,
		.iface	=	SNDRV_CTL_ELEM_IFACE_PCM,
		.name	=	SNDRV_CTL_NAME_IEC958("", PLAYBACK, MASK),
		.info	=	snd_dit4192_spdif_mask_info,
		.get	=	snd_dit4192_spdif_mask_get,
	},
	{
		.iface	=	SNDRV_CTL_ELEM_IFACE_PCM,
		.name	=	SNDRV_CTL_NAME_IEC958("", PLAYBACK,
							DEFAULT),
		.info	=	snd_dit4192_spdif_info,
		.get	=	snd_dit4192_spdif_get,
		.put	=	snd_dit4192_spdif_put,
		.private_value = 0
	},
	{
		.access	=	(SNDRV_CTL_ELEM_ACCESS_READWRITE |
				 SNDRV_CTL_ELEM_ACCESS_INACTIVE),
		.iface	=	SNDRV_CTL_ELEM_IFACE_PCM,
		.name	=	SNDRV_CTL_NAME_IEC958("", PLAYBACK, PCM_STREAM),
		.info	=	snd_dit4192_spdif_info,
		.get	=	snd_dit4192_spdif_get,
		.put	=	snd_dit4192_spdif_put,
		.private_value = 1
	},
	{
		.iface	=	SNDRV_CTL_ELEM_IFACE_PCM,
		.info	=	snd_dit4192_qsubcode_info,
		.name	=	"IEC958 Q-subcode Capture Default",
		.access =	(SNDRV_CTL_ELEM_ACCESS_READ |
				 SNDRV_CTL_ELEM_ACCESS_VOLATILE),
		.get	=	snd_dit4192_qsubcode_get
	}
};

static int dit4192_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct dit4192 *chip = dev_get_drvdata(codec->dev);
	int ret = 0;

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return -ENODEV;
	}
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	default:
		pr_err("invalid format\n");
		break;
	}
	dev_dbg(&chip->spi->dev,
		"%s(): substream = %s  stream = %d, fmt = %d\n" , __func__,
		 substream->name, substream->stream, params_format(params));
	return ret;
}

static int snd_dit4192_iec958_register_kcontrol(struct dit4192 *dit4192,
			    struct snd_card *card)
{
	struct dit4192 *chip = dit4192;
	struct snd_kcontrol *kctl;
	unsigned int idx;
	int err;

	for (idx = 0; idx < ARRAY_SIZE(snd_dit4192_iec958_controls); idx++) {
		kctl = snd_ctl_new1(&snd_dit4192_iec958_controls[idx], chip);
		if (kctl == NULL)
			return -ENOMEM;
		err = snd_ctl_add(card, kctl);
		if (err < 0) {
			dev_err(&chip->spi->dev,
				"failed to add the kcontrol\n");
			return err;
		}
	}
	return err;
}

static int dit4192_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct dit4192 *chip = dev_get_drvdata(dai->codec->dev);

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return -ENODEV;
	}

	return 0;
}

static void dit4192_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct dit4192 *chip = dev_get_drvdata(dai->codec->dev);

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return;
	}
	dev_dbg(&chip->spi->dev,
		"%s(): substream = %s  stream = %d\n" , __func__,
		 substream->name, substream->stream);
}

static int dit4192_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct dit4192 *chip = dev_get_drvdata(dai->codec->dev);

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return -ENODEV;
	}
	dev_dbg(&chip->spi->dev, "%s\n", __func__);
	return 0;
}

static struct snd_soc_dai_ops dit4192_dai_ops = {
	.startup = dit4192_startup,
	.shutdown = dit4192_shutdown,
	.hw_params = dit4192_hw_params,
	.set_fmt = dit4192_set_dai_fmt,
};

static struct snd_soc_dai_driver dit4192_dai[] = {
	{
		.name = "spdif_rx",
		.id = 1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.rates = DIT4192_RATES,
			.formats = DIT4192_FORMATS,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &dit4192_dai_ops,
	},
};


static unsigned int dit4192_soc_spi_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	struct dit4192 *chip = dev_get_drvdata(codec->dev);

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return -ENODEV;
	}
	return 0;
}

static int dit4192_soc_spi_write(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	struct dit4192 *chip = dev_get_drvdata(codec->dev);

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return -ENODEV;
	}
	return 0;
}

static int dit4192_soc_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct dit4192 *chip;
	codec->control_data = dev_get_drvdata(codec->dev);
	chip = codec->control_data;

	if (chip == NULL) {
		pr_err("invalid device private data\n");
		return -ENODEV;
	}
	snd_dit4192_iec958_register_kcontrol(chip, codec->card->snd_card);
	dev_set_drvdata(codec->dev, chip);
	return ret;
}

static struct snd_soc_codec_driver soc_codec_dev_dit4192 = {
	.read = dit4192_soc_spi_read,
	.write = dit4192_soc_spi_write,
	.probe = dit4192_soc_probe,
};

void dit4192_poweron(struct dit4192 *chip)
{
	/*Bring the chip out of reset. 500ns miimum wait.*/
	msleep(20);
	gpio_direction_output(chip->reset_pin, 1);
	printk(KERN_INFO"%s: dit4192_poweron\n", __func__);
}

void dit4192_init(struct dit4192 *chip)
{
	u8 header, spi_data[DIT4192_MAX_DATA_SIZE];

	/* Setup BLS as output. */
	header = DIT4192_CMD_W | DIT4192_IO_STEP_1 | DIT4192_REG_1;
	spi_data[0] = DIT4192_BIT_BLSM_OUT;
	dit4192_spi_write_device(chip, header, spi_data, 1);

	/* Setup OP mode and CLK. */
	header = DIT4192_CMD_W | DIT4192_IO_STEP_1 | DIT4192_REG_2;
	spi_data[0] = DIT4192_BIT_PDN_OP | DIT4192_BIT_CLK_256_FS;
	dit4192_spi_write_device(chip, header, spi_data, 1);

	/* Setup bit depth and other I2S parameters. */
	header = DIT4192_CMD_W | DIT4192_IO_STEP_1 | DIT4192_REG_3;
	spi_data[0] = DIT4192_BIT_LEFT_LOW | DIT4192_BIT_WLEN_16BITS | DIT4192_BIT_DELAY | DIT4192_BIT_MS_SLAVE; 
	dit4192_spi_write_device(chip, header, spi_data, 1);

	/* Enable MBT interrupt */
	header = DIT4192_CMD_W | DIT4192_IO_STEP_1 | DIT4192_REG_5;
	spi_data[0] = DIT4192_BIT_MBTI;
	dit4192_spi_write_device(chip, header, spi_data, 1);
}

int dit4192_gpio_init(struct dit4192 *chip)
{
	int ret;

        ret = gpio_request(chip->buf_int, "dit4192_spi-buf_int,");
        if (ret) {
                pr_err("%s: GPIO request for pin number %u failed\n",
                           __func__, chip->buf_int);
                return ret;
        }
	ret = gpio_direction_input(chip->buf_int);
	if (ret) {
		pr_err("%s: unable to set GPIO(%d) direction, err=%d\n",
			__func__, chip->buf_int, ret);
		gpio_free(chip->buf_int);
		goto err_gpio_dir;
        }

        ret = gpio_request(chip->ubit_pin, "dit4192_spi-ubit_pin");
        if (ret) {
                pr_err("%s: GPIO request for pin number %u failed\n",
                           __func__, chip->ubit_pin);
                return ret;
        }
	ret = gpio_direction_output(chip->ubit_pin, 0);
	if (ret) {
		pr_err("%s: unable to set GPIO(%d) direction, err=%d\n",
			__func__, chip->ubit_pin, ret);
		gpio_free(chip->ubit_pin);
		goto err_gpio_dir;
        }

        ret = gpio_request(chip->bls_pin, "dit4192_spi-bls_pin");
        if (ret) {
                pr_err("%s: GPIO request for pin number %u failed\n",
                           __func__, chip->bls_pin);
                return ret;
        }
	ret = gpio_direction_input(chip->bls_pin);
	if (ret) {
		pr_err("%s: unable to set GPIO(%d) direction, err=%d\n",
			__func__, chip->bls_pin, ret);
		gpio_free(chip->bls_pin);
		goto err_gpio_dir;
        }

        ret = gpio_request(chip->reset_pin, "dit4192_spi-reset_pin");
        if (ret) {
                pr_err("%s: GPIO request for pin number %u failed\n",
                           __func__, chip->reset_pin);
                return ret;
        }
	ret = gpio_direction_output(chip->reset_pin, 0);
	if (ret) {
		pr_err("%s: unable to set GPIO(%d) direction, err=%d\n",
			__func__, chip->reset_pin, ret);
		gpio_free(chip->reset_pin);
        }

	return ret;

err_gpio_dir:
	return ret;
}

static void dit4192_gpio_free(struct dit4192 *chip)
{
	gpio_free(chip->reset_pin);
	gpio_free(chip->buf_int);
	gpio_free(chip->ubit_pin);
	gpio_free(chip->bls_pin);
}

static void dit4192_reg_dump(struct dit4192 *dit4192, u8 reg_start, int size)
{
	int i;
	u8 data[DIT4192_CS_START+CHANNEL_STATUS_SIZE];

	memset(data, 0xA5, sizeof(data));
	dit4192_spi_read_device(dit4192, reg_start, size, data);
	for(i=0; i<size; i++) {
		printk(KERN_INFO"dit4192_reg_dump-%i: 0x%X\n", i, data[i]);
	}
}

static int dit4192_default_regs[DIT4192_CS_START] = {0, 0, 3, 0, 0, 0, 0, 0};
static int dit4192_check_device(struct dit4192 *chip)
{
	int i;
	u8 data[DIT4192_CS_START];

	memset(data, 0xA5, sizeof(data));
	dit4192_spi_read_device(chip, 0, DIT4192_CS_START, data);

	for( i=0; i<DIT4192_CS_START; i++) {
		if( data[i] != dit4192_default_regs[i]) {
			printk(KERN_ERR"dit4192_reg: bad reg-%d value-0x%X\n", i, data[i]);
			dit4192_reg_dump(chip, 0, DIT4192_CS_START);
			return -EINVAL;
		}
	}
	printk(KERN_INFO"%s: found device.\n", __func__);

	return 0;
}

static int __devinit dit4192_spi_probe(struct spi_device *spi)
{
	int ret;
	struct dit4192 *chip;
	struct dit4192_platform_data *pdata;

	if (spi->dev.platform_data == NULL) {
		pr_err("%s: platform data is missing\n", __func__);
		return -EINVAL;
	}
	pdata = (struct dit4192_platform_data *)spi->dev.platform_data;
        gl_chip = chip = devm_kzalloc(&spi->dev, sizeof(struct dit4192), GFP_KERNEL);
        if (!chip)
                return -ENOMEM;

	pr_info("%s(): \n" , __func__);

	chip->spi	= spi;
	chip->buf_int	= pdata->gpio_buf_flip_int;
	chip->ubit_pin	= pdata->gpio_u_bits;
	chip->bls_pin	= pdata->gpio_bls;
	chip->reset_pin	= pdata->gpio_reset;

        spi_set_drvdata(spi, chip);

	ret = dit4192_gpio_init(chip);
	if( ret < 0 ) {
		return ret;
	}

	dit4192_poweron(chip);

	ret = dit4192_check_device(chip);
	if (ret < 0) {
		dit4192_gpio_free(chip);
		return ret;
	}
		
	/* send initial values */
	dit4192_init(chip);

	ret = snd_soc_register_codec(	&spi->dev,
					&soc_codec_dev_dit4192, 
					dit4192_dai, 
					ARRAY_SIZE(dit4192_dai));

	return 0;
}

static int __devexit dit4192_spi_remove(struct spi_device *spi)
{
        struct dit4192 *chip = spi_get_drvdata(spi);

        spi_set_drvdata(chip->spi, NULL);
        chip->spi = NULL;

	dit4192_gpio_free(chip);

        return 0;
}

static struct spi_driver dit4192_spi_driver = {
	.driver = {
		.name   = "dit4192-spdif",
		.owner  = THIS_MODULE,
	},
	.probe	= dit4192_spi_probe,
	.remove	= __devexit_p(dit4192_spi_remove),
};

static int __init dit4192_module_init(void)
{
	int ret;

	ret = spi_register_driver(&dit4192_spi_driver);
	if (ret) {
		pr_err("Failed to register DIT4192 SPI driver: %d\n", ret);
		return ret;
	}

	return 0;
}

static void __exit dit4192_module_exit(void)
{
	spi_unregister_driver(&dit4192_spi_driver);
	pr_info("module exit\n");
}

module_init(dit4192_module_init)
module_exit(dit4192_module_exit)

MODULE_DESCRIPTION("DIT4192 interface driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kevin Huang <ziqiangh@lab126.com>");
