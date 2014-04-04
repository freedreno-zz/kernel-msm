/*
 * DIT4192 ASoC codec driver
 *
 * Copyright (C) 2013 Amazon.com inc. or its affillates. All rights reserved.
 *
 */

#ifndef __SOUND_DIT4192_H
#define __SOUND_DIT4192_H

#define CHANNEL_STATUS_SIZE	24

#define DIT4192_MAX_DATA_SIZE (CHANNEL_STATUS_SIZE+2)

struct dit4192_platform_data {
	int gpio_buf_flip_int;
	int gpio_u_bits;
	int gpio_bls;
	int gpio_reset;
};

struct snd_pcm_substream;

/* DIT4192 register defines */
#define DIT4192_REG_0		0
#define DIT4192_REG_1		1
#define DIT4192_REG_2		2
#define DIT4192_REG_3		3
#define DIT4192_REG_4		4
#define DIT4192_REG_5		5
#define DIT4192_REG_6		6
#define DIT4192_REG_7		7
#define DIT4192_CS_START	8

#define DIT4192_BIT_BLSM_IN	0x00
#define DIT4192_BIT_BLSM_OUT	0x01
#define DIT4192_BIT_VAL_LPCM	0x00
#define DIT4192_BIT_VAL_DATA	0x02
#define DIT4192_BIT_MUTE	0x01
#define DIT4192_BIT_BYPASS	0x02
#define DIT4192_BIT_MONO	0x04
#define DIT4192_BIT_MDAT	0x08
#define DIT4192_BIT_MCSD	0x10
#define DIT4192_BIT_TXOFF	0x20
#define DIT4192_BIT_PDN_OP	0x00
#define DIT4192_BIT_PDN_DOWN	0x01
#define DIT4192_BIT_CLK_MASK	0x06
#define DIT4192_BIT_CLK_128_FS	0x00
#define DIT4192_BIT_CLK_256_FS	0x02
#define DIT4192_BIT_CLK_384_FS	0x04
#define DIT4192_BIT_CLK_512_FS	0x06
#define DIT4192_BIT_RST		0x08
#define DIT4192_BIT_MS_SLAVE 	0x00
#define DIT4192_BIT_MS_MASTER 	0x01
#define DIT4192_BIT_SCLKR	0x02
#define DIT4192_BIT_WLEN_MASK	0x0C
#define DIT4192_BIT_WLEN_24BITS	0x00
#define DIT4192_BIT_WLEN_20BITS	0x04
#define DIT4192_BIT_WLEN_18BITS	0x08
#define DIT4192_BIT_WLEN_16BITS	0x0C
#define DIT4192_BIT_JUS_MASK	0x10
#define DIT4192_BIT_JUS_LEFT	0x00
#define DIT4192_BIT_JUS_RIGHT	0x10
#define DIT4192_BIT_DELAY	0x20
#define DIT4192_BIT_ISCLK	0x40
#define DIT4192_BIT_LEFT_LOW	0x80
#define DIT4192_BIT_BTI		0x01
#define DIT4192_BIT_TSLIP	0x02
#define DIT4192_BIT_MBTI	0x01
#define DIT4192_BIT_MTSLIP	0x02
#define DIT4192_BIT_BSSL	0x04
#define DIT4192_BIT_BTIM	0x03
#define DIT4192_BIT_TSLIPM	0x0C
#define DIT4192_BIT_BTD_ON	0x01
#define DIT4192_BIT_BTD_OFF	0x00


/* DIT4192 R/W Command */
#define DIT4192_CMD_W		0x00
#define DIT4192_CMD_R		0x80
#define DIT4192_IO_STEP_1	0x00
#define DIT4192_IO_STEP_2	0x40
#define DIT4192_HEADER_0	0x00
#define DIT4192_HEADER_1	0x01
#define DIT4192_CMD_DATA	0x02

int snd_dit4192_create( void );
int snd_dit4192_reg_write( void );
int snd_dit4192_iec958_build( void );
int snd_dit4192_iec958_active( void );
int snd_dit4192_iec958_pcm( void );
#endif /* __SOUND_DIT4192_H */
