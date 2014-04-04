/*
 * board IDME driver header file
 *
 * Copyright (C) 2011 Amazon Inc., All Rights Reserved.
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _IDME_INIT_H_
#define _IDME_INIT_H_

#ifdef CONFIG_IDME

#define BOARD_TYPE_OMAP4430 802
#define BOARD_TYPE_OMAP4470 803
#define BOARD_TYPE_MSM8960  804

/* The string must match with uboot */
#define IDME_SERIAL_PROCNAME	"serial"
#define IDME_PROCNAME_BOARDID	"board_id"
#define IDME_PROCNAME_PCBSN	"pcbsn"
#define IDME_PROCNAME_MACADDR	"mac_addr"
#define IDME_PROCNAME_BTMACADDR "bt_mac_addr"
#define IDME_PROCNAME_MACSEC	"mac_sec"
#define IDME_PROCNAME_BOOTMODE	"bootmode"
#define IDME_PROCNAME_POSTMODE	"postmode"
#define IDME_PROCNAME_BOOTCOUNT "bootcount"
#define IDME_PROCNAME_WIFIMODULE "wifi_module"

enum idme_board_type{
	IDME_BOARD_TYPE_OMAP4430=0,
	IDME_BOARD_TYPE_OMAP4470,
	IDME_BOARD_TYPE_OMAP5430,
	IDME_BOARD_TYPE_MSM8960,
};

#define IDME_MAGIC_NUMBER "beefdeed"
#define IDME_MAX_NAME_LEN 16

struct idme_desc {
    char name[IDME_MAX_NAME_LEN];
    unsigned int size;
    unsigned int exportable;
	unsigned int permission;
};

struct item_t {
    struct idme_desc desc;
    unsigned char data[1];
};

struct idme_t {
    char magic[8];
    char version[4];
    unsigned int items_num;
    unsigned char item_data[1];
};

int idme_get_var(char *name, char *buffer, int buflen);
void init_idme(void);
#endif //CONFIG_IDME

#endif //_IDME_INIT_H_
