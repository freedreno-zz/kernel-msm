/*
 * board IDME driver
 *
 * Copyright (C) 2011 Amazon Inc., All Rights Reserved.
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/idme_init.h>

#ifdef CONFIG_IDME

#define DRIVER_VER "2.0"

#define DRIVER_INFO "Idme driver version " DRIVER_VER

#define IDME_PROCNAME_PROD_NAME	"product_name"
#define IDME_PROCNAME_PROD_NAME_EXTRA	"product_name_extra"

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

#define BOARD_TYPE_OFFSET 0   //the top 3 bytes are the board type
#define BOARD_TYPE_LEN 3
#define BOARD_REV_OFFSET (BOARD_TYPE_OFFSET + BOARD_TYPE_LEN)
#define BOARD_REV_LEN 2
#define BOARD_VARIANT_OFFSET (BOARD_REV_OFFSET + BOARD_REV_LEN)
#define BOARD_VARIANT_LEN 2

int idme_get_board_revision(void);
char *idme_get_board_type_string(void);

#define PROD_NAME_EXTRA_DUMMY 0
#define PROD_NAME_EXTRA_ENG 1
#define PROD_NAME_EXTRA_PROD 2

#define IDME_ITEM_NEXT(curr_item) \
	curr_item = (struct item_t *)((char *)curr_item + sizeof(struct idme_desc) + curr_item->desc.size)

const char *prod_name_extra_string[] = {
  "Dummy\0",
  "Eng\0",
  "Prod\0"
};

static unsigned char idme_item_name[50][16];

#define MAX_PROD_NAME_STRING_LEN 256
static char prod_name_string[MAX_PROD_NAME_STRING_LEN];
static int prod_name_extra_index = PROD_NAME_EXTRA_DUMMY;

static int proc_prod_name_func(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        char *str = &prod_name_string[0];
        if ((str) && (strlen(str) < PAGE_SIZE)){
                memcpy(page, str, strlen(str));
                *eof = 1;
                return strlen(str);
        }
        return 0;
}

static int proc_prod_name_extra_func(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        const char *str = prod_name_extra_string[prod_name_extra_index];
        if ((str) && (strlen(str) < PAGE_SIZE)){
                memcpy(page, str, strlen(str));
                *eof = 1;
                return strlen(str);
        }
        return 0;
}

int idme_check_magic_number(struct idme_t *pidme)
{
	if(pidme == NULL) {
		printk("Error, the pointer of pidme_data is NULL.\n");
		return -1;
	}

	if (strncmp(pidme->magic, IDME_MAGIC_NUMBER, strlen(IDME_MAGIC_NUMBER))){
		printk("The current magic number of idme is %s!\n", pidme->magic);
		printk("Error, idme data is invalid!\n");
		return -2;
	}
	return 0;
}

int idme_get_var(char *name, char *buffer, int buflen)
{
	struct idme_t *pidme = (struct idme_t *)&system_idme[0];
	struct item_t *pitem = NULL;
	int i = 0;

	if (0 != idme_check_magic_number(pidme)){
		printk("The idme magic number error.\n");
		return -1;
	}

	pitem = (struct item_t *)(&(pidme->item_data[0]));
	for (i = 0; i < pidme->items_num; i++) {
		if ( 0 == strcmp(name, pitem->desc.name) ) {
			memcpy( buffer, &(pitem->data[0]), MIN(pitem->desc.size, buflen));
			break;
		}else{
			IDME_ITEM_NEXT(pitem);
		}
	}

        return 0;

}

static int proc_idme_read_item(char *page, char **start, off_t off, int count,
                                int *eof, void *data)
{
	struct idme_t *pidme = (struct idme_t *)&system_idme[0];
	struct item_t *pitem = NULL;
	unsigned long bootcount = 0;
	unsigned char idme_data[128] = {'\0'};
	int i = 0;

	if (0 != idme_check_magic_number(pidme)){
		printk("The idme magic number error.\n");
		return -1;
	}

	pitem = (struct item_t *)(&(pidme->item_data[0]));
	for (i = 0; i < pidme->items_num; i++) {
		if ( 0 == strcmp(data, pitem->desc.name) ) {
			memcpy( idme_data, &(pitem->data[0]), pitem->desc.size );
			if ( 0 == strcmp(data, IDME_PROCNAME_BOOTCOUNT) ) {
				memcpy(&bootcount, idme_data, sizeof(unsigned long));
				sprintf(page, "%lu", bootcount);
			} else {
				sprintf(page, "%s", idme_data);
			}
			*eof = 1;
			break;
		}else{
			IDME_ITEM_NEXT(pitem);
		}
	}

	return strlen(page);
}

void create_idme_proc(void)
{
	struct proc_dir_entry *proc_entry;
	struct idme_t *pidme = (struct idme_t *)&system_idme[0];
	struct item_t *pitem = NULL;
	int i = 0;

	if (0 != idme_check_magic_number(pidme)){
		printk("The idme magic number error.\n");
		return;
	}

        printk("IDME: initialize %s\n", DRIVER_INFO);


	pitem = (struct item_t *)(&(pidme->item_data[0]));
	for (i = 0; i < pidme->items_num; i++) {
		if(pitem->desc.exportable > 0) {
			proc_entry = create_proc_entry(pitem->desc.name, pitem->desc.permission, NULL);

			strcpy(idme_item_name[i], pitem->desc.name);

			proc_entry->data = idme_item_name[i];
			proc_entry->read_proc = proc_idme_read_item;
			proc_entry->write_proc = NULL;
		}
		IDME_ITEM_NEXT(pitem);
	}
}


static int __init idme_init_proc(void)
{
	struct proc_dir_entry *proc_prod_name = create_proc_entry(IDME_PROCNAME_PROD_NAME, S_IRUGO, NULL);
	struct proc_dir_entry *proc_prod_name_extra = create_proc_entry(IDME_PROCNAME_PROD_NAME_EXTRA, S_IRUGO, NULL);

        create_idme_proc();

        if (proc_prod_name != NULL) {
                proc_prod_name->data = NULL;
                proc_prod_name->read_proc = proc_prod_name_func;
                proc_prod_name->write_proc = NULL;
        }
        if (proc_prod_name_extra != NULL) {
                proc_prod_name_extra->data = NULL;
                proc_prod_name_extra->read_proc = proc_prod_name_extra_func;
                proc_prod_name_extra->write_proc = NULL;
        }

	return 0;
}

module_init(idme_init_proc);


int mystrtoi(const char *s, const char *e)
{
        int result = 0, value = 0;
        /* first remove all leading 0s */
        while(*s == '0' && s < e){
                s++;
        }

        while(s < e){
                value = *s - '0';
                result = result * 10 + value;
                s++;
        }
        return result;
}


/* return the variant of the board */
int idme_get_board_type(void)
{
        int board_type = 0;
        unsigned char boardid[REVISION16_SIZE+1] = {'\0'};

        idme_get_var(IDME_PROCNAME_BOARDID, boardid, REVISION16_SIZE);

        board_type = mystrtoi(&boardid[BOARD_TYPE_OFFSET],
                              &boardid[BOARD_TYPE_OFFSET+BOARD_TYPE_LEN]);
        return board_type;
}

EXPORT_SYMBOL(idme_get_board_type);

/* return the variant of the board */
int idme_get_board_variant(void)
{
        unsigned char boardid[REVISION16_SIZE+1] = {'\0'};
        idme_get_var(IDME_PROCNAME_BOARDID, boardid, REVISION16_SIZE);

        return mystrtoi(&boardid[BOARD_VARIANT_OFFSET],
                        &boardid[BOARD_VARIANT_OFFSET+BOARD_VARIANT_LEN]);
}

EXPORT_SYMBOL(idme_get_board_variant);


int idme_get_board_revision(void)
{
        unsigned char boardid[REVISION16_SIZE+1] = {'\0'};
        idme_get_var(IDME_PROCNAME_BOARDID, boardid, REVISION16_SIZE);

        return mystrtoi(&boardid[BOARD_REV_OFFSET],
                        &boardid[BOARD_REV_OFFSET+BOARD_REV_LEN]);
}


EXPORT_SYMBOL(idme_get_board_revision);

char *idme_get_board_type_string(void)
{
        switch(idme_get_board_type()){
        case BOARD_TYPE_OMAP4470:
               return "CommonOS OMAP4470\0";
        case BOARD_TYPE_OMAP4430:
				return "CommonOS OMAP4430\0";
		case BOARD_TYPE_MSM8960:
				return "CommonOS MSM8960\0";
		default: return "Unknow\0";
        }
        return "Unknow\0";
}

void init_idme(void)
{
    printk(DRIVER_INFO "\n");
    memset(prod_name_string, 0, sizeof(prod_name_string));

	/* get the product name string and product name extra string */
    strcpy(prod_name_string, idme_get_board_type_string());
}

#endif //CONFIG_IDME
