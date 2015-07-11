/*
 * Copyright (C) 2013-2015 Kay Sievers
 * Copyright (C) 2013-2015 Greg Kroah-Hartman <gregkh@linuxfoundation.org>
 * Copyright (C) 2013-2015 Daniel Mack <daniel@zonque.org>
 * Copyright (C) 2013-2015 David Herrmann <dh.herrmann@gmail.com>
 * Copyright (C) 2013-2015 Linux Foundation
 * Copyright (C) 2014-2015 Djalal Harouni <tixxdz@opendz.org>
 *
 * kdbus is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 */

#ifndef __KDBUS_ITEM_H
#define __KDBUS_ITEM_H

#include <linux/kernel.h>
#include <uapi/linux/kdbus.h>

#include "util.h"

/* generic access and iterators over a stream of items */
#define KDBUS_ITEM_NEXT(_i) (typeof(_i))((u8 *)(_i) + KDBUS_ALIGN8((_i)->size))
#define KDBUS_ITEMS_SIZE(_h, _is) ((_h)->size - offsetof(typeof(*(_h)), _is))
#define KDBUS_ITEM_HEADER_SIZE offsetof(struct kdbus_item, data)
#define KDBUS_ITEM_SIZE(_s) KDBUS_ALIGN8(KDBUS_ITEM_HEADER_SIZE + (_s))
#define KDBUS_ITEM_PAYLOAD_SIZE(_i) ((_i)->size - KDBUS_ITEM_HEADER_SIZE)

#define KDBUS_ITEMS_FOREACH(_i, _is, _s)				\
	for ((_i) = (_is);						\
	     ((u8 *)(_i) < (u8 *)(_is) + (_s)) &&			\
	       ((u8 *)(_i) >= (u8 *)(_is));				\
	     (_i) = KDBUS_ITEM_NEXT(_i))

#define KDBUS_ITEM_VALID(_i, _is, _s)					\
	((_i)->size >= KDBUS_ITEM_HEADER_SIZE &&			\
	 (u8 *)(_i) + (_i)->size > (u8 *)(_i) &&			\
	 (u8 *)(_i) + (_i)->size <= (u8 *)(_is) + (_s) &&		\
	 (u8 *)(_i) >= (u8 *)(_is))

#define KDBUS_ITEMS_END(_i, _is, _s)					\
	((u8 *)(_i) == ((u8 *)(_is) + KDBUS_ALIGN8(_s)))

/**
 * struct kdbus_item_header - Describes the fix part of an item
 * @size:	The total size of the item
 * @type:	The item type, one of KDBUS_ITEM_*
 */
struct kdbus_item_header {
	u64 size;
	u64 type;
};

int kdbus_item_validate_name(const struct kdbus_item *item);
int kdbus_item_validate(const struct kdbus_item *item);
int kdbus_items_validate(const struct kdbus_item *items, size_t items_size);
const char *kdbus_items_get_str(const struct kdbus_item *items,
				size_t items_size,
				unsigned int item_type);
struct kdbus_item *kdbus_item_set(struct kdbus_item *item, u64 type,
				  const void *data, size_t len);

#endif
