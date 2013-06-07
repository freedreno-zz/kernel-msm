/*
 * drivers/staging/omapdrm/omap_drv.c
 *
 * Copyright (C) 2013 Rob Clark <robclark@freedesktop.org>
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

#ifndef __REG_LOG_H_
#define __REG_LOG_H_

#include <linux/io.h>
#include <asm/io.h>
#include <linux/types.h>

void __log_ioremap(void __iomem *base, u32 size, const char *name);

void __log_add(u32 val, u32 addr, const char *func, u32 line, u32 op);

#undef __raw_writel
#undef __raw_readl

#define __raw_writel(val, addr) (void)({ \
	__log_add((val), (u32)(addr), __func__, __LINE__, 1); \
	__raw_writel_no_log(val, addr); \
})

#define __raw_readl(addr) ({ \
	u32 _val = __raw_readl_no_log(addr); \
	__log_add(_val, (u32)(addr), __func__, __LINE__, 0); \
	_val; \
})

#endif
