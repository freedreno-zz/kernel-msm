/*
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
	printk(KERN_ERR "IO:W %08x %08x (%s:%d)\n", (u32)(addr), (u32)(val), __FILE__, __LINE__); \
})

#define __raw_readl(addr) ({ \
	u32 _val = __raw_readl_no_log(addr); \
	__log_add(_val, (u32)(addr), __func__, __LINE__, 0); \
	printk(KERN_ERR "IO:R %08x %08x (%s:%d)\n", (u32)(addr), _val, __FILE__, __LINE__); \
	_val; \
})

#include <linux/clk.h>

const char *clk_name(struct clk *clk);


#define __clk_enable            clk_enable
#define __clk_prepare_enable    clk_prepare_enable
#define __clk_disable           clk_disable
#define __clk_disable_unprepare clk_disable_unprepare
#define __clk_prepare           clk_prepare
#define __clk_unprepare         clk_unprepare
#define __clk_set_rate          clk_set_rate

#define clk_enable(c)             ({printk(KERN_ERR"CLK: clk_enable(%s) (%s:%d)\n", clk_name(c), __FILE__, __LINE__);            __clk_enable(c);})
#define clk_disable(c)            ({printk(KERN_ERR"CLK: clk_disable(%s) (%s:%d)\n", clk_name(c), __FILE__, __LINE__);           __clk_disable(c);})
#define clk_prepare_enable(c)     ({printk(KERN_ERR"CLK: clk_prepare_enable(%s) (%s:%d)\n", clk_name(c), __FILE__, __LINE__);    __clk_prepare_enable(c);})
#define clk_disable_unprepare(c)  ({printk(KERN_ERR"CLK: clk_disable_unprepare(%s) (%s:%d)\n", clk_name(c), __FILE__, __LINE__); __clk_disable_unprepare(c);})
#define clk_prepare(c)            ({printk(KERN_ERR"CLK: clk_prepare(%s) (%s:%d)\n", clk_name(c), __FILE__, __LINE__);           __clk_prepare(c);})
#define clk_unprepare(c)          ({printk(KERN_ERR"CLK: clk_unprepare(%s) (%s:%d)\n", clk_name(c), __FILE__, __LINE__);         __clk_unprepare(c);})
#define clk_set_rate(c, r)        ({printk(KERN_ERR"CLK: clk_set_rate(%s, %lu) (%s:%d)\n", clk_name(c), (unsigned long)(r), __FILE__, __LINE__);     __clk_set_rate(c, r);})


#include <linux/regulator/consumer.h>

#define __regulator_enable      regulator_enable
#define __regulator_disable     regulator_disable

const char *regulator_name(struct regulator *r);

#define regulator_enable(r)       ({printk(KERN_ERR"RG: regulator_enable(%s) (%s:%d)\n", regulator_name(r), __FILE__, __LINE__);  __regulator_enable(r);})
#define regulator_disable(r)      ({printk(KERN_ERR"RG: regulator_disable(%s) (%s:%d)\n", regulator_name(r), __FILE__, __LINE__);  __regulator_disable(r);})

#endif
