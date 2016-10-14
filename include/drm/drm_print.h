/*
 * Copyright (C) 2016 Red Hat
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Rob Clark <robdclark@gmail.com>
 */

#ifndef DRM_PRINT_H_
#define DRM_PRINT_H_

#include <stdarg.h>
#include <linux/seq_file.h>

/* A simple wrapper to abstract seq_file vs printk, so same logging code
 * does not have to be duplicated.
 */
struct drm_print {
	void (*printfn)(struct drm_print *p, const char *f, va_list args);
	void *arg;
};

void __drm_printfn_seq_file(struct drm_print *p, const char *f, va_list args);
void __drm_printfn_info(struct drm_print *p, const char *f, va_list args);

void drm_printf(struct drm_print *p, const char *f, ...);

static inline struct drm_print drm_print_seq_file(struct seq_file *f)
{
	struct drm_print p = {
		.printfn = __drm_printfn_seq_file,
		.arg = f,
	};
	return p;
}

static inline struct drm_print drm_print_info(void)
{
	struct drm_print p = {
		.printfn = __drm_printfn_info,
	};
	return p;
}

#endif /* DRM_PRINT_H_ */
