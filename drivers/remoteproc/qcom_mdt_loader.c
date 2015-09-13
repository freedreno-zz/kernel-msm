/*
 * Qualcomm Peripheral Image Loader
 *
 * Copyright (C) 2014 Sony Mobile Communications AB
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/remoteproc.h>
#include <linux/interrupt.h>
#include <linux/memblock.h>
#include <linux/of.h>
#include <linux/elf.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/qcom_scm.h>
#include <linux/soc/qcom/smem.h>
#include <linux/soc/qcom/smem_state.h>

#include "remoteproc_internal.h"

/**
 * struct pil_mdt - Representation of <name>.mdt file in memory
 * @hdr: ELF32 header
 * @phdr: ELF32 program headers
 */
struct mdt_hdr {
	struct elf32_hdr hdr;
	struct elf32_phdr phdr[];
};

#define segment_is_hash(flag) (((flag) & (0x7 << 24)) == (0x2 << 24))

static int segment_is_loadable(const struct elf32_phdr *p)
{
	return (p->p_type == PT_LOAD) &&
	       !segment_is_hash(p->p_flags) &&
	       p->p_memsz;
}

/**
 * rproc_mdt_sanity_check() - sanity check mdt firmware header
 * @rproc: the remote processor handle
 * @fw: the mdt header firmware image
 */
int qcom_mdt_sanity_check(struct rproc *rproc,
				  const struct firmware *fw)
{
	struct elf32_hdr *ehdr;
	struct mdt_hdr *mdt;

	if (!fw) {
		dev_err(&rproc->dev, "failed to load %s\n", rproc->name);
		return -EINVAL;
	}

	if (fw->size < sizeof(struct elf32_hdr)) {
		dev_err(&rproc->dev, "image is too small\n");
		return -EINVAL;
	}

	mdt = (struct mdt_hdr *)fw->data;
	ehdr = &mdt->hdr;

	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG)) {
		dev_err(&rproc->dev, "image is corrupted (bad magic)\n");
		return -EINVAL;
	}

	if (ehdr->e_phnum == 0) {
		dev_err(&rproc->dev, "no loadable segments\n");
		return -EINVAL;
	}

	if (sizeof(struct elf32_phdr) * ehdr->e_phnum +
	    sizeof(struct elf32_hdr) > fw->size) {
		dev_err(&rproc->dev, "firmware size is too small\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mdt_sanity_check);

struct resource_table * qcom_mdt_find_rsc_table(struct rproc *rproc,
						    const struct firmware *fw,
						    int *tablesz)
{
	static struct resource_table table = { .ver = 1, };

	*tablesz = sizeof(table);
	return &table;
}
EXPORT_SYMBOL_GPL(qcom_mdt_find_rsc_table);

static int qproc_load_segment(struct rproc *rproc, const char *fw_name, const struct elf32_phdr *phdr)
{
	const struct firmware *fw;
	void *ptr;
	int ret = 0;

	ptr = ioremap(phdr->p_paddr, phdr->p_memsz);
	if (!ptr) {
		dev_err(&rproc->dev, "failed to ioremap segment area (0x%x+0x%x)\n", phdr->p_paddr, phdr->p_memsz);
		return -EBUSY;
	}

	if (phdr->p_filesz) {
		ret = request_firmware(&fw, fw_name, &rproc->dev);
		if (ret) {
			dev_err(&rproc->dev, "failed to load %s\n", fw_name);
			goto out;
		}

		memcpy(ptr, fw->data, fw->size);

		release_firmware(fw);
	}

	if (phdr->p_memsz > phdr->p_filesz)
		memset(ptr + phdr->p_filesz, 0, phdr->p_memsz - phdr->p_filesz);

out:
	iounmap(ptr);
	return ret;
}

int qcom_mdt_load(struct rproc *rproc, unsigned pas_id, const struct firmware *fw)
{
	const struct elf32_phdr *phdr;
	const struct elf32_hdr *ehdr;
	const struct mdt_hdr *mdt;
	phys_addr_t min_addr = (phys_addr_t)ULLONG_MAX;
	phys_addr_t max_addr = 0;
	unsigned fw_name_len;
	char *fw_name;
	int ret;
	int i;

	mdt = (struct mdt_hdr *)fw->data;
	ehdr = &mdt->hdr;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		if (phdr->p_paddr < min_addr)
			min_addr = phdr->p_paddr;

		if (phdr->p_paddr + phdr->p_memsz > max_addr)
			max_addr = round_up(phdr->p_paddr + phdr->p_memsz, SZ_4K);
	}

	ret = qcom_scm_pas_init_image(pas_id, fw->data, fw->size);
	if (ret) {
		dev_err(&rproc->dev, "Invalid firmware metadata\n");
		return -EINVAL;
	}

	dev_dbg(&rproc->dev, "pas_mem_setup(0x%x, 0x%x)\n", min_addr, max_addr - min_addr);

	ret = qcom_scm_pas_mem_setup(pas_id, min_addr, max_addr - min_addr);
	if (ret) {
		dev_err(&rproc->dev, "unable to setup memory for image\n");
		return -EINVAL;
	}

	fw_name = kstrdup(rproc->firmware, GFP_KERNEL);
	if (!fw_name)
		return -ENOMEM;

	fw_name_len = strlen(fw_name);
	if (fw_name_len <= 4)
		return -EINVAL;

	for (i = 0; i < ehdr->e_phnum; i++) {
		phdr = &mdt->phdr[i];

		if (!segment_is_loadable(phdr))
			continue;

		sprintf(fw_name + fw_name_len - 3, "b%02d", i);
		ret = qproc_load_segment(rproc, fw_name, phdr);
		if (ret)
			break;
	}

	kfree(fw_name);

	return 0;
}
EXPORT_SYMBOL_GPL(qcom_mdt_load);
