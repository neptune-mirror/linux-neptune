// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#include "coredump_fw.h"
#include "pci.h"
#include "debug.h"

static size_t ath11k_get_paging_buf_len(struct mhi_controller *mhi_cntrl)
{
	struct image_info *img = mhi_cntrl->fbc_image;
	int i = 0;
	size_t len = 0;
	u32 entries = img->entries;
	size_t seg_size = mhi_cntrl->seg_len;

	for (i = 0; i < entries; ++i) {
		size_t vec_size = seg_size;

		if (i == entries - 1)
			vec_size = sizeof(struct ath11k_vec_entry) * i;
		len += vec_size;
	}

	return len;
}

int ath11k_coredump_fw_paging_dump(struct ath11k_pci *ab_pci, struct mhi_controller *mhi_cntrl)
{
	struct image_info *img = mhi_cntrl->fbc_image;
	struct ath11k_mhi_fw_crash_data *crash_data = &ab_pci->mhi_fw_crash_data;
	char *buf = NULL;
	unsigned int size = 0;
	int seg = 0;
	u32 offset = 0;
	u32 fw_vec_entry_num = img->entries - 1;
	size_t paging_dump_buf_len = ath11k_get_paging_buf_len(mhi_cntrl);

	crash_data->paging_dump_buf_len = paging_dump_buf_len;
	dev_info(&mhi_cntrl->mhi_dev->dev,
		 "%s FW paging dump buffer len=%lu\n",
		 __func__, paging_dump_buf_len);
	crash_data->paging_dump_buf = vzalloc(paging_dump_buf_len);
	if (!crash_data->paging_dump_buf)
		return -ENOMEM;

	for (seg = 0; seg < fw_vec_entry_num; seg++) {
		buf = img->mhi_buf[seg].buf;
		size = img->mhi_buf[seg].len;
		memcpy(crash_data->paging_dump_buf + offset, buf, size);
		offset += size;
	}

	buf = crash_data->paging_dump_buf + offset;
	size = img->mhi_buf[img->entries - 1].len;
	ath11k_info(ab_pci->ab, "to write last block: mem: 0x%p, size: 0x%x\n",
		    buf, size);
	memcpy(buf, img->mhi_buf[img->entries - 1].buf, size);

	return 0;
}

int ath11k_coredump_fw_rddm_dump(struct ath11k_pci *ab_pci, struct mhi_controller *mhi_cntrl)
{
	struct ath11k_mhi_fw_crash_data *crash_data = &ab_pci->mhi_fw_crash_data;
	struct image_info *img = mhi_cntrl->rddm_image;
	char *buf = NULL;
	unsigned int size = 0;
	int seg = 0;
	u32 offset = 0;
	u32 rddm_vec_entry_num;
	u32 entries = mhi_cntrl->rddm_image->entries;

	rddm_vec_entry_num = DIV_ROUND_UP(RDDM_DUMP_SIZE,
					  mhi_cntrl->seg_len);
	crash_data->ramdump_buf_len = (entries - 1) * mhi_cntrl->seg_len +
				      rddm_vec_entry_num * sizeof(struct ath11k_vec_entry);

	ath11k_info(ab_pci->ab, "rddm_vec_entry_num=%d entries=%d\n",
		    rddm_vec_entry_num, img->entries);

	crash_data->ramdump_buf = vzalloc(crash_data->ramdump_buf_len);
	if (!crash_data->ramdump_buf)
		return -ENOMEM;

	for (seg = 0; seg < rddm_vec_entry_num; seg++) {
		buf = img->mhi_buf[seg].buf;
		size = img->mhi_buf[seg].len;
		ath11k_info(ab_pci->ab,
			    "write rddm memory: mem: 0x%p, size: 0x%x\n",
			    buf, size);
		memcpy(crash_data->ramdump_buf + offset, buf, size);
		offset += size;
	}

	buf = crash_data->ramdump_buf + offset;
	size = img->mhi_buf[img->entries - 1].len;
	ath11k_info(ab_pci->ab,
		    "to write vector table block: mem: 0x%p, size: 0x%x\n",
		    buf, size);
	memcpy(buf, img->mhi_buf[img->entries - 1].buf, size);

	return 0;
}
