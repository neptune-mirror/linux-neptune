/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#ifndef _COREDUMP_FW_H_
#define _COREDUMP_FW_H_

#include <linux/mhi.h>

struct ath11k_pci;

struct ath11k_vec_entry {
	u64 dma_addr;
	u64 size;
};

struct ath11k_mhi_fw_crash_data {
	u8 *paging_dump_buf;
	size_t paging_dump_buf_len;
};

int ath11k_coredump_fw_paging_dump(struct ath11k_pci *ab_pci, struct mhi_controller *mhi_cntrl);

#endif
