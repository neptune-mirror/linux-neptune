// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) 2020 The Linux Foundation. All rights reserved.
 */

#include <linux/devcoredump.h>
#include "coredump.h"
#include "pci.h"

static struct ath11k_dump_file_data *
ath11k_coredump_build(struct ath11k_mhi_fw_crash_data *crash_data,
		      struct fw_remote_crash_data *remote_crash_data,
		      struct register_crash_data *reg_crash_data)
{
	struct ath11k_dump_file_data *dump_data;
	struct ath11k_tlv_dump_data *dump_tlv;
	size_t hdr_len = sizeof(*dump_data);
	size_t len, sofar = 0;
	unsigned char *buf;
	struct timespec64 timestamp;

	len = hdr_len;

	len += sizeof(*dump_tlv) + crash_data->paging_dump_buf_len;
	len += sizeof(*dump_tlv) + crash_data->ramdump_buf_len;
	len += sizeof(*dump_tlv) + remote_crash_data->remote_buf_len;
	len += sizeof(*dump_tlv) + reg_crash_data->reg_buf_len;
	len += sizeof(*dump_tlv) + reg_crash_data->reg_rddm_buf_len;
	sofar += hdr_len;

	buf = vzalloc(len);
	if (!buf)
		return NULL;

	dump_data = (struct ath11k_dump_file_data *)(buf);
	strlcpy(dump_data->df_magic, "ATH11K-FW-DUMP",
		sizeof(dump_data->df_magic));
	dump_data->len = cpu_to_le32(len);
	dump_data->version = cpu_to_le32(ATH11K_FW_CRASH_DUMP_VERSION);
	guid_gen(&dump_data->guid);
	ktime_get_real_ts64(&timestamp);
	dump_data->tv_sec = cpu_to_le64(timestamp.tv_sec);
	dump_data->tv_nsec = cpu_to_le64(timestamp.tv_nsec);

	/* Gather FW paging dump */
	dump_tlv = (struct ath11k_tlv_dump_data *)(buf + sofar);
	dump_tlv->type = cpu_to_le32(ATH11K_FW_CRASH_PAGING_DATA);
	dump_tlv->tlv_len = cpu_to_le32(crash_data->paging_dump_buf_len);
	memcpy(dump_tlv->tlv_data, crash_data->paging_dump_buf,
	       crash_data->paging_dump_buf_len);
	sofar += sizeof(*dump_tlv) + crash_data->paging_dump_buf_len;

	/* Gather RDDM dump */
	dump_tlv = (struct ath11k_tlv_dump_data *)(buf + sofar);
	dump_tlv->type = cpu_to_le32(ATH11K_FW_CRASH_RDDM_DATA);
	dump_tlv->tlv_len = cpu_to_le32(crash_data->ramdump_buf_len);
	memcpy(dump_tlv->tlv_data, crash_data->ramdump_buf,
	       crash_data->ramdump_buf_len);
	sofar += sizeof(*dump_tlv) + crash_data->ramdump_buf_len;

	/* gather remote memory */
	dump_tlv = (struct ath11k_tlv_dump_data *)(buf + sofar);
	dump_tlv->type = cpu_to_le32(ATH11K_FW_REMOTE_MEM_DATA);
	dump_tlv->tlv_len = cpu_to_le32(remote_crash_data->remote_buf_len);
	memcpy(dump_tlv->tlv_data, remote_crash_data->remote_buf,
	       remote_crash_data->remote_buf_len);
	sofar += sizeof(*dump_tlv) + remote_crash_data->remote_buf_len;

	/* gather register memory */
	dump_tlv = (struct ath11k_tlv_dump_data *)(buf + sofar);
	dump_tlv->type = cpu_to_le32(ATH11K_FW_REGISTER_DATA);
	dump_tlv->tlv_len = cpu_to_le32(reg_crash_data->reg_buf_len);
	memcpy(dump_tlv->tlv_data, reg_crash_data->reg_buf,
	       reg_crash_data->reg_buf_len);
	sofar += sizeof(*dump_tlv) + reg_crash_data->reg_buf_len;

	/* gather register for rddm fail */
	dump_tlv = (struct ath11k_tlv_dump_data *)(buf + sofar);
	dump_tlv->type = cpu_to_le32(ATH11K_FW_REGISTER_RDDM_DATA);
	dump_tlv->tlv_len = cpu_to_le32(reg_crash_data->reg_rddm_buf_len);
	memcpy(dump_tlv->tlv_data, reg_crash_data->reg_rddm_buf,
	       reg_crash_data->reg_rddm_buf_len);
	sofar += sizeof(*dump_tlv) + reg_crash_data->reg_rddm_buf_len;

	return dump_data;
}

static int ath11k_coredump_submit(struct ath11k_pci *ab_pci)
{
	struct ath11k_dump_file_data *dump;

	dump = ath11k_coredump_build(&ab_pci->mhi_fw_crash_data,
				     &ab_pci->ab->remote_crash_data,
				     &ab_pci->reg_data);
	if (!dump)
		return -ENODATA;

	dev_coredumpv(ab_pci->mhi_ctrl->cntrl_dev, dump,
		      le32_to_cpu(dump->len), GFP_KERNEL);

	return 0;
}

static void ath11k_coredump_buf_release(struct ath11k_pci *ab_pci)
{
	struct fw_remote_crash_data *remote = &ab_pci->ab->remote_crash_data;
	struct ath11k_mhi_fw_crash_data *mhi = &ab_pci->mhi_fw_crash_data;
	struct register_crash_data *reg = &ab_pci->reg_data;

	if (remote->remote_buf) {
		vfree(remote->remote_buf);
		remote->remote_buf = NULL;
	}

	if (mhi->ramdump_buf) {
		vfree(mhi->ramdump_buf);
		mhi->ramdump_buf = NULL;
	}

	if (mhi->paging_dump_buf) {
		vfree(mhi->paging_dump_buf);
		mhi->paging_dump_buf = NULL;
	}

	if (reg->reg_buf) {
		vfree(reg->reg_buf);
		reg->reg_buf = NULL;
	}

	if (reg->reg_rddm_buf) {
		vfree(reg->reg_rddm_buf);
		reg->reg_rddm_buf = NULL;
	}
}
