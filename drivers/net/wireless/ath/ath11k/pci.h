/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Copyright (c) 2019-2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2021-2022, Qualcomm Innovation Center, Inc. All rights reserved.
 */
#ifndef _ATH11K_PCI_H
#define _ATH11K_PCI_H

#include <linux/mhi.h>

#include "core.h"
#include "coredump_fw.h"

#define PCIE_SOC_GLOBAL_RESET			0x3008
#define PCIE_SOC_GLOBAL_RESET_V			1

#define WLAON_WARM_SW_ENTRY			0x1f80504
#define WLAON_RESET_DBG_SW_ENTRY		0x01F80508
#define WLAON_SOC_RESET_CAUSE_REG		0x01f8060c

#define PCIE_Q6_COOKIE_ADDR			0x01f80500
#define PCIE_Q6_COOKIE_DATA			0xc0000000

/* register to wake the UMAC from power collapse */
#define PCIE_SCRATCH_0_SOC_PCIE_REG		0x4040

/* register used for handshake mechanism to validate UMAC is awake */
#define PCIE_SOC_WAKE_PCIE_LOCAL_REG		0x3004

#define PCIE_PCIE_PARF_LTSSM			0x1e081b0
#define PARM_LTSSM_VALUE			0x111

#define GCC_GCC_PCIE_HOT_RST			0x1e402bc
#define GCC_GCC_PCIE_HOT_RST_VAL		0x10

#define PCIE_PCIE_INT_ALL_CLEAR			0x1e08228
#define PCIE_SMLH_REQ_RST_LINK_DOWN		0x2
#define PCIE_INT_CLEAR_ALL			0xffffffff

#define PCIE_QSERDES_COM_SYSCLK_EN_SEL_REG(x) \
		(ab->hw_params.regs->pcie_qserdes_sysclk_en_sel)
#define PCIE_QSERDES_COM_SYSCLK_EN_SEL_VAL	0x10
#define PCIE_QSERDES_COM_SYSCLK_EN_SEL_MSK	0xffffffff
#define PCIE_PCS_OSC_DTCT_CONFIG1_REG(x) \
		(ab->hw_params.regs->pcie_pcs_osc_dtct_config_base)
#define PCIE_PCS_OSC_DTCT_CONFIG1_VAL		0x02
#define PCIE_PCS_OSC_DTCT_CONFIG2_REG(x) \
		(ab->hw_params.regs->pcie_pcs_osc_dtct_config_base + 0x4)
#define PCIE_PCS_OSC_DTCT_CONFIG2_VAL		0x52
#define PCIE_PCS_OSC_DTCT_CONFIG4_REG(x) \
		(ab->hw_params.regs->pcie_pcs_osc_dtct_config_base + 0xc)
#define PCIE_PCS_OSC_DTCT_CONFIG4_VAL		0xff
#define PCIE_PCS_OSC_DTCT_CONFIG_MSK		0x000000ff

#define WLAON_QFPROM_PWR_CTRL_REG		0x01f8031c
#define QFPROM_PWR_CTRL_VDD4BLOW_MASK		0x4

#define HWIO_PCIE_PCIE_BHI_VERSION_LOWER_ADDR           (0x1e0e200)
#define HWIO_PCIE_PCIE_BHI_VERSION_UPPER_ADDR           (0x1e0e204)
#define HWIO_PCIE_PCIE_BHI_IMGADDR_LOWER_ADDR           (0x1e0e208)
#define HWIO_PCIE_PCIE_BHI_IMGADDR_UPPER_ADDR           (0x1e0e20c)
#define HWIO_PCIE_PCIE_BHI_IMGSIZE_ADDR                 (0x1e0e210)
#define HWIO_PCIE_PCIE_BHI_IMGTXDB_ADDR                 (0x1e0e218)
#define HWIO_PCIE_PCIE_BHI_INTVEC_ADDR                  (0x1e0e220)
#define HWIO_PCIE_PCIE_BHI_EXECENV_ADDR                 (0x1e0e228)
#define HWIO_PCIE_PCIE_BHI_STATUS_ADDR                  (0x1e0e22c)
#define HWIO_PCIE_PCIE_BHI_ERRCODE_ADDR                 (0x1e0e230)
#define HWIO_PCIE_PCIE_BHI_ERRDBG1_ADDR                 (0x1e0e234)
#define HWIO_PCIE_PCIE_BHI_ERRDBG2_ADDR                 (0x1e0e238)
#define HWIO_PCIE_PCIE_BHI_ERRDBG3_ADDR                 (0x1e0e23c)
#define HWIO_PCIE_PCIE_BHI_SERIALNUM_ADDR               (0x1e0e240)
#define HWIO_PCIE_PCIE_BHI_SBLANTIROLLVER_ADDR          (0x1e0e244)
#define HWIO_PCIE_PCIE_BHI_NUMSEG_ADDR                  (0x1e0e248)
#define HWIO_PCIE_PCIE_BHI_MSMHWID_0_ADDR               (0x1e0e24c)
#define HWIO_PCIE_PCIE_BHI_MSMHWID_1_ADDR               (0x1e0e250)
#define HWIO_PCIE_PCIE_BHI_MSMHWID_2_ADDR               (0x1e0e254)
#define HWIO_PCIE_PCIE_BHI_MSMHWID_3_ADDR               (0x1e0e258)
#define HWIO_PCIE_PCIE_BHI_MSMHWID_4_ADDR               (0x1e0e25c)
#define HWIO_PCIE_PCIE_BHI_MSMHWID_5_ADDR               (0x1e0e260)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_0_ADDR             (0x1e0e264)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_1_ADDR             (0x1e0e268)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_2_ADDR             (0x1e0e26c)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_3_ADDR             (0x1e0e270)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_4_ADDR             (0x1e0e274)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_5_ADDR             (0x1e0e278)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_6_ADDR             (0x1e0e27c)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_7_ADDR             (0x1e0e280)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_8_ADDR             (0x1e0e284)
#define HWIO_PCIE_PCIE_BHI_OEMPKHASH_9_ADDR             (0x1e0e288)
#define HWIO_PCIE_PCIE_BHI_TXVECDB_ADDR                 (0x1e0e360)
#define HWIO_PCIE_PCIE_BHI_TXVECSTATUS_ADDR             (0x1e0e368)
#define HWIO_PCIE_PCIE_BHI_RXVECDB_ADDR                 (0x1e0e394)
#define HWIO_PCIE_PCIE_BHI_RXVECSTATUS_ADDR             (0x1e0e39c)

#define QDSS_APB_DEC_CS_QDSSCSR_ETRIRQCTRL  (0x1C0106C)
#define QDSS_APB_DEC_CS_QDSSCSR_PRESERVEETF  (0x1C01070)
#define QDSS_APB_DEC_CS_QDSSCSR_PRESERVEETR0  (0x1C01074)
#define QDSS_APB_DEC_CS_QDSSCSR_PRESERVEETR1  (0x1C01078)
#define Q6SS_PRIVCSR_QDSP6SS_QTMR_V1_CNTP_CTL_0 (0x00DA102C)
#define Q6SS_PRIVCSR_QDSP6SS_QTMR_V1_CNTP_CTL_1 (0x00DA202C)
#define Q6SS_PRIVCSR_QDSP6SS_QTMR_V1_CNTP_CTL_2 (0x00DA302C)

struct cnss_pci_reg {
	char *name;
	u32 offset;
	u32 value;
};

struct register_crash_data {
	u8 *reg_buf;
	size_t reg_buf_len;
	u8 *reg_rddm_buf;
	size_t reg_rddm_buf_len;
};


enum ath11k_pci_flags {
	ATH11K_PCI_ASPM_RESTORE,
};

struct ath11k_pci {
	struct pci_dev *pdev;
	struct ath11k_base *ab;
	u16 dev_id;
	char amss_path[100];
	struct mhi_controller *mhi_ctrl;
	const struct ath11k_msi_config *msi_config;
	u32 register_window;

	/* protects register_window above */
	spinlock_t window_lock;

	/* enum ath11k_pci_flags */
	unsigned long flags;
	u16 link_ctl;
	struct register_crash_data reg_data;
	struct ath11k_mhi_fw_crash_data mhi_fw_crash_data;

};

static inline struct ath11k_pci *ath11k_pci_priv(struct ath11k_base *ab)
{
	return (struct ath11k_pci *)ab->drv_priv;
}

int ath11k_pci_get_msi_irq(struct ath11k_base *ab, unsigned int vector);
void ath11k_pci_register_dump(struct ath11k_pci *ab_pci);
#endif
