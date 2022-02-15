/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef _MHI_QCOM_
#define _MHI_QCOM_

/* iova cfg bitmask */
#define MHI_SMMU_ATTACH BIT(0)
#define MHI_SMMU_S1_BYPASS BIT(1)
#define MHI_SMMU_FAST BIT(2)
#define MHI_SMMU_ATOMIC BIT(3)
#define MHI_SMMU_FORCE_COHERENT BIT(4)

#define MHI_PCIE_VENDOR_ID (0x17cb)
#define MHI_PCIE_DEBUG_ID (0xffff)

/* runtime suspend timer */
#define MHI_RPM_SUSPEND_TMR_MS (250)
#define MHI_PCI_BAR_NUM (0)

extern const char * const mhi_ee_str[MHI_EE_MAX];
#define TO_MHI_EXEC_STR(ee) (ee >= MHI_EE_MAX ? "INVALID_EE" : mhi_ee_str[ee])

struct mhi_dev {
	struct platform_device *pdev;
	struct pci_dev *pci_dev;
	u32 smmu_cfg;
	int resn;
	void *arch_info;
	bool powered_on;
	dma_addr_t iova_start;
	dma_addr_t iova_stop;
};

void mhi_deinit_pci_dev(struct mhi_controller *mhi_cntrl);
int mhi_pci_probe(struct pci_dev *pci_dev,
		  const struct pci_device_id *device_id);

static inline int mhi_arch_iommu_init(struct mhi_controller *mhi_cntrl)
{
	struct mhi_dev *mhi_dev = mhi_controller_get_devdata(mhi_cntrl);

	mhi_cntrl->dev = &mhi_dev->pci_dev->dev;

	return dma_set_mask_and_coherent(mhi_cntrl->dev, DMA_BIT_MASK(32));
}

static inline void mhi_arch_iommu_deinit(struct mhi_controller *mhi_cntrl)
{
}

#ifdef CONFIG_QTI_MHI_DEBUG
#define MHI_IPC_LOG_PAGES (100)
#define MHI_IPC_LOG_LVL MHI_MSG_LVL_VERBOSE
#else
#define MHI_IPC_LOG_LVL MHI_MSG_LVL_ERROR
#define MHI_IPC_LOG_PAGES (10)
#endif

static inline int mhi_arch_pcie_init(struct mhi_controller *mhi_cntrl)
{
	char node[32];

	snprintf(node, sizeof(node), "mhi_%04x_%02u.%02u.%02u",
		 mhi_cntrl->dev_id, mhi_cntrl->domain, mhi_cntrl->bus,
		 mhi_cntrl->slot);
	mhi_cntrl->log_buf = ipc_log_context_create(MHI_IPC_LOG_PAGES,
						    node, 0);
	mhi_cntrl->log_lvl = MHI_IPC_LOG_LVL;

	return 0;
}

static inline void mhi_arch_pcie_deinit(struct mhi_controller *mhi_cntrl)
{
}

static inline int mhi_arch_link_off(struct mhi_controller *mhi_cntrl,
				    bool graceful)
{
	return 0;
}

static inline int mhi_arch_link_on(struct mhi_controller *mhi_cntrl)
{
	return 0;
}

#endif /* _MHI_QCOM_ */
