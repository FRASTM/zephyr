/*
 * Copyright (c) 2020 Piotr Mienkowski
 * Copyright (c) 2020 Linaro Limited
 * Copyright (c) 2021 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_ospi_nor

#include <errno.h>
#include <kernel.h>
#include <toolchain.h>
#include <arch/common/ffs.h>
#include <sys/util.h>
#include <soc.h>
#include <drivers/pinctrl.h>
#include <drivers/clock_control/stm32_clock_control.h>
#include <drivers/clock_control.h>
#include <drivers/flash.h>
#include <drivers/dma.h>
#include <drivers/dma/dma_stm32.h>

#include <stm32_ll_dma.h>

#include "spi_nor.h"
#include "jesd216.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(flash_stm32_ospi, CONFIG_FLASH_LOG_LEVEL);

#define STM32_OSPI_FIFO_THRESHOLD         4
#define STM32_OSPI_CLOCK_PRESCALER_MAX  255

#define STM32_OSPI_USE_DMA DT_NODE_HAS_PROP(DT_PARENT(DT_DRV_INST(0)), dmas)

#if  DT_HAS_COMPAT_STATUS_OKAY(st_stm32_dma)
	LL_DMA_MDATAALIGN_BYTE,
	LL_DMA_MDATAALIGN_HALFWORD,
	LL_DMA_MDATAALIGN_WORD,
};

uint32_t table_p_size[] = {
	LL_DMA_PDATAALIGN_BYTE,
	LL_DMA_PDATAALIGN_HALFWORD,
	LL_DMA_PDATAALIGN_WORD,
};
#endif

typedef void (*irq_config_func_t)(const struct device *dev);


struct stream {
	DMA_TypeDef *reg;
	const struct device *dev;
	uint32_t channel;
	struct dma_config cfg;
};

struct flash_stm32_ospi_config {
	OCTOSPI_TypeDef *regs;
	struct stm32_pclken pclken;
	irq_config_func_t irq_config;
	size_t flash_size;
	uint32_t max_frequency;
	int data_mode;
	const struct pinctrl_dev_config *pcfg;
};

struct flash_stm32_ospi_data {
	OSPI_HandleTypeDef hospi;
	struct k_sem sem;
	struct k_sem sync;
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout layout;
#endif
	struct jesd216_erase_type erase_types[JESD216_NUM_ERASE_TYPES];
	/* Number of bytes per page */
	uint16_t page_size;
	int cmd_status;
	struct stream dma;
};

#define DEV_NAME(dev) ((dev)->name)
#define DEV_CFG(dev) \
	(const struct flash_stm32_ospi_config * const)(dev->config)
#define DEV_DATA(dev) \
	(struct flash_stm32_ospi_data * const)(dev->data)

static inline void ospi_lock_thread(const struct device *dev)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);

	k_sem_take(&dev_data->sem, K_FOREVER);
}

static inline void ospi_unlock_thread(const struct device *dev)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);

	k_sem_give(&dev_data->sem);
}

/*
 * Send a command over OSPI bus.
 */
static int ospi_send_cmd(const struct device *dev, OSPI_RegularCmdTypeDef *cmd)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	cmd->DataMode = dev_cfg->data_mode;

	LOG_DBG("Instruction 0x%x", cmd->Instruction);

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command_IT(&dev_data->hospi, cmd);
//	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}
	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

/*
 * Perform a read access over OSPI bus for SDFP (DataMode is already set)
 */
static int ospi_read_access_sdfp(const struct device *dev, OSPI_RegularCmdTypeDef *cmd,
			    uint8_t *data, size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	ARG_UNUSED(dev_cfg);

	cmd->NbData = size;

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd,
				HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
	}

	hal_ret = HAL_OSPI_Receive(&dev_data->hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
	}

	return dev_data->cmd_status;
}

/*
 * Perform a read access over OSPI bus.
 */
static int ospi_read_access(const struct device *dev, OSPI_RegularCmdTypeDef *cmd,
			    uint8_t *data, size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	cmd->DataMode = dev_cfg->data_mode;
	cmd->NbData = size;

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command_IT(&dev_data->hospi, cmd);
//	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}

#if STM32_OSPI_USE_DMA
	hal_ret = HAL_OSPI_Receive_DMA(&dev_data->hospi, data);
#else
	hal_ret = HAL_OSPI_Receive_IT(&dev_data->hospi, data);
#endif
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

/*
 * Perform a write access over OSPI bus.
 */
static int ospi_write_access(const struct device *dev, OSPI_RegularCmdTypeDef *cmd,
			     const uint8_t *data, size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	LOG_DBG("Instruction 0x%x", cmd->Instruction);

	cmd->NbData = size;

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command_IT(&dev_data->hospi, cmd);
//	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}

#if STM32_OSPI_USE_DMA
	hal_ret = HAL_OSPI_Transmit_DMA(&dev_data->hospi, (uint8_t *)data);
#else
	hal_ret = HAL_OSPI_Transmit_IT(&dev_data->hospi, (uint8_t *)data);
#endif
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}
	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}


/*
 * Read Serial Flash ID Parameter
 */
static int ospi_read_norid(const struct device *dev, uint8_t *data,
			  size_t size)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	OSPI_RegularCmdTypeDef cmd = {
		.Instruction = 0x9F,
		.DummyCycles = 8,
		.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE,
		.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS,
		.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId            = HAL_OSPI_FLASH_ID_1,
		.DataMode           = HAL_OSPI_DATA_1_LINE,
		.AddressMode        = HAL_OSPI_ADDRESS_NONE,
		.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD,
		.NbData = size,
	};
	dev_data->cmd_status = 0;

	/* do not use command IT here */
	hal_ret = HAL_OSPI_Command(&dev_data->hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}
	/* do not use receive IT here */
	hal_ret = HAL_OSPI_Receive(&dev_data->hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read SFDP", hal_ret);
		return -EIO;
	}

	return dev_data->cmd_status;
}

/*
 * Read Serial Flash Discovery Parameter
 */
static int ospi_read_sfdp(const struct device *dev, off_t addr, uint8_t *data,
			  size_t size)
{
	OSPI_RegularCmdTypeDef cmd = {
		.Instruction = JESD216_CMD_READ_SFDP,
		.Address = addr,
		.AddressSize = HAL_OSPI_ADDRESS_24_BITS,
		.AddressMode = HAL_OSPI_ADDRESS_1_LINE,
		.DummyCycles = 8,
		.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
		.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
		.DataMode = HAL_OSPI_DATA_1_LINE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	return ospi_read_access_sdfp(dev, &cmd, data, size);
}

static bool ospi_address_is_valid(const struct device *dev, off_t addr,
				  size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	size_t flash_size = dev_cfg->flash_size;

	return (addr >= 0) && ((uint64_t)addr + (uint64_t)size <= flash_size);
}

/* This function Configures Automatic-polling mode to wait for the Mem ready */
static int flash_stm32_ospi_auto_polling(OSPI_HandleTypeDef *hospi)
{
	OSPI_RegularCmdTypeDef sCommand = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.Instruction = SPI_NOR_OCMD_RDSR,
		.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES,
		.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS,
		.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
		.AddressMode = HAL_OSPI_ADDRESS_8_LINES,
		.AddressSize = HAL_OSPI_ADDRESS_32_BITS,
		.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
		.Address = 0,
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DummyCycles = 4,
		.DQSMode = HAL_OSPI_DQS_DISABLE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
		.DataMode = HAL_OSPI_DATA_8_LINES,
		.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
		.NbData = 1,
	};

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			LOG_ERR("OSPI Write Enable command failed");
			return -EIO;
	}

	/* Set the mask to  0x01 to mask all Status REG bits except WIP */
	/* Set the match to 0x00 to check if the WIP bit is Reset */
	OSPI_AutoPollingTypeDef sConfig = {
		.Match = SPI_NOR_MEM_RDY_MATCH,
		.Mask = SPI_NOR_MEM_RDY_MASK,
		.MatchMode = HAL_OSPI_MATCH_MODE_AND,
		.Interval = SPI_NOR_AUTO_POLLING_INTERVAL,
		.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
	};
	/* Start Automatic-Polling mode to wait until the memory is ready WIP=0 */
	if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
			LOG_ERR("OSPI Write Enable command failed");
			return -EIO;
	}

	return 0;
}

/* Enables writing to the memory: write enable cmd is sent in octal SPI mode */
static int flash_stm32_ospi_write_enable(OSPI_HandleTypeDef *hospi)
{
	OSPI_RegularCmdTypeDef sCommand;
	OSPI_AutoPollingTypeDef sConfig;
	/* Initialize the Write Enable cmd in octo SPI mode */
	sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
	sCommand.Instruction = SPI_NOR_OCMD_WREN;
	sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
	sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS;
	sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE;
	sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = HAL_OSPI_DATA_NONE;
	sCommand.DummyCycles = 0;
	sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
	/* Send Write Enable command in octo SPI mode */
	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI Write Enable command failed");
		return -EIO;
	}

	/* Initialize Automatic-Polling mode to wait until WEL=1 */
	sCommand.Instruction = SPI_NOR_OCMD_RDSR;
	sCommand.Address = 0;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
	sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
	sCommand.DataMode = HAL_OSPI_DATA_8_LINES;
	sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	sCommand.NbData = 1;
	sCommand.DummyCycles = 4;
	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI Automatic-Polling failed");
		return -EIO;
	}
	/* Set the mask to 0x02 to mask all Status REG bits except WEL */
	/* Set the match to 0x02 to check if the WEL bit is set */
	sConfig.Match = SPI_NOR_WREN_MATCH;
	sConfig.Mask = SPI_NOR_WREN_MASK;
	sConfig.MatchMode = HAL_OSPI_MATCH_MODE_AND;
	sConfig.Interval = SPI_NOR_AUTO_POLLING_INTERVAL;
	sConfig.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

	/* Start Automatic-Polling mode to wait until WEL=1 */
	if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI Automatic-Polling failed");
		return -EIO;
	}
	return 0;
}

static int flash_stm32_ospi_erase(const struct device *dev, off_t addr,
				  size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	int ret = 0;

	if (!ospi_address_is_valid(dev, addr, size)) {
		LOG_DBG("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	/* erase non-zero size */
	if (size == 0) {
		return 0;
	}

	if (size != SPI_NOR_SECTOR_SIZE) {
		LOG_DBG("Error: wrong sector size 0x%x", size);
		return -ENOTSUP;
	}

	OSPI_RegularCmdTypeDef cmd_erase = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES,
		.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS,
		.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
		.DataMode = HAL_OSPI_DATA_NONE,
		.DummyCycles = 0,
		.DQSMode = HAL_OSPI_DQS_DISABLE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	ospi_lock_thread(dev);

	ret = flash_stm32_ospi_write_enable(&dev_data->hospi);

	while ((size > 0) && (ret == 0)) {
		if (size == dev_cfg->flash_size) {
			/* chip erase */
			cmd_erase.Instruction = SPI_NOR_OCMD_CE;
			cmd_erase.AddressMode = HAL_OSPI_ADDRESS_NONE;
			/* full chip erase command */
			ospi_send_cmd(dev, &cmd_erase);
			size -= dev_cfg->flash_size;
		} else {
			/* sector erase */
			cmd_erase.Address = addr;

			const struct jesd216_erase_type *erase_types =
							dev_data->erase_types;
			const struct jesd216_erase_type *bet = NULL;

			for (uint8_t ei = 0;
				ei < JESD216_NUM_ERASE_TYPES; ++ei) {
				const struct jesd216_erase_type *etp =
							&erase_types[ei];

				if ((etp->exp != 0)
				    && SPI_NOR_IS_ALIGNED(addr, etp->exp)
				    && SPI_NOR_IS_ALIGNED(size, etp->exp)
				    && ((bet == NULL)
					|| (etp->exp > bet->exp))) {
					bet = etp;
					cmd_erase.Instruction = bet->cmd;
				} else {
					/* use the default sector erase cmd */
					cmd_erase.Instruction = SPI_NOR_OCMD_SE;
					cmd_erase.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
					cmd_erase.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
					cmd_erase.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
					cmd_erase.Address = addr;
				}
			}
			ospi_send_cmd(dev, &cmd_erase);
			if (bet != NULL) {
				addr += BIT(bet->exp);
				size -= BIT(bet->exp);
			} else {
				addr += SPI_NOR_SECTOR_SIZE;
				size -= SPI_NOR_SECTOR_SIZE;
				/*
				LOG_ERR("Can't erase %zu at 0x%lx",
					size, (long)addr);
				ret = -EINVAL;*/
			}
		}
		ret = flash_stm32_ospi_auto_polling(&dev_data->hospi);
	}

	ospi_unlock_thread(dev);

	return ret;
}

static int flash_stm32_ospi_read(const struct device *dev, off_t addr,
				 void *data, size_t size)
{
	int ret;

	if (!ospi_address_is_valid(dev, addr, size)) {
		LOG_DBG("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	/* read non-zero size */
	if (size == 0) {
		return 0;
	}

	OSPI_RegularCmdTypeDef cmd = {
		.Instruction = SPI_NOR_CMD_READ,
		.Address = addr,
		.AddressSize = HAL_OSPI_ADDRESS_24_BITS,
		.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
		.AddressMode = HAL_OSPI_ADDRESS_1_LINE,
	};

	ospi_lock_thread(dev);

	ret = ospi_read_access(dev, &cmd, data, size);

	ospi_unlock_thread(dev);

	return ret;
}

static int flash_stm32_ospi_write(const struct device *dev, off_t addr,
				  const void *data, size_t size)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	int ret = 0;

	if (!ospi_address_is_valid(dev, addr, size)) {
		LOG_DBG("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	/* write non-zero size */
	if (size == 0) {
		return 0;
	}

	OSPI_RegularCmdTypeDef cmd_pp = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.Instruction = SPI_NOR_OCMD_PAGE_PRG,
		.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS,
		.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES,
		.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
		.AddressMode = HAL_OSPI_ADDRESS_8_LINES,
		.AddressSize = HAL_OSPI_ADDRESS_32_BITS,
		.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
		.DataMode = HAL_OSPI_DATA_8_LINES,
		.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
		.DummyCycles = 0,
		.DQSMode = HAL_OSPI_DQS_ENABLE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	ospi_lock_thread(dev);

	flash_stm32_ospi_write_enable(&dev_data->hospi);

	while ((size > 0) && (ret == 0)) {
		size_t to_write = size;

		/* Don't write more than a page. */
		if (to_write >= SPI_NOR_PAGE_SIZE) {
			to_write = SPI_NOR_PAGE_SIZE;
		}

		/* Don't write across a page boundary */
		if (((addr + to_write - 1U) / SPI_NOR_PAGE_SIZE)
		    != (addr / SPI_NOR_PAGE_SIZE)) {
			to_write = SPI_NOR_PAGE_SIZE -
						(addr % SPI_NOR_PAGE_SIZE);
		}

		cmd_pp.Address = addr;
		ret = ospi_write_access(dev, &cmd_pp, data, to_write);
		if (ret != 0) {
			break;
		}

		size -= to_write;
		data = (const uint8_t *)data + to_write;
		addr += to_write;

		ret = flash_stm32_ospi_auto_polling(&dev_data->hospi);
		if (ret != 0) {
			break;
		}
	}

	ospi_unlock_thread(dev);

	return ret;
}

static int ospi_wait_until_ready(const struct device *dev)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	uint8_t reg;
	int ret;

	OSPI_RegularCmdTypeDef cmd = {
		.Instruction = SPI_NOR_CMD_RDSR,
		.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
		.DataMode = dev_cfg->data_mode,
	};

	do {
		ret = ospi_read_access(dev, &cmd, &reg, sizeof(reg));
	} while (!ret && (reg & SPI_NOR_WIP_BIT));

	return ret;
}

#if defined (CONFIG_FLASH_NOR_DTR)
/* Configures the MX25LM51245G memory during the init in Double Data Rate */
static int flash_stm32_ospi_DTR_mode_config(OSPI_HandleTypeDef *hospi)
{
	OSPI_RegularCmdTypeDef sCommand;
	uint8_t tmp;

	/* Enable writing to memory in order to set Dummy */
	flash_stm32_ospi_write_enable(hospi);
	/* Initialize Indirect write mode to configure Dummy */
	sCommand.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
	sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
//	sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	sCommand.Instruction = SPI_NOR_CMD_CFGREG2;
	sCommand.Address = SPI_NOR_REG2_ADDR3;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
//	sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
	sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	sCommand.DataMode = HAL_OSPI_DATA_1_LINE;
//	sCommand.DataDtrMode= HAL_OSPI_DATA_DTR_DISABLE;
	sCommand.NbData = 1;
	sCommand.DummyCycles = 0;
	sCommand.DQSMode = HAL_OSPI_DQS_DISABLE;
	sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI write op failed");
		return -EIO;
	}
	/* Write Configuration register 2 with new dummy cycles */
	tmp = SPI_NOR_CR2_DUMMY_CYCLES_66MHZ;
	if (HAL_OSPI_Transmit(hospi, &tmp, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI write dummy failed");
		return -EIO;
	}
	if (flash_stm32_ospi_autoPolling_WIP(hospi) != 0) {
		LOG_ERR("OSPI write dummy failed");
		return -EIO;
	}
	/* Enable writing to memory in order to set Octal DTR mode */
	if (flash_stm32_ospi_write_enable(hospi) != 0) {
		LOG_ERR("OSPI write dummy failed");
		return -EIO;
	}
	/* Initialize OCTOSPI to Indirect write mode to configure Octal mode */
	sCommand.Instruction = SPI_NOR_CMD_CFGREG2;
	sCommand.Address = SPI_NOR_REG2_ADDR1;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI write dummy failed");
		return -EIO;
	}
	/* Write Configuration register 2 with with Octal mode */
	tmp = SPI_NOR_CR2_DTR_OPI_EN;
	if (HAL_OSPI_Transmit(hospi, &tmp, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI Configuration register 2 failed");
		return -EIO;
	}
	return 0;
}
# endif /* CONFIG_FLASH_NOR_DTR */

/* Configures the MX25LM51245G memory during the init */
static int flash_stm32_ospi_mode_config(OSPI_HandleTypeDef *hospi)
{
	/* Enable write operations in SPI node */
	OSPI_RegularCmdTypeDef sCommand = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.Instruction = SPI_NOR_CMD_WREN,
		.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
		.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
		.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
		.AddressMode = HAL_OSPI_ADDRESS_NONE,
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DataMode = HAL_OSPI_DATA_NONE,
		.DummyCycles = 0,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
		.DQSMode = HAL_OSPI_DQS_DISABLE,
	};

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI write op failed");
		return -EIO;
	}

	/* Configure automatic polling mode to wait for write enabling */
	sCommand.Instruction = SPI_NOR_CMD_RDSR;
	sCommand.DataMode = HAL_OSPI_DATA_1_LINE;
	sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	sCommand.NbData = 1;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI write op failed");
		return -EIO;
	}

	OSPI_AutoPollingTypeDef sConfig = {
		.Match         = SPI_NOR_WREN_MATCH,
		.Mask          = SPI_NOR_WREN_MASK,
		.MatchMode     = HAL_OSPI_MATCH_MODE_AND,
		.Interval      = SPI_NOR_AUTO_POLLING_INTERVAL,
		.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE,
	};

	if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI autoPolling failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with new dummy cycles) */
	sCommand.Instruction = SPI_NOR_CMD_CFGREG2;
	sCommand.Address = SPI_NOR_REG2_ADDR3;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
	sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI write CFGREG2 failed");
		return -EIO;
	}

	/* Write config with new dummy cycle */
	uint32_t tmp = SPI_NOR_CR2_DUMMY_CYCLES_66MHZ;
	if (HAL_OSPI_Transmit(hospi, &tmp, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI Configuration register 2 failed");
		return -EIO;
	}

	/* Wait that the memory is ready */
	sCommand.Instruction = SPI_NOR_CMD_RDSR;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_NONE;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI memory ready failed");
		return -EIO;
	}

	sConfig.Match = SPI_NOR_MEM_RDY_MATCH;
	sConfig.Mask  = SPI_NOR_MEM_RDY_MASK;

	if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI Memory not ready");
		return -EIO;
	}

	/* Enable write operations to set octal mode */
	sCommand.Instruction = SPI_NOR_CMD_WREN;
	sCommand.DataMode    = HAL_OSPI_DATA_NONE;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI write op failed");
		return -EIO;
	}

	/* Configure automatic polling mode to wait for write enabling */
	sCommand.Instruction = SPI_NOR_CMD_RDSR;
	sCommand.DataMode    = HAL_OSPI_DATA_1_LINE;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI automatic polling failed");
		return -EIO;
	}

	sConfig.Match = SPI_NOR_WREN_MATCH;
	sConfig.Mask  = SPI_NOR_WREN_MASK;

	if (HAL_OSPI_AutoPolling(hospi, &sConfig, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI write En failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with octal mode) */
	sCommand.Instruction = SPI_NOR_CMD_CFGREG2;
	sCommand.Address     = SPI_NOR_REG2_ADDR1;
	sCommand.AddressMode = HAL_OSPI_ADDRESS_1_LINE;

	if (HAL_OSPI_Command(hospi, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI write Reg 2 failed");
		return -EIO;
	}

	tmp = SPI_NOR_CR2_STR_OPI_EN; /* single Data Rate */

	if (HAL_OSPI_Transmit(hospi, &tmp, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		LOG_ERR("OSPI DTR enable failed");
		return -EIO;
	}

	/* Wait that the configuration is effective */
	HAL_Delay(100);
	/* check that memory is ready */
	if (flash_stm32_ospi_auto_polling(hospi) != 0) {
		LOG_ERR("OSPI auto polling not ready");
		return -EIO;
	}

	return 0;
}

static const struct flash_parameters flash_stm32_ospi_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff
};

static const struct flash_parameters *
flash_stm32_ospi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_stm32_ospi_parameters;
}

static void flash_stm32_ospi_isr(const struct device *dev)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);

	HAL_OSPI_IRQHandler(&dev_data->hospi);
}

/* This function is executed in the interrupt context */
#if STM32_OSPI_USE_DMA
static void ospi_dma_callback(const struct device *dev, void *arg,
			 uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	if (status != 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);

	}

	HAL_DMA_IRQHandler(hdma);
}
#endif

__weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}

/*
 * Transfer Error callback.
 */
void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	LOG_DBG("Enter");

	dev_data->cmd_status = -EIO;

	k_sem_give(&dev_data->sync);
}

/*
 * Command completed callback.
 */
void HAL_OSPI_CmdCpltCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	k_sem_give(&dev_data->sync);
}

/*
 * Rx Transfer completed callback.
 */
void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	k_sem_give(&dev_data->sync);
}

/*
 * Tx Transfer completed callback.
 */
void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	k_sem_give(&dev_data->sync);
}

/*
 * Status Match callback.
 */
void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	k_sem_give(&dev_data->sync);
}

/*
 * Timeout callback.
 */
void HAL_OSPI_TimeOutCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	LOG_DBG("Enter");

	dev_data->cmd_status = -EIO;

	k_sem_give(&dev_data->sync);
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_stm32_ospi_pages_layout(const struct device *dev,
				const struct flash_pages_layout **layout,
				size_t *layout_size)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);

	*layout = &dev_data->layout;
	*layout_size = 1;
}
#endif

static const struct flash_driver_api flash_stm32_ospi_driver_api = {
	.read = flash_stm32_ospi_read,
	.write = flash_stm32_ospi_write,
	.erase = flash_stm32_ospi_erase,
	.get_parameters = flash_stm32_ospi_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_stm32_ospi_pages_layout,
#endif
};

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static int setup_pages_layout(const struct device *dev)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *data = DEV_DATA(dev);
	const size_t flash_size = dev_cfg->flash_size;
	uint32_t layout_page_size = data->page_size;
	uint8_t value = 0;
	int rv = 0;

	/* Find the smallest erase size. */
	for (size_t i = 0; i < ARRAY_SIZE(data->erase_types); ++i) {
		const struct jesd216_erase_type *etp = &data->erase_types[i];

		if ((etp->cmd != 0)
		    && ((value == 0) || (etp->exp < value))) {
			value = etp->exp;
		}
	}

	uint32_t erase_size;

	if (value == 0) {
		erase_size = BIT(value);
		/*return -ENOTSUP;*/
	} else {
		erase_size = BIT(value);
	}

	/* We need layout page size to be compatible with erase size */
	if ((layout_page_size % erase_size) != 0) {
		LOG_DBG("layout page %u not compatible with erase size %u",
			layout_page_size, erase_size);
		LOG_DBG("erase size will be used as layout page size");
		layout_page_size = erase_size;
	}

	/* Warn but accept layout page sizes that leave inaccessible
	 * space.
	 */
	if ((flash_size % layout_page_size) != 0) {
		LOG_INF("layout page %u wastes space with device size %zu",
			layout_page_size, flash_size);
	}

	data->layout.pages_size = layout_page_size;
	data->layout.pages_count = flash_size / layout_page_size;
	LOG_DBG("layout %u x %u By pages", data->layout.pages_count,
					   data->layout.pages_size);

	return rv;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int spi_nor_process_bfp(const struct device *dev,
			       const struct jesd216_param_header *php,
			       const struct jesd216_bfp *bfp)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *data = DEV_DATA(dev);
	struct jesd216_erase_type *etp = data->erase_types;
	const size_t flash_size = jesd216_bfp_density(bfp) / 8U;

	if (flash_size != dev_cfg->flash_size) {
		LOG_ERR("Unexpected flash size: %u", flash_size);
	}

	LOG_INF("%s: %u MiBy flash", dev->name, (uint32_t)(flash_size >> 20));

	/* Copy over the erase types, preserving their order.  (The
	 * Sector Map Parameter table references them by index.)
	 */
	memset(data->erase_types, 0, sizeof(data->erase_types));
	for (uint8_t ti = 1; ti <= ARRAY_SIZE(data->erase_types); ++ti) {
		if (jesd216_bfp_erase(bfp, ti, etp) == 0) {
			LOG_DBG("Erase %u with %02x",
					(uint32_t)BIT(etp->exp), etp->cmd);
		}
		++etp;
	}

	data->page_size = jesd216_bfp_page_size(php, bfp);

	LOG_DBG("Page size %u bytes", data->page_size);
	LOG_DBG("Flash size %u bytes", flash_size);
	return 0;
}

static int flash_stm32_ospi_init(const struct device *dev)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	uint32_t ahb_clock_freq;
	uint32_t prescaler = 0;
	int ret;

	/* Signals configuration */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("OSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Initializes the independent peripherals clock */
	__HAL_RCC_OSPI_CONFIG(RCC_OSPICLKSOURCE_SYSCLK); /* */

#if STM32_OSPI_USE_DMA
	/*
	 * DMA configuration
	 * Due to use of OSPI HAL API in current driver,
	 * both HAL and Zephyr DMA drivers should be configured.
	 * The required configuration for Zephyr DMA driver should only provide
	 * the minimum information to inform the DMA slot will be in used and
	 * how to route callbacks.
	 */
	struct dma_config dma_cfg = dev_data->dma.cfg;
	static DMA_HandleTypeDef hdma;

	if (!device_is_ready(dev_data->dma.dev)) {
		LOG_ERR("%s device not ready", dev_data->dma.dev->name);
		return -ENODEV;
	}

	/* Proceed to the minimum Zephyr DMA driver init */
	dma_cfg.user_data = &hdma;
	/* HACK: This field is used to inform driver that it is overridden */
	dma_cfg.linked_channel = STM32_DMA_HAL_OVERRIDE;
	ret = dma_config(dev_data->dma.dev, dev_data->dma.channel, &dma_cfg);
	if (ret != 0) {
		return ret;
	}

	/* Proceed to the HAL DMA driver init */
	if (dma_cfg.source_data_size != dma_cfg.dest_data_size) {
		LOG_ERR("Source and destination data sizes not aligned");
		return -EINVAL;
	}

	int index = find_lsb_set(dma_cfg.source_data_size) - 1;

	hdma.Init.PeriphDataAlignment = table_p_size[index];
	hdma.Init.MemDataAlignment = table_m_size[index];
	hdma.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma.Init.MemInc = DMA_MINC_ENABLE;
	hdma.Init.Mode = DMA_NORMAL;
	hdma.Init.Priority = dma_cfg.channel_priority;
#ifdef CONFIG_DMA_STM32_V1
	/* TODO: Not tested in this configuration */
	hdma.Init.Channel = dma_cfg.dma_slot;
	hdma.Instance = __LL_DMA_GET_STREAM_INSTANCE(dev_data->dma.reg,
						     dev_data->dma.channel);
#else
	hdma.Init.Request = dma_cfg.dma_slot;
#ifdef CONFIG_DMAMUX_STM32
	/* HAL expects a valid DMA channel (not DAMMUX) */
	/* TODO: Get DMA instance from DT */
	hdma.Instance = __LL_DMA_GET_CHANNEL_INSTANCE(DMA1,
						      dev_data->dma.channel+1);
#else
	hdma.Instance = __LL_DMA_GET_CHANNEL_INSTANCE(dev_data->dma.reg,
						      dev_data->dma.channel-1);
#endif
#endif /* CONFIG_DMA_STM32_V1 */

	/* Initialize DMA HAL */
	__HAL_LINKDMA(&dev_data->hospi, hdma, hdma);
	HAL_DMA_Init(&hdma);

#endif /* STM32_OSPI_USE_DMA */

	/* Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dev_cfg->pclken) != 0) {
		LOG_DBG("Could not enable OSPI clock");
		return -EIO;
	}

	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			(clock_control_subsys_t) &dev_cfg->pclken,
			&ahb_clock_freq) < 0) {
		LOG_DBG("Failed to get AHB clock frequency");
		return -EIO;
	}

	for (; prescaler <= STM32_OSPI_CLOCK_PRESCALER_MAX; prescaler++) {
		uint32_t clk = ahb_clock_freq / (prescaler + 1);

		if (clk <= dev_cfg->max_frequency) {
			break;
		}
	}
	__ASSERT_NO_MSG(prescaler <= STM32_OSPI_CLOCK_PRESCALER_MAX);

	/* Initialize OSPI HAL structure completely */
	dev_data->hospi.Init.FifoThreshold = 4;
	dev_data->hospi.Init.ClockPrescaler = prescaler;
	dev_data->hospi.Init.DeviceSize = 26;
//	dev_data->hospi.Init.DeviceSize = find_lsb_set(dev_cfg->flash_size);
	dev_data->hospi.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	dev_data->hospi.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
	dev_data->hospi.Init.ChipSelectHighTime = 2;
	dev_data->hospi.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	dev_data->hospi.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	dev_data->hospi.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
	dev_data->hospi.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	dev_data->hospi.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	dev_data->hospi.Init.ChipSelectBoundary = 0;
	dev_data->hospi.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
	dev_data->hospi.Init.Refresh = 0;
	if (HAL_OSPI_Init(&dev_data->hospi) != HAL_OK) {
		LOG_ERR("OSPI Init failed");
		return -EIO;
	}

#if defined(CONFIG_SOC_SERIES_STM32U5X)
	/* OCTOSPI I/O manager init Function */
	OSPIM_CfgTypeDef ospi_mgr_cfg = {0};

	__HAL_RCC_OSPIM_CLK_ENABLE();
	if (dev_data->hospi.Instance == OCTOSPI1) {
		ospi_mgr_cfg.ClkPort = 1;
		ospi_mgr_cfg.DQSPort = 1;
		ospi_mgr_cfg.NCSPort = 1;
		ospi_mgr_cfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
		ospi_mgr_cfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;

	} else if (dev_data->hospi.Instance == OCTOSPI2) {
		ospi_mgr_cfg.ClkPort = 2;
		ospi_mgr_cfg.DQSPort = 2;
		ospi_mgr_cfg.NCSPort = 2;
		ospi_mgr_cfg.IOLowPort = HAL_OSPIM_IOPORT_2_LOW;
		ospi_mgr_cfg.IOHighPort = HAL_OSPIM_IOPORT_2_HIGH;

	}
	ospi_mgr_cfg.Req2AckTime = 1; /* arbitrary value */
	if (HAL_OSPIM_Config(&dev_data->hospi, &ospi_mgr_cfg,
		HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI M config failed");
		return -EIO;
	}
printk("OSPI M init'd\n");
	/* OCTOSPI2 delay block init Function */
	HAL_OSPI_DLYB_CfgTypeDef ospi_delay_block_cfg = {0};

	ospi_delay_block_cfg.Units = 56;
	ospi_delay_block_cfg.PhaseSel = 2;
	if (HAL_OSPI_DLYB_SetConfig(&dev_data->hospi, &ospi_delay_block_cfg) != HAL_OK) {
		LOG_ERR("OSPI DelayBlock failed");
		return -EIO;
	}
printk("OSPI delay block init'd\n");
#endif /* CONFIG_SOC_SERIES_STM32U5X */

	/* send the instruction to read the NOR-flash ID : check access to mem. */
	uint8_t nor_id[3];

	ret = ospi_read_norid(dev, nor_id, sizeof(nor_id));
	if (ret != 0) {
		LOG_ERR("reading Nor-flash ID failed: %d", ret);
		return ret;
	}

	/* Initialize semaphores */
	k_sem_init(&dev_data->sem, 1, 1);
	k_sem_init(&dev_data->sync, 0, 1);

	/* Run IRQ init */
	dev_cfg->irq_config(dev);

	/* send the instruction to read the SFDP  */
	const uint8_t decl_nph = 2;
	union {
		/* We only process BFP so use one parameter block */
		uint8_t raw[JESD216_SFDP_SIZE(decl_nph)];
		struct jesd216_sfdp_header sfdp;
	} u;
	const struct jesd216_sfdp_header *hp = &u.sfdp;

	ret = ospi_read_sfdp(dev, 0, u.raw, sizeof(u.raw));
	if (ret != 0) {
		LOG_ERR("SFDP read failed: %d", ret);
		return ret;
	}

	uint32_t magic = jesd216_sfdp_magic(hp);

	if (magic != JESD216_SFDP_MAGIC) {
		/* this is not and error, continue */
		LOG_INF("SFDP magic not supported");
		dev_data->page_size = 0x1000; /* 4K erase page size */
		goto run_nor_init;
	}

	LOG_INF("%s: SFDP v %u.%u AP %x with %u PH", dev->name,
		hp->rev_major, hp->rev_minor, hp->access, 1 + hp->nph);

	const struct jesd216_param_header *php = hp->phdr;
	const struct jesd216_param_header *phpe = php +
						     MIN(decl_nph, 1 + hp->nph);

	while (php != phpe) {
		uint16_t id = jesd216_param_id(php);

		LOG_INF("PH%u: %04x rev %u.%u: %u DW @ %x",
			(php - hp->phdr), id, php->rev_major, php->rev_minor,
			php->len_dw, jesd216_param_addr(php));

		if (id == JESD216_SFDP_PARAM_ID_BFP) {
			union {
				uint32_t dw[MIN(php->len_dw, 20)];
				struct jesd216_bfp bfp;
			} u;
			const struct jesd216_bfp *bfp = &u.bfp;

			ret = ospi_read_sfdp(dev, jesd216_param_addr(php),
					     (uint8_t *)u.dw, sizeof(u.dw));
			if (ret == 0) {
				ret = spi_nor_process_bfp(dev, php, bfp);
			}

			if (ret != 0) {
				LOG_ERR("SFDP BFP failed: %d", ret);
				printk("SFDP BFP failed\n");
				break;
			}
		}
		++php;
	}

run_nor_init:

	/* Configure the memory in octal mode */
	if (flash_stm32_ospi_mode_config(&dev_data->hospi) != 0) {
		LOG_ERR("OSPI octal mode cfg failed");
		return -EIO;
	}


#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	ret = setup_pages_layout(dev);
	if (ret != 0) {
		LOG_ERR("layout setup failed: %d", ret);
		return -ENODEV;
	}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}

#if STM32_OSPI_USE_DMA
#define DMA_CHANNEL_CONFIG(node, dir)					\
		DT_DMAS_CELL_BY_NAME(node, dir, channel_config)

#define OSPI_DMA_CHANNEL_INIT(node, dir)				\
	.dev = DEVICE_DT_GET(DT_DMAS_CTLR(node)),			\
	.channel = DT_DMAS_CELL_BY_NAME(node, dir, channel),		\
	.reg = (DMA_TypeDef *)DT_REG_ADDR(				\
				   DT_PHANDLE_BY_NAME(node, dmas, dir)),\
	.cfg = {							\
		.dma_slot = DT_DMAS_CELL_BY_NAME(node, dir, slot),	\
		.source_data_size = STM32_DMA_CONFIG_PERIPHERAL_DATA_SIZE( \
					DMA_CHANNEL_CONFIG(node, dir)), \
		.dest_data_size = STM32_DMA_CONFIG_MEMORY_DATA_SIZE(    \
					DMA_CHANNEL_CONFIG(node, dir)), \
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(		\
					DMA_CHANNEL_CONFIG(node, dir)), \
		.dma_callback = ospi_dma_callback,			\
	},								\

#define OSPI_DMA_CHANNEL(node, dir)					\
	.dma = {							\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, dir),		\
			(OSPI_DMA_CHANNEL_INIT(node, dir)),		\
			(NULL))						\
		},

#else
#define OSPI_DMA_CHANNEL(node, dir)
#endif /* CONFIG_USE_STM32_HAL_DMA */
#define OSPI_FLASH_MODULE(drv_id, flash_id)					\
		(DT_DRV_INST(drv_id), ospi_nor_flash_##flash_id)

static void flash_stm32_ospi_irq_config_func(const struct device *dev);

#define STM32_OSPI_NODE DT_INST_PARENT(0)

PINCTRL_DT_DEFINE(STM32_OSPI_NODE);

static const struct flash_stm32_ospi_config flash_stm32_ospi_cfg = {
	.regs = (OCTOSPI_TypeDef *)DT_REG_ADDR(STM32_OSPI_NODE),
	.pclken = {
		.enr = DT_CLOCKS_CELL(STM32_OSPI_NODE, bits),
		.bus = DT_CLOCKS_CELL(STM32_OSPI_NODE, bus)
	},
	.irq_config = flash_stm32_ospi_irq_config_func,
	.flash_size = DT_INST_PROP(0, size) / 8U,
	.max_frequency = DT_INST_PROP(0, ospi_max_frequency),
	.data_mode = DT_INST_PROP(0, data_mode),
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(STM32_OSPI_NODE),
};

static struct flash_stm32_ospi_data flash_stm32_ospi_dev_data = {
	.hospi = {
		.Instance = (OCTOSPI_TypeDef *)DT_REG_ADDR(STM32_OSPI_NODE),
		.Init = {
			.FifoThreshold = STM32_OSPI_FIFO_THRESHOLD,
			.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE,
			.ChipSelectHighTime = 1,
			.ClockMode = HAL_OSPI_CLOCK_MODE_0,
			},
	},
	OSPI_DMA_CHANNEL(STM32_OSPI_NODE, tx_rx)
};

DEVICE_DT_INST_DEFINE(0, &flash_stm32_ospi_init, NULL,
		      &flash_stm32_ospi_dev_data, &flash_stm32_ospi_cfg,
		      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		      &flash_stm32_ospi_driver_api);

static void flash_stm32_ospi_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_IRQN(STM32_OSPI_NODE), DT_IRQ(STM32_OSPI_NODE, priority),
		    flash_stm32_ospi_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_IRQN(STM32_OSPI_NODE));
}
