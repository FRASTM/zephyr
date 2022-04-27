/*
 * Copyright (c) 2022 STMicroelectronics
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
#include <dt-bindings/flash_controller/stm32_ospi.h>

#include "spi_nor.h"
#include "jesd216.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(flash_stm32_ospi, CONFIG_FLASH_LOG_LEVEL);

#define STM32_OSPI_FIFO_THRESHOLD         4
#define STM32_OSPI_CLOCK_PRESCALER_MAX  255

/* Max Time value during reset or erase operation */
#define STM32_OSPI_RESET_MAX_TIME               100U
#define STM32_OSPI_BULK_ERASE_MAX_TIME          460000U
#define STM32_OSPI_SECTOR_ERASE_MAX_TIME        1000U
#define STM32_OSPI_SUBSECTOR_4K_ERASE_MAX_TIME  400U
#define STM32_OSPI_WRITE_REG_MAX_TIME           40U

typedef void (*irq_config_func_t)(const struct device *dev);

struct flash_stm32_ospi_config {
	OCTOSPI_TypeDef *regs;
	struct stm32_pclken pclken;
	irq_config_func_t irq_config;
	size_t flash_size;
	uint32_t max_frequency;
	int data_mode; /* SPI or QSPI or OSPI */
	int data_rate; /* DTR or STR */
	const struct pinctrl_dev_config *pcfg;
#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_ospi_nor), sfdp_bfp)
	uint8_t sfdp_bfp[DT_INST_PROP_LEN(0, sfdp_bfp)];
#endif /* sfdp_bfp */
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
#if  DT_HAS_COMPAT_STATUS_OKAY(st_stm32_dma)
	struct stream dma;
#endif
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

	LOG_DBG("Instruction 0x%x", cmd->Instruction);

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}
	LOG_DBG("CCR 0x%x", dev_cfg->regs->CCR);

	return dev_data->cmd_status;
}

/*
 * Perform a read access over OSPI bus.
 */
static int ospi_read_access(const struct device *dev, OSPI_RegularCmdTypeDef *cmd,
			    uint8_t *data, size_t size)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	cmd->NbData = size;

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}

	hal_ret = HAL_OSPI_Receive_IT(&dev_data->hospi, data);

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
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	HAL_StatusTypeDef hal_ret;

	LOG_DBG("Instruction 0x%x", cmd->Instruction);

	cmd->NbData = size;

	dev_data->cmd_status = 0;

	hal_ret = HAL_OSPI_Command(&dev_data->hospi, cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}

	hal_ret = HAL_OSPI_Transmit_IT(&dev_data->hospi, (uint8_t *)data);

	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}

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
		LOG_ERR("%d: Failed to read NOR ID", hal_ret);
		return -EIO;
	}

	return dev_data->cmd_status;
}


/*
 * Read Serial Flash Discovery Parameter :
 * perform a read access over SPI bus for SDFP (DataMode is already set)
 * or get it from the sdfp table
 */
static int ospi_read_sfdp(const struct device *dev, off_t addr, uint8_t *data,
			  size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);

#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_ospi_nor), sfdp_bfp)
	/* simulate the SDFP */
	ARG_UNUSED(addr); /* addr is 0 */

	for (uint8_t i_ind = 0; i_ind < ARRAY_SIZE(dev_cfg->sfdp_bfp); i_ind++) {
		*(data + i_ind) = dev_cfg->sfdp_bfp[i_ind];
	}
#else /* sfdp_bfp */

	OSPI_RegularCmdTypeDef cmd = {
		.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId            = HAL_OSPI_FLASH_ID_1,
		.InstructionMode    = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES),
		.InstructionDtrMode = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE),
		.InstructionSize    = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS),
		.Instruction        = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? JESD216_CMD_READ_SFDP
				: JESD216_OCMD_READ_SFDP),
		.Address = addr,
		.AddressDtrMode     = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE),
		.AddressSize        = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				?  HAL_OSPI_ADDRESS_24_BITS
				:  HAL_OSPI_ADDRESS_32_BITS),
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DataMode           = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES),
		.DataDtrMode        = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE),
		.DummyCycles        = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE) ? 8U : 20U),
		.DQSMode            = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DQS_ENABLE
				: HAL_OSPI_DQS_DISABLE),
		.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	cmd.NbData = size;

	HAL_StatusTypeDef hal_ret;

	hal_ret = HAL_OSPI_Command(&dev_data->hospi, &cmd,
				HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send OSPI instruction", hal_ret);
		return -EIO;
	}

	hal_ret = HAL_OSPI_Receive(&dev_data->hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}

#endif /* sfdp_bfp */
	dev_data->cmd_status = 0;

	return 0;
}

static bool ospi_address_is_valid(const struct device *dev, off_t addr,
				  size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	size_t flash_size = dev_cfg->flash_size;

	return (addr >= 0) && ((uint64_t)addr + (uint64_t)size <= flash_size);
}

/*
 * This function Polls the WIP(Write In Progress) bit to become to 0
 * in nor_mode SPI/OPI STM32_OSPI_SPI_MODE or STM32_OSPI_OPI_MODE
 * and nor_rate transfer STR/DTR STM32_OSPI_STR_TRANSFER or STM32_OSPI_DTR_TRANSFER
 */
static int stm32_ospi_mem_ready(OSPI_HandleTypeDef *hospi, uint8_t nor_mode, uint8_t nor_rate)
{
	OSPI_RegularCmdTypeDef s_command = {0};
	OSPI_AutoPollingTypeDef s_config = {0};

	/* NOR flash memory SPI/DTR config is not a valid mode */
	if ((nor_mode == STM32_OSPI_SPI_MODE) && (nor_rate == STM32_OSPI_DTR_TRANSFER)) {
		LOG_ERR("OSPI SPI/DTR mode is not supported");
		return -ENOTSUP;
	}

	/* Configure automatic polling mode command to wait for memory ready */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_RDSR
				: SPI_NOR_OCMD_RDSR;
	s_command.AddressMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_NONE
				: HAL_OSPI_ADDRESS_8_LINES;
	s_command.AddressDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	s_command.Address = 0;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES;
	s_command.DataDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = (nor_mode == STM32_OSPI_SPI_MODE)
				? 0U
				: ((nor_rate == STM32_OSPI_DTR_TRANSFER)
					? SPI_NOR_DUMMY_REG_OCTAL_DTR
					: SPI_NOR_DUMMY_REG_OCTAL);
	s_command.NbData = (nor_rate == STM32_OSPI_DTR_TRANSFER) ? 2U : 1U;
	s_command.DQSMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DQS_ENABLE
				: HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Set the mask to  0x01 to mask all Status REG bits except WIP */
	/* Set the match to 0x00 to check if the WIP bit is Reset */
	s_config.Match              = SPI_NOR_MEM_RDY_MATCH;
	s_config.Mask               = SPI_NOR_MEM_RDY_MASK; /* Write in progress */
	s_config.MatchMode          = HAL_OSPI_MATCH_MODE_AND;
	s_config.Interval           = SPI_NOR_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop      = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_OSPI_Command(hospi, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI AutoPoll command failed");
		return -EIO;
	}

	/* Start Automatic-Polling mode to wait until the memory is ready WIP=0 */
	if (HAL_OSPI_AutoPolling(hospi, &s_config, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI AutoPoll failed");
		return -EIO;
	}

	return 0;
}

/* Enables writing to the memory sending a Write Enable and wait it is effective */
static int stm32_ospi_write_enable(OSPI_HandleTypeDef *hospi, uint8_t nor_mode, uint8_t nor_rate)
{
	OSPI_RegularCmdTypeDef s_command = {0};
	OSPI_AutoPollingTypeDef s_config = {0};

	/* Initialize the write enable command */
	s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode    = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize    = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS;
	s_command.Instruction        = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_WREN
				: SPI_NOR_OCMD_WREN;
	s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode           = HAL_OSPI_DATA_NONE;
	s_command.DummyCycles        = 0U;
	s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(hospi, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI flash write enable cmd failed");
		return -EIO;
	}

	/* Configure automatic polling mode to wait for write enabling */
	s_command.Instruction    = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_RDSR
				: SPI_NOR_OCMD_RDSR;
	s_command.AddressMode    = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES;
	s_command.AddressDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize    = HAL_OSPI_ADDRESS_32_BITS;
	s_command.Address        = 0U;
	s_command.DataMode       = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES;
	s_command.DataDtrMode    = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles    = (nor_mode == STM32_OSPI_SPI_MODE)
				? 0U
				: ((nor_rate == STM32_OSPI_DTR_TRANSFER)
				? SPI_NOR_DUMMY_REG_OCTAL_DTR
				: SPI_NOR_DUMMY_REG_OCTAL);
	s_command.NbData         = (nor_rate == STM32_OSPI_DTR_TRANSFER) ? 2U : 1U;
	s_command.DQSMode        = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DQS_ENABLE
				: HAL_OSPI_DQS_DISABLE;

	/* Send the command */
	if (HAL_OSPI_Command(hospi, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI config auto polling cmd failed");
		return -EIO;
	}

	s_config.Match           = SPI_NOR_WREN_MATCH;
	s_config.Mask            = SPI_NOR_WREN_MASK;
	s_config.MatchMode       = HAL_OSPI_MATCH_MODE_AND;
	s_config.Interval        = SPI_NOR_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop   = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_OSPI_AutoPolling(hospi, &s_config, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI config auto polling failed");
		return -EIO;
	}

	return 0;
}

/* Write Flash configuration register 2 with new dummy cycles */
static int stm32_ospi_write_cfg2reg_dummy(OSPI_HandleTypeDef *hospi,
					uint8_t nor_mode, uint8_t nor_rate)
{
	OSPI_RegularCmdTypeDef s_command;

	/* Initialize the writing of configuration register 2 */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize    = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_WR_CFGREG2
				: SPI_NOR_OCMD_WR_CFGREG2;
	s_command.AddressMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES;
	s_command.AddressDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize  = HAL_OSPI_ADDRESS_32_BITS;
	s_command.Address = SPI_NOR_REG2_ADDR3;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES;
	s_command.DataDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = 0U;
	s_command.NbData = (nor_mode == STM32_OSPI_SPI_MODE) ? 1U
			: ((nor_rate == STM32_OSPI_DTR_TRANSFER) ? 2U : 1U);
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(hospi, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI transmit ");
		return -EIO;
	}
	uint8_t tmp = SPI_NOR_CR2_DUMMY_CYCLES_66MHZ;

	if (HAL_OSPI_Transmit(hospi, &tmp, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI transmit ");
		return -EIO;
	}

	return 0;
}

/* Write Flash configuration register 2 with new single or octal SPI protocol */
static int stm32_ospi_write_cfg2reg_io(OSPI_HandleTypeDef *hospi,
				       uint8_t nor_mode, uint8_t nor_rate, uint8_t op_enable)
{
	OSPI_RegularCmdTypeDef s_command;

	if ((op_enable != SPI_NOR_CR2_STR_OPI_EN) && (op_enable != SPI_NOR_CR2_DTR_OPI_EN)) {
		LOG_ERR("OSPI wrong OSPI protocol");
		return -EIO;
	}

	/* Initialize the writing of configuration register 2 */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize    = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_WR_CFGREG2
				: SPI_NOR_OCMD_WR_CFGREG2;
	s_command.AddressMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES;
	s_command.AddressDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize  = HAL_OSPI_ADDRESS_32_BITS;
	s_command.Address = SPI_NOR_REG2_ADDR1;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES;
	s_command.DataDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = 0U;
	s_command.NbData = (nor_mode == STM32_OSPI_SPI_MODE) ? 1U
			: ((nor_rate == STM32_OSPI_DTR_TRANSFER) ? 2U : 1U);
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(hospi, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}
	uint8_t tmp = op_enable;

	if (HAL_OSPI_Transmit(hospi, &tmp, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	return 0;
}

/* Read Flash configuration register 2 with new single or octal SPI protocol */
static int stm32_ospi_read_cfg2reg(OSPI_HandleTypeDef *hospi,
				   uint8_t nor_mode, uint8_t nor_rate, uint8_t *value)
{
	OSPI_RegularCmdTypeDef s_command;

	/* Initialize the writing of configuration register 2 */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_RD_CFGREG2
				: SPI_NOR_OCMD_RD_CFGREG2;
	s_command.AddressMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES;
	s_command.AddressDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.InstructionSize = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (nor_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_RD_CFGREG2
				: SPI_NOR_OCMD_RD_CFGREG2;
	s_command.AddressMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES;
	s_command.AddressDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
	s_command.Address = SPI_NOR_REG2_ADDR1;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = (nor_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES;
	s_command.DataDtrMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = (nor_mode == STM32_OSPI_SPI_MODE)
				? 0U
				: ((nor_rate == STM32_OSPI_DTR_TRANSFER)
					? SPI_NOR_DUMMY_REG_OCTAL_DTR
					: SPI_NOR_DUMMY_REG_OCTAL);
	s_command.NbData = (nor_rate == STM32_OSPI_DTR_TRANSFER) ? 2U : 1U;
	s_command.DQSMode = (nor_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DQS_ENABLE
				: HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(hospi, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	/* Reception of the data */
	if (HAL_OSPI_Receive(hospi, value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	return 0;
}

/* Set the NOR Flash to desired Interface mode : SPI/OSPI and STR/DTR according to the DTS */
static int stm32_ospi_config_mem(const struct device *dev)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	uint8_t reg[2];

	/* going to set the SPI mode and STR transfer rate : done */
	if ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
		&& (dev_cfg->data_rate == STM32_OSPI_STR_TRANSFER)) {
		return 0;
	}

	/* going to set the OPI mode (STR or DTR transfer rate) */
	LOG_INF("OSPI configuring OctoSPI mode");

	/* Enable write operations */
	if (stm32_ospi_write_enable(&dev_data->hospi,
		STM32_OSPI_SPI_MODE, STM32_OSPI_STR_TRANSFER) != 0) {
		LOG_ERR("OSPI write Enable failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with new dummy cycles) */
	if (stm32_ospi_write_cfg2reg_dummy(&dev_data->hospi,
		STM32_OSPI_SPI_MODE, STM32_OSPI_STR_TRANSFER) != 0) {
		LOG_ERR("OSPI write CFGR2 failed");
		return -EIO;
	}

	/* wait for the memory ready */
	if (stm32_ospi_mem_ready(&dev_data->hospi,
		STM32_OSPI_SPI_MODE, STM32_OSPI_STR_TRANSFER) != 0) {
		LOG_ERR("OSPI autopolling failed");
		return -EIO;
	}
	/* Enable write operations */
	if (stm32_ospi_write_enable(&dev_data->hospi,
		STM32_OSPI_SPI_MODE, STM32_OSPI_STR_TRANSFER) != 0) {
		LOG_ERR("OSPI write Enable 2 failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with Octal I/O SPI protocol : choose STR or DTR) */
	uint8_t mode_enable = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? SPI_NOR_CR2_DTR_OPI_EN
				: SPI_NOR_CR2_STR_OPI_EN);
	if (stm32_ospi_write_cfg2reg_io(&dev_data->hospi,
		STM32_OSPI_SPI_MODE, STM32_OSPI_STR_TRANSFER, mode_enable) != 0) {
		LOG_ERR("OSPI write CFGR2 failed");
		return -EIO;
	}

	/* Wait that the configuration is effective and check that memory is ready */
	HAL_Delay(STM32_OSPI_WRITE_REG_MAX_TIME);

	/* Reconfigure the memory type of the peripheral */
	dev_data->hospi.Init.MemoryType            = HAL_OSPI_MEMTYPE_MACRONIX;
	dev_data->hospi.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
	if (HAL_OSPI_Init(&dev_data->hospi) != HAL_OK) {
		LOG_ERR("OSPI mem type MACRONIX failed");
		return -EIO;
	}

	if (dev_cfg->data_rate == STM32_OSPI_STR_TRANSFER) {
		if (stm32_ospi_mem_ready(&dev_data->hospi,
			STM32_OSPI_OPI_MODE, STM32_OSPI_STR_TRANSFER) != 0) {
		/* Check Flash busy ? */
			LOG_ERR("OSPI flash busy failed");
			return -EIO;
		}

		if (stm32_ospi_read_cfg2reg(&dev_data->hospi,
			STM32_OSPI_OPI_MODE, STM32_OSPI_STR_TRANSFER, reg) != 0) {
		/* Check the configuration has been correctly done on SPI_NOR_REG2_ADDR1 */
			LOG_ERR("OSPI flash config read failed");
			return -EIO;
		}

		LOG_INF("OSPI flash config is OPI / STR");
	}

	if (dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER) {
		if (stm32_ospi_mem_ready(&dev_data->hospi,
			STM32_OSPI_OPI_MODE, STM32_OSPI_DTR_TRANSFER) != 0) {
		/* Check Flash busy ? */
			LOG_ERR("OSPI flash busy failed");
			return -EIO;
		}

		LOG_INF("OSPI flash config is OPI / DTR");
	}

	return 0;
}

/* gpio or send the different reset command to the NOR flash in SPI/OSPI and STR/DTR */
static int stm32_ospi_mem_reset(const struct device *dev)
{
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);

#if STM32_QSPI_RESET_GPIO
	/* Generate RESETn pulse for the flash memory */
	gpio_pin_configure_dt(&dev_cfg->reset, GPIO_OUTPUT_ACTIVE);
	k_msleep(DT_INST_PROP(0, reset_gpios_duration));
	gpio_pin_set_dt(&dev_cfg->reset, 0);
#else

	/* reset command sent sucessively for each mode SPI/OPS & STR/DTR */
	OSPI_RegularCmdTypeDef s_command = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.AddressMode = HAL_OSPI_ADDRESS_NONE,
		.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE,
		.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
		.Instruction = SPI_NOR_CMD_RESET_EN,
		.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DataMode = HAL_OSPI_DATA_NONE,
		.DummyCycles = 0U,
		.DQSMode = HAL_OSPI_DQS_DISABLE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	/* reset enable in SPI mode and STR transfer mode */
	if (HAL_OSPI_Command(&dev_data->hospi,
		&s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI reset enable (SPI/STR) failed");
		return -EIO;
	}

	/* reset memory in SPI mode and STR transfer mode */
	s_command.Instruction = SPI_NOR_CMD_RESET_MEM;
	if (HAL_OSPI_Command(&dev_data->hospi,
		&s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI reset memory (SPI/STR) failed");
		return -EIO;
	}

	/* reset enable in OPI mode and STR transfer mode */
	s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.Instruction = SPI_NOR_OCMD_RESET_EN;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_16_BITS;
	if (HAL_OSPI_Command(&dev_data->hospi,
		&s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI reset enable (OPI/STR) failed");
		return -EIO;
	}

	/* reset memory in OPI mode and STR transfer mode */
	s_command.Instruction = SPI_NOR_OCMD_RESET_MEM;
	if (HAL_OSPI_Command(&dev_data->hospi,
		&s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI reset memory (OPI/STR) failed");
		return -EIO;
	}

	/* reset enable in OPI mode and DTR transfer mode */
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_ENABLE;
	s_command.Instruction = SPI_NOR_OCMD_RESET_EN;
	if (HAL_OSPI_Command(&dev_data->hospi,
		&s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI reset enable (OPI/DTR) failed");
		return -EIO;
	}

	/* reset memory in OPI mode and DTR transfer mode */
	s_command.Instruction = SPI_NOR_OCMD_RESET_MEM;
	if (HAL_OSPI_Command(&dev_data->hospi,
		&s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("OSPI reset memory (OPI/DTR) failed");
		return -EIO;
	}

#endif
	/* After SWreset CMD, wait in case SWReset occurred during erase operation */
	HAL_Delay(STM32_OSPI_RESET_MAX_TIME);

	return 0;
}

/*
 * Function to erase the flash : chip or sector with possible OSPI/SPI and STR/DTR
 * to erase the complete chip (using dedicated command) :
 *   set size >= flash size
 *   set addr = 0
 */
static int flash_stm32_ospi_erase(const struct device *dev, off_t addr,
				  size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
	struct flash_stm32_ospi_data *dev_data = DEV_DATA(dev);
	int ret = 0;

	/* erase non-zero size */
	if (size == 0) {
		return 0;
	}

	/* Maximise erase size : means the complete chip */
	if (size > dev_cfg->flash_size) {
		/* reset the addr in that case */
		addr = 0;
		size = dev_cfg->flash_size;
	}

	if (!ospi_address_is_valid(dev, addr, size)) {
		LOG_DBG("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	if ((size != SPI_NOR_SECTOR_SIZE) && (size < dev_cfg->flash_size)) {
		LOG_DBG("Error: wrong sector size 0x%x", size);
		return -ENOTSUP;
	}

	OSPI_RegularCmdTypeDef cmd_erase = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DataMode = HAL_OSPI_DATA_NONE,
		.DummyCycles = 0,
		.DQSMode = HAL_OSPI_DQS_DISABLE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	ospi_lock_thread(dev);

	/* Check Flash busy */
	if (stm32_ospi_mem_ready(&dev_data->hospi,
		dev_cfg->data_mode, dev_cfg->data_rate) != 0) {
		LOG_DBG("Erase failed : flash busy");
		return -ENOTSUP;
	}

	/* Enable write operations */
	if (stm32_ospi_write_enable(&dev_data->hospi,
		dev_cfg->data_mode, dev_cfg->data_rate) != 0) {
		LOG_DBG("Erase failed : write enable");
		return -ENOTSUP;
	}

	cmd_erase.InstructionMode    = (dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
					? HAL_OSPI_INSTRUCTION_1_LINE
					: HAL_OSPI_INSTRUCTION_8_LINES;
	cmd_erase.InstructionDtrMode = (dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
					? HAL_OSPI_INSTRUCTION_DTR_ENABLE
					: HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	cmd_erase.InstructionSize    = (dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
					? HAL_OSPI_INSTRUCTION_8_BITS
					: HAL_OSPI_INSTRUCTION_16_BITS;

	while ((size > 0) && (ret == 0)) {
		if (size == dev_cfg->flash_size) {
			/* chip erase */
			LOG_INF("Chip Erase");
			cmd_erase.Instruction = (dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
					? SPI_NOR_CMD_BULKE
					: SPI_NOR_OCMD_BULKE;
			cmd_erase.AddressMode = HAL_OSPI_ADDRESS_NONE;
			/* full chip erase command */
			ospi_send_cmd(dev, &cmd_erase);

			size -= dev_cfg->flash_size;
		} else {
			/* sector erase */
			LOG_INF("Sector Erase");			cmd_erase.Address = addr;

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
					cmd_erase.Instruction =
						(dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
						? SPI_NOR_CMD_SE /* erase sector size 3 bytes */
						: SPI_NOR_OCMD_SE;
					cmd_erase.AddressMode =
						(dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
						? HAL_OSPI_ADDRESS_1_LINE
						: HAL_OSPI_ADDRESS_8_LINES;
					cmd_erase.AddressDtrMode =
						(dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
						? HAL_OSPI_ADDRESS_DTR_ENABLE
						: HAL_OSPI_ADDRESS_DTR_DISABLE;
					cmd_erase.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
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
			}
		ret = stm32_ospi_mem_ready(&dev_data->hospi,
						 dev_cfg->data_mode, dev_cfg->data_rate);
		}
	}

	ospi_unlock_thread(dev);

	return ret;
}

/* Function to read the flash with possible OSPI/SPI and STR/DTR */
static int flash_stm32_ospi_read(const struct device *dev, off_t addr,
				 void *data, size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
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
		.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId            = HAL_OSPI_FLASH_ID_1,
		.InstructionMode    = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES),
		.InstructionDtrMode = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE),
		.InstructionSize    = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS),
		.AddressMode       = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES),
		.AddressDtrMode     = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE),
		.Address = addr,
		.AddressSize = HAL_OSPI_ADDRESS_32_BITS,
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DataMode           = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES),
		.DataDtrMode        = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE),
		/* dataSize is set by the read cmd */
		.DQSMode            = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DQS_ENABLE
				: HAL_OSPI_DQS_DISABLE),
		.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD,
		/* other parameters are set below */
	};

	LOG_INF("OSPI: read %u data", size);
	ospi_lock_thread(dev);

	/* configure other parameters */
	if (dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER) {
		/* DTR transfer rate (==> Octal mode) */
		cmd.Instruction = SPI_NOR_OCMD_DTR_RD;
		cmd.DummyCycles = SPI_NOR_DUMMY_RD_OCTAL_DTR;
	} else {
		/* STR transfer rate */
		if (dev_cfg->data_mode == STM32_OSPI_SPI_MODE) {
			/* SPI and STR : use fast read with addr on 4 bytes */
			cmd.Instruction = SPI_NOR_CMD_4B_FAST_RD;
			cmd.DummyCycles = SPI_NOR_DUMMY_RD;
		} else {
			/* OPI and STR */
			cmd.Instruction = SPI_NOR_OCMD_RD;
			cmd.DummyCycles = SPI_NOR_DUMMY_RD_OCTAL;
		}
	}

	ret = ospi_read_access(dev, &cmd, data, size);

	ospi_unlock_thread(dev);

	return ret;
}

/* Function to write the flash (page program) : with possible OSPI/SPI and STR/DTR */
static int flash_stm32_ospi_write(const struct device *dev, off_t addr,
				  const void *data, size_t size)
{
	const struct flash_stm32_ospi_config *dev_cfg = DEV_CFG(dev);
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

	/* page program for STR or DTR mode */
	OSPI_RegularCmdTypeDef cmd_pp = {
		.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
		.FlashId = HAL_OSPI_FLASH_ID_1,
		.InstructionMode = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_1_LINE
				: HAL_OSPI_INSTRUCTION_8_LINES),
		.InstructionSize = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_INSTRUCTION_8_BITS
				: HAL_OSPI_INSTRUCTION_16_BITS),
		.InstructionDtrMode = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_INSTRUCTION_DTR_ENABLE
				: HAL_OSPI_INSTRUCTION_DTR_DISABLE),
		.AddressMode = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_ADDRESS_1_LINE
				: HAL_OSPI_ADDRESS_8_LINES),
		.AddressDtrMode = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_ADDRESS_DTR_ENABLE
				: HAL_OSPI_ADDRESS_DTR_DISABLE),
		.AddressSize = HAL_OSPI_ADDRESS_32_BITS,
		.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
		.DataMode = ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? HAL_OSPI_DATA_1_LINE
				: HAL_OSPI_DATA_8_LINES),
		.DataDtrMode = ((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)
				? HAL_OSPI_DATA_DTR_ENABLE
				: HAL_OSPI_DATA_DTR_DISABLE),
		.DummyCycles = 0,
		.DQSMode = HAL_OSPI_DQS_DISABLE,
		.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
	};

	LOG_INF("OSPI: write %u data", size);
	ospi_lock_thread(dev);

	while ((size > 0) && (ret == 0)) {
		size_t to_write = size;

		ret = stm32_ospi_mem_ready(&dev_data->hospi,
						 dev_cfg->data_mode, dev_cfg->data_rate);
		if (ret != 0) {
			break;
		}

		ret = stm32_ospi_write_enable(&dev_data->hospi,
						    dev_cfg->data_mode, dev_cfg->data_rate);
		if (ret != 0) {
			break;
		}
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

		cmd_pp.Instruction = (dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
				? SPI_NOR_CMD_4B_PP
				: SPI_NOR_OCMD_PAGE_PRG;
		cmd_pp.Address = addr;
		ret = ospi_write_access(dev, &cmd_pp, data, to_write);
		if (ret != 0) {
			break;
		}

		size -= to_write;
		data = (const uint8_t *)data + to_write;
		addr += to_write;

		/* Configure automatic polling mode to wait for end of program */
		ret = stm32_ospi_mem_ready(&dev_data->hospi,
						 dev_cfg->data_mode, dev_cfg->data_rate);
		if (ret != 0) {
			break;
		}
	}

	ospi_unlock_thread(dev);

	return ret;
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

/* weak function requires for HAL compilation */
__weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}

/* weak function requires for HAL compilation */
__weak HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
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

	LOG_DBG("Error cb");

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

	LOG_DBG("Cmd Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Rx Transfer completed callback.
 */
void HAL_OSPI_RxCpltCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	LOG_DBG("Rx Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Tx Transfer completed callback.
 */
void HAL_OSPI_TxCpltCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	LOG_DBG("Tx Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Status Match callback.
 */
void HAL_OSPI_StatusMatchCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	LOG_DBG("Status Match cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Timeout callback.
 */
void HAL_OSPI_TimeOutCallback(OSPI_HandleTypeDef *hospi)
{
	struct flash_stm32_ospi_data *dev_data =
		CONTAINER_OF(hospi, struct flash_stm32_ospi_data, hospi);

	LOG_DBG("Timeout cb");

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

	uint32_t erase_size = BIT(value);

	if (erase_size == 0) {
		erase_size = SPI_NOR_SECTOR_SIZE;
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

	/* The SPI/DTR is not a valid config of data_mode/data_rate according to the DTS */
	if ((dev_cfg->data_mode == STM32_OSPI_SPI_MODE)
		&& (dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER)) {
		/* already the right config, continue */
		LOG_ERR("OSPI mode SPI/DTR is not valid");
		return -ENOTSUP;
	}

	/* Signals configuration */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("OSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Initializes the independent peripherals clock */
	__HAL_RCC_OSPI_CONFIG(RCC_OSPICLKSOURCE_SYSCLK); /* */

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
	dev_data->hospi.Init.DeviceSize = find_lsb_set(dev_cfg->flash_size);
	dev_data->hospi.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	dev_data->hospi.Init.ChipSelectHighTime = 2;
	dev_data->hospi.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	dev_data->hospi.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	dev_data->hospi.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
	dev_data->hospi.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	/* STR mode else Macronix for DTR mode */
	if (dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER) {
		dev_data->hospi.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
		dev_data->hospi.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
	} else {
		dev_data->hospi.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
		dev_data->hospi.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	}
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

	/* OCTOSPI2 delay block init Function */
	HAL_OSPI_DLYB_CfgTypeDef ospi_delay_block_cfg = {0};

	ospi_delay_block_cfg.Units = 56;
	ospi_delay_block_cfg.PhaseSel = 2;
	if (HAL_OSPI_DLYB_SetConfig(&dev_data->hospi, &ospi_delay_block_cfg) != HAL_OK) {
		LOG_ERR("OSPI DelayBlock failed");
		return -EIO;
	}
#endif /* CONFIG_SOC_SERIES_STM32U5X */

	/* reset NOR flash memory : still with the SPI/STR config for the NOR */
	if (stm32_ospi_mem_reset(dev) != 0) {
		LOG_ERR("OSPI reset failed");
		return -EIO;
	}
	/* check if memory is ready in the SPI/STR mode */
	if (stm32_ospi_mem_ready(&dev_data->hospi,
		STM32_OSPI_SPI_MODE, STM32_OSPI_STR_TRANSFER) != 0) {
		LOG_ERR("OSPI memory not ready");
		return -EIO;
	}

	if (stm32_ospi_config_mem(dev) != 0) {
		LOG_ERR("OSPI mode not config'd (%u rate %u)",
			dev_cfg->data_mode, dev_cfg->data_rate);
		return -EIO;
	}

	LOG_INF("OSPI flash config'd to %s/%s mode",
			((dev_cfg->data_mode == STM32_OSPI_OPI_MODE) ? "OPI" : "SPI"),
			((dev_cfg->data_rate == STM32_OSPI_DTR_TRANSFER) ? "DTR" : "STR"));

	/* send the instruction to read the NOR-flash ID : check access to mem. */
	uint8_t nor_id[3];

	ret = ospi_read_norid(dev, nor_id, sizeof(nor_id));
	if (ret != 0) {
		LOG_ERR("reading Nor-flash ID failed: %d", ret);
		return ret;
	}

	LOG_INF("Nor-flash ID : Manuf 0x%X, Mem Type 0x%X, Density 0x%X",
		nor_id[2], nor_id[0], nor_id[1]);

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
		LOG_ERR("SFDP magic %08x invalid", magic);
		return -EINVAL;
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
				break;
			}
		}
		++php;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	ret = setup_pages_layout(dev);
	if (ret != 0) {
		LOG_ERR("layout setup failed: %d", ret);
		return -ENODEV;
	}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	/* Initialize semaphores */
	k_sem_init(&dev_data->sem, 1, 1);
	k_sem_init(&dev_data->sync, 0, 1);

	/* Run IRQ init */
	dev_cfg->irq_config(dev);

	LOG_INF("Device %s initialized", DEV_NAME(dev));

	return 0;
}


#define OSPI_FLASH_MODULE(drv_id, flash_id)				\
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
	.data_mode = ((DT_INST_PROP(0, spi_bus_width) == 8)
		? STM32_OSPI_OPI_MODE : STM32_OSPI_SPI_MODE), /* SPI or OPI */
	.data_rate = ((DT_INST_PROP(0, data_rate) == 2)
		? STM32_OSPI_DTR_TRANSFER : STM32_OSPI_STR_TRANSFER), /* DTR or STR */
	.pcfg = PINCTRL_DT_DEV_CONFIG_GET(STM32_OSPI_NODE),
#if DT_NODE_HAS_PROP(DT_INST(0, st_stm32_ospi_nor), sfdp_bfp)
	.sfdp_bfp = DT_INST_PROP(0, sfdp_bfp),
#endif /* sfdp_bfp */
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
