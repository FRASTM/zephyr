/*
 * Copyright (c) 2023 Jeroen van Dooren, Nobleo Technology
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Common part of MDMA drivers for stm32.
 */

#include "dma_stm32_mdma.h"

#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma/dma_stm32.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(dma_stm32_mdma, CONFIG_DMA_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32_mdma

#define MDMA_STM32_0_CHANNEL_COUNT 8

static const uint32_t table_m_size[] = {
	LL_MDMA_SRC_DATA_SIZE_BYTE,
	LL_MDMA_SRC_DATA_SIZE_HALFWORD,
	LL_MDMA_SRC_DATA_SIZE_WORD,
	LL_MDMA_SRC_DATA_SIZE_DOUBLEWORD,
};

static const uint32_t table_p_size[] = {
	LL_MDMA_DEST_DATA_SIZE_BYTE,
	LL_MDMA_DEST_DATA_SIZE_HALFWORD,
	LL_MDMA_DEST_DATA_SIZE_WORD,
	LL_MDMA_DEST_DATA_SIZE_DOUBLEWORD,
};

uint32_t mdma_stm32_id_to_channel(uint32_t id)
{
	static const uint32_t channel_nr[] = {
		LL_MDMA_CHANNEL_0,
		LL_MDMA_CHANNEL_1,
		LL_MDMA_CHANNEL_2,
		LL_MDMA_CHANNEL_3,
		LL_MDMA_CHANNEL_4,
		LL_MDMA_CHANNEL_5,
		LL_MDMA_CHANNEL_6,
		LL_MDMA_CHANNEL_7,
		LL_MDMA_CHANNEL_8,
		LL_MDMA_CHANNEL_9,
		LL_MDMA_CHANNEL_10,
		LL_MDMA_CHANNEL_11,
		LL_MDMA_CHANNEL_12,
		LL_MDMA_CHANNEL_13,
		LL_MDMA_CHANNEL_14,
		LL_MDMA_CHANNEL_15,
	};

	__ASSERT_NO_MSG(id < ARRAY_SIZE(channel_nr));

	return channel_nr[id];
}

uint32_t mdma_stm32_slot_to_channel(uint32_t slot)
{
	static const uint32_t channel_nr[] = {
		LL_MDMA_CHANNEL_0,
		LL_MDMA_CHANNEL_1,
		LL_MDMA_CHANNEL_2,
		LL_MDMA_CHANNEL_3,
		LL_MDMA_CHANNEL_4,
		LL_MDMA_CHANNEL_5,
		LL_MDMA_CHANNEL_6,
		LL_MDMA_CHANNEL_7,
		LL_MDMA_CHANNEL_8,
		LL_MDMA_CHANNEL_9,
		LL_MDMA_CHANNEL_10,
		LL_MDMA_CHANNEL_11,
		LL_MDMA_CHANNEL_12,
		LL_MDMA_CHANNEL_13,
		LL_MDMA_CHANNEL_14,
		LL_MDMA_CHANNEL_15,
	};

	__ASSERT_NO_MSG(slot < ARRAY_SIZE(channel_nr));

	return channel_nr[slot];
}

void mdma_stm32_clear_tc(MDMA_TypeDef *dma, uint32_t id)
{
	LL_MDMA_ClearFlag_TC(dma, mdma_stm32_id_to_channel(id));
}

bool mdma_stm32_is_tc_active(MDMA_TypeDef *dma, uint32_t id)
{
	return LL_MDMA_IsActiveFlag_TC(dma, mdma_stm32_id_to_channel(id));
}

void mdma_stm32_clear_te(MDMA_TypeDef *dma, uint32_t id)
{
	LL_MDMA_ClearFlag_TE(dma, mdma_stm32_id_to_channel(id));
}

void mdma_stm32_clear_gi(MDMA_TypeDef *dma, uint32_t id)
{
	LL_MDMA_ClearFlag_TE(dma, mdma_stm32_id_to_channel(id));
	LL_MDMA_ClearFlag_TC(dma, mdma_stm32_id_to_channel(id));
	LL_MDMA_ClearFlag_CTC(dma, mdma_stm32_id_to_channel(id));
	LL_MDMA_ClearFlag_BT(dma, mdma_stm32_id_to_channel(id));
	LL_MDMA_ClearFlag_BRT(dma, mdma_stm32_id_to_channel(id));
}

bool mdma_stm32_is_te_active(MDMA_TypeDef *dma, uint32_t id)
{
	return LL_MDMA_IsActiveFlag_TE(dma, mdma_stm32_id_to_channel(id));
}

bool mdma_stm32_is_gi_active(MDMA_TypeDef *dma, uint32_t id)
{
	return LL_MDMA_IsActiveFlag_GI(dma, mdma_stm32_id_to_channel(id));
}

void stm32_mdma_dump_channel_irq(MDMA_TypeDef *dma, uint32_t id)
{
	LOG_INF("te: %d, tc: %d, gi: %d",
		mdma_stm32_is_te_active(dma, id),
		mdma_stm32_is_tc_active(dma, id),
		mdma_stm32_is_gi_active(dma, id));
}

inline bool stm32_mdma_is_tc_irq_active(MDMA_TypeDef *dma, uint32_t id)
{
	return LL_MDMA_IsEnabledIT_TC(dma, mdma_stm32_id_to_channel(id)) &&
	       mdma_stm32_is_tc_active(dma, id);
}

static inline bool stm32_mdma_is_te_irq_active(MDMA_TypeDef *dma, uint32_t id)
{
	return LL_MDMA_IsEnabledIT_TE(dma, mdma_stm32_id_to_channel(id)) &&
	       mdma_stm32_is_te_active(dma, id);
}

bool stm32_mdma_is_irq_active(MDMA_TypeDef *dma, uint32_t id)
{
	return stm32_mdma_is_tc_irq_active(dma, id) ||
	       stm32_mdma_is_te_irq_active(dma, id);
}

void stm32_mdma_clear_channel_irq(MDMA_TypeDef *dma, uint32_t id)
{
	mdma_stm32_clear_gi(dma, id);
	mdma_stm32_clear_tc(dma, id);
	mdma_stm32_clear_te(dma, id);
}

bool stm32_mdma_is_enabled_channel(MDMA_TypeDef *dma, uint32_t id)
{
	if (LL_MDMA_IsEnabledChannel(dma, mdma_stm32_id_to_channel(id)) == 1) {
		return true;
	}
	return false;
}

int stm32_mdma_disable_channel(MDMA_TypeDef *dma, uint32_t id)
{
	LL_MDMA_DisableChannel(dma, mdma_stm32_id_to_channel(id));

	if (!LL_MDMA_IsEnabledChannel(dma, mdma_stm32_id_to_channel(id))) {
		return 0;
	}

	return -EAGAIN;
}

void stm32_mdma_enable_channel(MDMA_TypeDef *dma, uint32_t id)
{
	LL_MDMA_EnableChannel(dma, mdma_stm32_id_to_channel(id));
}

static void mdma_stm32_dump_channel_irq(const struct device *dev, uint32_t id)
{
	const struct mdma_stm32_config *config = dev->config;
	MDMA_TypeDef *dma = (MDMA_TypeDef *)(config->base);

	stm32_mdma_dump_channel_irq(dma, id);
}

static void mdma_stm32_clear_channel_irq(const struct device *dev, uint32_t id)
{
	const struct mdma_stm32_config *config = dev->config;
	MDMA_TypeDef *dma = (MDMA_TypeDef *)(config->base);

	mdma_stm32_clear_tc(dma, id);
	stm32_mdma_clear_channel_irq(dma, id);
}

static void mdma_stm32_irq_handler(const struct device *dev, uint32_t id)
{
	const struct mdma_stm32_config *config = dev->config;
	MDMA_TypeDef *dma = (MDMA_TypeDef *)(config->base);
	struct mdma_stm32_channel *channel;
	uint32_t callback_arg;

	__ASSERT_NO_MSG(id < config->max_channels);

	channel = &config->channels[id];

	/* The busy channel is pertinent if not overridden by the HAL */
	if ((channel->hal_override != true) && (channel->busy == false)) {
		/*
		 * When DMA channel is not overridden by HAL,
		 * ignore irq if the channel is not busy anymore
		 */
		mdma_stm32_clear_channel_irq(dev, id);
		return;
	}

#ifdef CONFIG_DMAMUX_STM32
	callback_arg = channel->mux_channel;
#else
	callback_arg = id;
#endif /* CONFIG_DMAMUX_STM32 */

	if (!IS_ENABLED(CONFIG_DMAMUX_STM32)) {
		channel->busy = false;
	}

	/* the dma channel id is in range from 0..<dma-requests> */
	if (stm32_mdma_is_tc_irq_active(dma, id)) {
#ifdef CONFIG_DMAMUX_STM32
		channel->busy = false;
#endif
		/* Let HAL DMA handle flags on its own */
		if (!channel->hal_override) {
			mdma_stm32_clear_tc(dma, id);
		}
		channel->mdma_callback(dev, channel->user_data, callback_arg, 0);
	} else {
		LOG_ERR("Transfer Error.");
		mdma_stm32_dump_channel_irq(dev, id);
		mdma_stm32_clear_channel_irq(dev, id);
		channel->mdma_callback(dev, channel->user_data,
				     callback_arg, -EIO);
	}
}

static int mdma_stm32_get_priority(uint8_t priority, uint32_t *ll_priority)
{
	switch (priority) {
	case 0x0:
		*ll_priority = LL_MDMA_PRIORITY_LOW;
		break;
	case 0x1:
		*ll_priority = LL_MDMA_PRIORITY_MEDIUM;
		break;
	case 0x2:
		*ll_priority = LL_MDMA_PRIORITY_HIGH;
		break;
	case 0x3:
		*ll_priority = LL_MDMA_PRIORITY_VERYHIGH;
		break;
	default:
		LOG_ERR("Priority error. %d", priority);
		return -EINVAL;
	}

	return 0;
}

static int mdma_stm32_get_direction(enum dma_channel_direction direction,
				   uint32_t *ll_direction)
{
	return 0;
}

static int mdma_stm32_get_memory_increment(enum dma_addr_adj increment,
					  uint32_t *ll_increment)
{
	switch (increment) {
	case DMA_ADDR_ADJ_INCREMENT:
		*ll_increment = LL_MDMA_SRC_INCREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*ll_increment = LL_MDMA_SRC_FIXED;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		*ll_increment = LL_MDMA_SRC_DECREMENT;
	default:
		LOG_ERR("Source increment error. %d", increment);
		return -EINVAL;
	}

	return 0;
}

static int mdma_stm32_get_periph_increment(enum dma_addr_adj increment,
					  uint32_t *ll_increment)
{
	switch (increment) {
	case DMA_ADDR_ADJ_INCREMENT:
		*ll_increment = LL_MDMA_DEST_INCREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*ll_increment = LL_MDMA_DEST_FIXED;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		*ll_increment = LL_MDMA_DEST_DECREMENT;
	default:
		LOG_ERR("Dest increment error. %d", increment);
		return -EINVAL;
	}

	return 0;
}

static int mdma_stm32_disable_channel(MDMA_TypeDef *mdma, uint32_t id)
{
	int count = 0;

	for (;;) {
		if (stm32_mdma_disable_channel(mdma, id) == 0) {
			return 0;
		}
		/* After trying for 5 seconds, give up */
		if (count++ > (5 * 1000)) {
			return -EBUSY;
		}
		k_sleep(K_MSEC(1));
	}

	return 0;
}

static bool mdma_stm32_is_valid_memory_address(const uint32_t address, const uint32_t size)
{
	/* The MDMA can only access memory addresses in SRAM4 */

	const uint32_t sram4_start = DT_REG_ADDR(DT_NODELABEL(sram4));
	const uint32_t sram4_end = sram4_start + DT_REG_SIZE(DT_NODELABEL(sram4));

	if (address < sram4_start) {
		return false;
	}

	if (address + size > sram4_end) {
		return false;
	}

	return true;
}


MDMA_STM32_EXPORT_API int mdma_stm32_configure(const struct device *dev,
					     uint32_t id,
					     struct dma_config *config)
{
	const struct mdma_stm32_config *dev_config = dev->config;
	struct mdma_stm32_channel *channel =
				&dev_config->channels[id];
	MDMA_TypeDef *mdma = (MDMA_TypeDef *)dev_config->base;
	LL_MDMA_InitTypeDef MDMA_InitStruct;
	int index;
	int ret;

	LL_MDMA_StructInit(&MDMA_InitStruct);

	if (id >= dev_config->max_channels) {
		LOG_ERR("cannot configure the mdma channel %d.", id);
		return -EINVAL;
	}

	if (channel->busy) {
		LOG_ERR("mdma channel %d is busy.", id);
		return -EBUSY;
	}

	if (mdma_stm32_disable_channel(mdma, id) != 0) {
		LOG_ERR("could not disable mdma channel %d.", id);
		return -EBUSY;
	}

	mdma_stm32_clear_channel_irq(dev, id);

	if (config->head_block->block_size > MDMA_STM32_MAX_DATA_ITEMS) {
		LOG_ERR("Data size too big: %d\n",
		       config->head_block->block_size);
		return -EINVAL;
	}

	if ((config->channel_direction == MEMORY_TO_MEMORY) &&
		(!dev_config->support_m2m)) {
		LOG_ERR("Memcopy not supported for device %s",
			dev->name);
		return -ENOTSUP;
	}

	/* support only the same data width for source and dest */
	if (config->dest_data_size != config->source_data_size) {
		LOG_ERR("source and dest data size differ.");
		return -EINVAL;
	}

	if (config->source_data_size != 4U &&
	    config->source_data_size != 2U &&
	    config->source_data_size != 1U) {
		LOG_ERR("source and dest unit size error, %d",
			config->source_data_size);
		return -EINVAL;
	}

	/*
	 * STM32's circular mode will auto reset both source address
	 * counter and destination address counter.
	 */
	if (config->head_block->source_reload_en !=
		config->head_block->dest_reload_en) {
		LOG_ERR("source_reload_en and dest_reload_en must "
			"be the same.");
		return -EINVAL;
	}

	channel->busy		= true;
	channel->mdma_callback	= config->dma_callback;
	channel->direction	= config->channel_direction;
	channel->user_data	= config->user_data;
	channel->src_size	= config->source_data_size;
	channel->dst_size	= config->dest_data_size;

	/* check dest or source memory address, warn if 0 */
	if (config->head_block->source_address == 0) {
		LOG_WRN("source_buffer address is null.");
	}

	if (config->head_block->dest_address == 0) {
		LOG_WRN("dest_buffer address is null.");
	}

	/* ensure all memory addresses are in SRAM4 */
	if (channel->direction == MEMORY_TO_PERIPHERAL || channel->direction == MEMORY_TO_MEMORY) {
		if (!mdma_stm32_is_valid_memory_address(config->head_block->source_address,
							config->head_block->block_size)) {
			LOG_ERR("invalid source address");
			return -EINVAL;
		}
	}
	if (channel->direction == PERIPHERAL_TO_MEMORY || channel->direction == MEMORY_TO_MEMORY) {
		if (!mdma_stm32_is_valid_memory_address(config->head_block->dest_address,
							config->head_block->block_size)) {
			LOG_ERR("invalid destination address");
			return -EINVAL;
		}
	}

	if (channel->direction == MEMORY_TO_PERIPHERAL) {
		MDMA_InitStruct.SrcAddress =
					config->head_block->source_address;
		MDMA_InitStruct.DstAddress =
					config->head_block->dest_address;
	} else {
		MDMA_InitStruct.DstAddress =
					config->head_block->source_address;
		MDMA_InitStruct.SrcAddress =
					config->head_block->dest_address;
	}

	uint16_t memory_addr_adj = 0, periph_addr_adj = 0;

	ret = mdma_stm32_get_priority(config->channel_priority,
				     &MDMA_InitStruct.Priority);
	if (ret < 0) {
		return ret;
	}

/*	ret = mdma_stm32_get_direction(config->channel_direction,
				      &MDMA_InitStruct.Direction);
	if (ret < 0) {
		return ret;
	}*/

	switch (config->channel_direction) {
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		memory_addr_adj = config->head_block->dest_addr_adj;
		periph_addr_adj = config->head_block->source_addr_adj;
		break;
	case MEMORY_TO_PERIPHERAL:
		memory_addr_adj = config->head_block->source_addr_adj;
		periph_addr_adj = config->head_block->dest_addr_adj;
		break;
	/* Direction has been asserted in mdma_stm32_get_direction. */
	default:
		LOG_ERR("Channel direction error (%d).",
				config->channel_direction);
		return -EINVAL;
	}

	ret = mdma_stm32_get_memory_increment(memory_addr_adj,
					&MDMA_InitStruct.SrcIncMode);
	if (ret < 0) {
		return ret;
	}
	ret = mdma_stm32_get_periph_increment(periph_addr_adj,
					&MDMA_InitStruct.DestIncMode);
	if (ret < 0) {
		return ret;
	}

/*
 *	if (config->head_block->source_reload_en) {
		MDMA_InitStruct.Mode = LL_MDMA_MODE_CIRCULAR;
	} else {
		MDMA_InitStruct.Mode = LL_MDMA_MODE_NORMAL;
	}
*/
	channel->source_periph = (channel->direction == PERIPHERAL_TO_MEMORY);

	/* set the data width, when source_data_size equals dest_data_size */
	index = find_lsb_set(config->source_data_size) - 1;
	MDMA_InitStruct.DestDataSize = table_p_size[index];
	index = find_lsb_set(config->dest_data_size) - 1;
	MDMA_InitStruct.SrcDataSize = table_m_size[index];

/*
	if (channel->source_periph) {
		MDMA_InitStruct.NbData = config->head_block->block_size /
					config->source_data_size;
	} else {
		MDMA_InitStruct.NbData = config->head_block->block_size /
					config->dest_data_size;
	}
*/

#if defined(CONFIG_DMAMUX_STM32)
	/*
	 * with mdma mux,
	 * the request ID is stored in the dma_slot
	 */
	MDMA_InitStruct.PeriphRequest = config->dma_slot;
#endif
	LL_MDMA_Init(mdma, mdma_stm32_id_to_channel(id), &MDMA_InitStruct);

	LL_MDMA_EnableIT_TC(mdma, mdma_stm32_id_to_channel(id));

	return ret;
}

MDMA_STM32_EXPORT_API int mdma_stm32_reload(const struct device *dev, uint32_t id,
					  uint32_t src, uint32_t dst,
					  size_t size)
{
	const struct mdma_stm32_config *config = dev->config;
	MDMA_TypeDef *mdma = (MDMA_TypeDef *)(config->base);
	struct mdma_stm32_channel *channel;

	if (id >= config->max_channels) {
		return -EINVAL;
	}

	channel = &config->channels[id];

	if (mdma_stm32_disable_channel(mdma, id) != 0) {
		return -EBUSY;
	}

	switch (channel->direction) {
	case MEMORY_TO_PERIPHERAL:
		LL_MDMA_SetSourceAddress(mdma, mdma_stm32_id_to_channel(id), src);
		LL_MDMA_SetDestinationAddress(mdma, mdma_stm32_id_to_channel(id), dst);
		break;
	case MEMORY_TO_MEMORY:
	case PERIPHERAL_TO_MEMORY:
		LL_MDMA_SetDestinationAddress(mdma, mdma_stm32_id_to_channel(id), src);
		LL_MDMA_SetSourceAddress(mdma, mdma_stm32_id_to_channel(id), dst);
		break;
	default:
		return -EINVAL;
	}

	if (channel->source_periph) {
		LL_MDMA_SetBlkDataLength(mdma, mdma_stm32_id_to_channel(id),
				     size / channel->src_size);
	} else {
		LL_MDMA_SetBlkDataLength(mdma, mdma_stm32_id_to_channel(id),
				     size / channel->dst_size);
	}

	/* When reloading the dma, the channel is busy again before enabling */
	channel->busy = true;

	stm32_mdma_enable_channel(mdma, id);

	return 0;
}

MDMA_STM32_EXPORT_API int mdma_stm32_start(const struct device *dev, uint32_t id)
{
	const struct mdma_stm32_config *config = dev->config;
	MDMA_TypeDef *mdma = (MDMA_TypeDef *)(config->base);
	struct mdma_stm32_channel *channel;

	/* Only M2P or M2M mode can be started manually. */
	if (id >= config->max_channels) {
		return -EINVAL;
	}

	/* Repeated start : return now if channel is already started */
	if (stm32_mdma_is_enabled_channel(mdma, id)) {
		return 0;
	}

	/* When starting the dma, the channel is busy before enabling */
	channel = &config->channels[id];
	channel->busy = true;

	mdma_stm32_clear_channel_irq(dev, id);
	stm32_mdma_enable_channel(mdma, id);

	return 0;
}

MDMA_STM32_EXPORT_API int mdma_stm32_stop(const struct device *dev, uint32_t id)
{
	const struct mdma_stm32_config *config = dev->config;
	struct mdma_stm32_channel *channel = &config->channels[id];
	MDMA_TypeDef *mdma = (MDMA_TypeDef *)(config->base);

	if (id >= config->max_channels) {
		return -EINVAL;
	}

	/* Repeated stop : return now if channel is already stopped */
	if (!stm32_mdma_is_enabled_channel(mdma, id)) {
		return 0;
	}

	/* in mdma_stm32_configure, enabling is done regardless of defines */
	LL_MDMA_DisableIT_TC(mdma, mdma_stm32_id_to_channel(id));

	mdma_stm32_disable_channel(mdma, id);
	mdma_stm32_clear_channel_irq(dev, id);

	/* Finally, flag channel as free */
	channel->busy = false;

	return 0;
}

static int mdma_stm32_init(const struct device *dev)
{
	const struct mdma_stm32_config *config = dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	if (clock_control_on(clk,
		(clock_control_subsys_t *) &config->pclken) != 0) {
		LOG_ERR("clock op failed\n");
		return -EIO;
	}

	config->config_irq(dev);

	for (uint32_t i = 0; i < config->max_channels; i++) {
		config->channels[i].busy = false;
	}

	((struct mdma_stm32_data *)dev->data)->dma_ctx.magic = 0;
	((struct mdma_stm32_data *)dev->data)->dma_ctx.dma_channels = 0;
	((struct mdma_stm32_data *)dev->data)->dma_ctx.atomic = 0;

	return 0;
}

MDMA_STM32_EXPORT_API int mdma_stm32_get_status(const struct device *dev,
				uint32_t id, struct dma_status *stat)
{
	const struct mdma_stm32_config *config = dev->config;
	MDMA_TypeDef *mdma = (MDMA_TypeDef *)(config->base);
	struct mdma_stm32_channel *channel;

	if (id >= config->max_channels) {
		return -EINVAL;
	}

	channel = &config->channels[id];
	stat->pending_length = LL_MDMA_GetBlkDataLength(mdma, mdma_stm32_id_to_channel(id));
	stat->dir = channel->direction;
	stat->busy = channel->busy;

	return 0;
}

static const struct dma_driver_api dma_funcs = {
	.reload		 = mdma_stm32_reload,
	.config		 = mdma_stm32_configure,
	.start		 = mdma_stm32_start,
	.stop		 = mdma_stm32_stop,
	.get_status	 = mdma_stm32_get_status,
};

#define MDMA_STM32_OFFSET_INIT(index)

#define MDMA_STM32_INIT_DEV(index)					\
static struct mdma_stm32_channel					\
	mdma_stm32_channels_##index[MDMA_STM32_##index##_CHANNEL_COUNT];\
									\
const struct mdma_stm32_config mdma_stm32_config_##index = {		\
	.pclken = { .bus = DT_INST_CLOCKS_CELL(index, bus),		\
		    .enr = DT_INST_CLOCKS_CELL(index, bits) },		\
	.config_irq = mdma_stm32_config_irq_##index,			\
	.base = DT_INST_REG_ADDR(index),				\
	.support_m2m = DT_INST_PROP(index, st_mem2mem),			\
	.max_channels = MDMA_STM32_##index##_CHANNEL_COUNT,		\
	.channels = mdma_stm32_channels_##index,			\
	MDMA_STM32_OFFSET_INIT(index)					\
};									\
									\
static struct mdma_stm32_data mdma_stm32_data_##index = {		\
};									\
									\
DEVICE_DT_INST_DEFINE(index,							\
		    &mdma_stm32_init,						\
		    NULL,							\
		    &mdma_stm32_data_##index, &mdma_stm32_config_##index,	\
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		\
		    &dma_funcs)

#define MDMA_STM32_DEFINE_IRQ_HANDLER(mdma, chan)			\
static void mdma_stm32_irq_##mdma##_##chan(const struct device *dev)	\
{									\
	mdma_stm32_irq_handler(dev, chan);				\
}


#define MDMA_STM32_IRQ_CONNECT(mdma, chan)				\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(mdma, chan, irq),	\
			    DT_INST_IRQ_BY_IDX(mdma, chan, priority),	\
			    mdma_stm32_irq_##mdma##_##chan,		\
			    DEVICE_DT_INST_GET(mdma), 0);		\
		irq_enable(DT_INST_IRQ_BY_IDX(mdma, chan, irq));	\
	} while (false)


#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)

#define MDMA_STM32_DEFINE_IRQ_HANDLER_GEN(i, _) \
	MDMA_STM32_DEFINE_IRQ_HANDLER(0, i)
LISTIFY(DT_NUM_IRQS(DT_DRV_INST(0)), MDMA_STM32_DEFINE_IRQ_HANDLER_GEN, (;));

static void mdma_stm32_config_irq_0(const struct device *dev)
{
	ARG_UNUSED(dev);

	#define MDMA_STM32_IRQ_CONNECT_GEN(i, _) \
		MDMA_STM32_IRQ_CONNECT(0, i);
	LISTIFY(DT_NUM_IRQS(DT_DRV_INST(0)), MDMA_STM32_IRQ_CONNECT_GEN, (;));
}

MDMA_STM32_INIT_DEV(0);

#endif /* DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay) */
