/*
 * Copyright (c) 2021 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sram2), okay) && \
	(DT_NODE_HAS_STATUS(DT_NODELABEL(dma1), okay) || \
	DT_NODE_HAS_STATUS(DT_NODELABEL(dma2), okay))

SECTION_DATA_PROLOGUE(sram_dma,(NOLOAD),)
{
    . = ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(sram2)));
    *(.sram_dma)
    . = ABSOLUTE(DT_REG_ADDR(DT_NODELABEL(sram2))) + 16K;
} GROUP_DATA_LINK_IN(SRAM2, SRAM2)

#endif
