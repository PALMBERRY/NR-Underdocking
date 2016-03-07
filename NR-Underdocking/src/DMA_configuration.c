/*
 * DMA-configuration.c
 *
 *  Created on: 2014-12-30
 *      Author: MiaoWork
 */

#include "DMA_configuration.h"

#ifdef c6748
volatile uint32_t* g_EDMA3CC_ESR_addr = EDMA3CC_ESR;
volatile uint32_t* g_EDMA3_0_IPR_addr = EDMA3_0_IPR;
volatile uint32_t* g_EDMA3_0_ICR_addr = EDMA3_0_ICR;

EDMA3CC_PaRAM* g_PaRAM3 = EDMA3CC_PaRAM3;
EDMA3CC_PaRAM* g_PaRAM5 = EDMA3CC_PaRAM5;
EDMA3CC_PaRAM* g_PaRAM40 = EDMA3CC_PaRAM40;
EDMA3CC_PaRAM* g_PaRAM41 = EDMA3CC_PaRAM41;
#endif
