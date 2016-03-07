/**
 * @file DMA-configuration.h
 * @brief DMA相关配置
 *
 * @author 戴舒炜,钟宬
 * @date 2014.12.29
 *
 * 修订说明： 最初版本
 */
//#include <stdint.h>

#ifndef DMA_CONFIGURATION_H_
#define DMA_CONFIGURATION_H_

#ifdef c6748
#define EDMA3CC_ESR			(uint32_t*)0x01C01010	/**< Event Set Register */
#define EDMA3_0_IPR			(uint32_t*)0x01C01068	/**< Interrupt Pending Register */
#define EDMA3_0_ICR			(uint32_t*)0x01C01070	/**< Interrupt Clear Register */

/**
 * EDMA3CC Parameter type
 */
typedef struct _EDMA3CC_PaRAM
{
   volatile uint32_t OPT;			/**< Channel Options Parameter */
   volatile uint8_t *SRC;			/**< Channel Source Address Parameter */
   volatile uint32_t A_B_CNT;		/**< A Count/B Count Parameter */
   volatile uint8_t *DST;			/**< Channel Destination Address Parameter */
   volatile uint32_t SRC_DST_BIDX;	/**< Source B Index/Destination B Index Parameter */
   volatile uint32_t LINK_BCNTRLD;	/**< Link Address/B Count Reload Parameter */
   volatile uint32_t SRC_DST_CIDX;	/**< Source C Index/Destination C Index Parameter */
   volatile uint32_t CCNT;			/**< C Count Parameter */
}EDMA3CC_PaRAM;

#define EDMA3CC_PaRAM3			(EDMA3CC_PaRAM*)0x01C04060	/**< EDMA3CC Parameter3 Address */
#define EDMA3CC_PaRAM5 			(EDMA3CC_PaRAM*)0x01C040A0	/**< EDMA3CC Parameter5 Address */
#define EDMA3CC_PaRAM40			(EDMA3CC_PaRAM*)0x01C04800	/**< EDMA3CC Parameter40 Address */
#define EDMA3CC_PaRAM40_LINK	(uint32_t)0x00000800		/**< LINK Address To EDMA3CC Parameter40 */
#define EDMA3CC_PaRAM41			(EDMA3CC_PaRAM*)0x01C04820	/**< EDMA3CC Parameter41 Address */
#define EDMA3CC_PaRAM41_LINK	(uint32_t)0x00000820		/**< LINK Address To EDMA3CC Parameter41 */

extern volatile uint32_t* g_EDMA3CC_ESR_addr;
extern volatile uint32_t* g_EDMA3_0_IPR_addr;
extern volatile uint32_t* g_EDMA3_0_ICR_addr;

extern EDMA3CC_PaRAM* g_PaRAM3;
extern EDMA3CC_PaRAM* g_PaRAM5;
extern EDMA3CC_PaRAM* g_PaRAM40;
extern EDMA3CC_PaRAM* g_PaRAM41;

/** Configure EDMA3CC Parameter
 *
 * @param 	PaRAM EDMA3CC	Parameter Address
 * @param 	PaRAM_num		Parameter Number
 * @param	SRC				Channel Source Address Parameter
 * @param	DST				Channel Destination Address Parammeter
 * @param	ACNT			A Count Parameter
 * @param	BCNT			B Count Parameter
 * @param	SRC_BIDX		Source B Index Parameter
 * @param	DST_BIDX		Destination B Index Parameter
 * @param	SRC_CIDX		Source C Index Parameter
 * @param	DST_CIDX		Destination C Index Parameter
 * @param	CCNT			C Count Parameter
 * @param	LINK			Link Address
 */
inline void configureEDMA3CC(EDMA3CC_PaRAM* PaRAM, uint16_t PaRAM_num, const uint8_t* SRC, uint8_t* DST,
							uint32_t ACNT, uint32_t BCNT, uint32_t SRC_BIDX, uint32_t DST_BIDX,
							uint32_t SRC_CIDX, uint32_t DST_CIDX, uint32_t CCNT, uint32_t LINK)
{
	PaRAM->OPT = (1<<20)|(PaRAM_num<<12)|(1<<2);
	PaRAM->SRC = (volatile uint8_t*)SRC;
	PaRAM->A_B_CNT = (uint32_t)(((BCNT<<16)&0xFFFF0000)|(ACNT&0x0000FFFF));
	PaRAM->DST = DST;
	PaRAM->SRC_DST_BIDX = (uint32_t)(((DST_BIDX<<16)&0xFFFF0000)|(SRC_BIDX&0x0000FFFF));
	PaRAM->LINK_BCNTRLD = (uint32_t)(LINK&0x0000FFFF);
	PaRAM->SRC_DST_CIDX = (uint32_t)(((DST_CIDX<<16)&0xFFFF0000)|(SRC_CIDX&0x0000FFFF));
	PaRAM->CCNT = CCNT;
}

inline void startEDMA3(uint16_t PaRAM_num)
{
	*g_EDMA3CC_ESR_addr = 1<<PaRAM_num;
}

inline int waitForEDMA3(uint16_t PaRAM_num)
{
	int i = 0;

	g_EDMA3_0_IPR_addr = EDMA3_0_IPR;
	while((*g_EDMA3_0_IPR_addr & (1 << PaRAM_num)) == 0)
		i++;
	*g_EDMA3_0_ICR_addr = 1<<PaRAM_num;

	return i;
}


#endif

#endif /* DMA_CONFIGUREATION_H_ */
