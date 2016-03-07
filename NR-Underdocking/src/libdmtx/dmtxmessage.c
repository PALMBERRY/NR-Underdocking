/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxmessage.c
 * \brief Data message handling
 */

/**
 * \brief  Allocate memory for message
 * \param  sizeIdx
 * \param  symbolFormat DmtxFormatMatrix | DmtxFormatMosaic
 * \return Address of allocated memory
 */

#include "libdmtx/dmtx.h"
#include "libdmtx/dmtxstatic.h"

DmtxMessage g_msg;

extern DmtxMessage *
dmtxMessageCreate(int sizeIdx, int symbolFormat)
{
   DmtxMessage *message;
   int mappingRows, mappingCols;

   assert(symbolFormat == DmtxFormatMatrix || symbolFormat == DmtxFormatMosaic);

   /*mappingRows = dmtxGetSymbolAttribute(DmtxSymAttribMappingMatrixRows, sizeIdx);
   mappingCols = dmtxGetSymbolAttribute(DmtxSymAttribMappingMatrixCols, sizeIdx);*/

   mappingRows = 8;
   mappingCols = 8;

   message = &g_msg;

   memset(message, 0, sizeof(DmtxMessage));

   message->arraySize = sizeof(unsigned char) * mappingRows * mappingCols;

   message->codeSize = 8;/*sizeof(unsigned char) *
         dmtxGetSymbolAttribute(DmtxSymAttribSymbolDataWords, sizeIdx) +
         dmtxGetSymbolAttribute(DmtxSymAttribSymbolErrorWords, sizeIdx);*/

   /*if(symbolFormat == DmtxFormatMosaic)
      message->codeSize *= 3;*/

   /* XXX not sure if this is the right place or even the right approach.
      Trying to allocate memory for the decoded data stream and will
      initially assume that decoded data will not be larger than 2x encoded data */
   message->outputSize = sizeof(unsigned char) * message->codeSize * 10;

   return message;
}
