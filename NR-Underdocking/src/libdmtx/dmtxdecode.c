/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 * Copyright 2009 Mackenzie Straight. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxdecode.c
 * \brief Decode regions
 */

/**
 * \brief  Initialize decode struct with default values
 * \param  img
 * \return Initialized DmtxDecode struct
 */
#include "libdmtx/dmtx.h"
#include "libdmtx/dmtxstatic.h"

#include "project.h"

DmtxDecode g_dec;

extern Inter_Linear_Table dm_proj_table[DOWN_PROJ_IMG_SIZE*DOWN_PROJ_IMG_SIZE];

extern Camera_Param* cur_camera;

#pragma DATA_SECTION(g_cache, ".cache");
#pragma DATA_ALIGN(g_cache, 8)
DmtxImgInfo g_cache[UP_PROJ_IMG_SIZE*UP_PROJ_IMG_SIZE];

int g_minArea;

extern DmtxDecode *
dmtxDecodeCreate(DmtxImage *img, int16_t last_bottom_angle)
{
   DmtxDecode *dec;
   int width, height;

   dec = &g_dec;
   //if(dec == NULL)
   //   return NULL;

   width = dmtxImageGetProp(img, DmtxPropWidth);
   height = dmtxImageGetProp(img, DmtxPropHeight);

   //dec->edgeMin = DmtxUndefined;
   if(cur_camera->camera_id == 0)
   {
	   dec->edgeMin = DOWN_EDGE_MIN;//38;//38*PROJ_IMG_X/300;
	   g_minArea = DOWN_EDGE_MIN*DOWN_EDGE_MIN;
   }
   else
   {
	   dec->edgeMin = UP_EDGE_MIN;
	   g_minArea = UP_EDGE_MIN*UP_EDGE_MIN;
   }
   //dec->edgeMax = DmtxUndefined;
   dec->edgeMax = EDGE_MAX;//48;//48*PROJ_IMG_X/300;
   dec->scanGap = 1;
   dec->squareDevn = cosdp_i(50 * (M_PI/180));
   //dec->sizeIdxExpected = DmtxSymbolShapeAuto;
   dec->sizeIdxExpected = DmtxSymbol10x10;
   dec->edgeThresh = 10;

   dec->xMin = 0;
   dec->xMax = width - 1;
   dec->yMin = 0;
   dec->yMax = height - 1;
   dec->scale = 1;

   dec->last_bottom_angle = last_bottom_angle;

   if(cur_camera->camera_id == 0)
	   dec->cache = (DmtxImgInfo*)dm_proj_table;
   else
	   dec->cache = g_cache;
   memset(dec->cache, 0, g_dm_proj_img_size*g_dm_proj_img_size*sizeof(DmtxImgInfo));

   dec->image = img;
   dec->grid = InitScanGrid(dec);

   return dec;
}

/**
 * \brief  Get decoding behavior property
 * \param  dec
 * \param  prop
 * \return value
 */
extern int
dmtxDecodeGetProp(DmtxDecode *dec, int prop)
{
   switch(prop) {
      case DmtxPropEdgeMin:
         return dec->edgeMin;
      case DmtxPropEdgeMax:
         return dec->edgeMax;
      case DmtxPropScanGap:
         return dec->scanGap;
      case DmtxPropSquareDevn:
         return _dpint(acos(dec->squareDevn) * (180.0/M_PI));
      case DmtxPropSymbolSize:
         return dec->sizeIdxExpected;
      case DmtxPropEdgeThresh:
         return dec->edgeThresh;
      case DmtxPropXmin:
         return dec->xMin;
      case DmtxPropXmax:
         return dec->xMax;
      case DmtxPropYmin:
         return dec->yMin;
      case DmtxPropYmax:
         return dec->yMax;
      case DmtxPropScale:
         return dec->scale;
      case DmtxPropWidth:
         return dmtxImageGetProp(dec->image, DmtxPropWidth);
      case DmtxPropHeight:
         return dmtxImageGetProp(dec->image, DmtxPropHeight);
      default:
         break;
   }

   return DmtxUndefined;
}

/**
 * \brief  Returns xxx
 * \param  img
 * \param  Scaled x coordinate
 * \param  Scaled y coordinate
 * \return Scaled pixel offset
 */
extern DmtxImgInfo *
dmtxDecodeGetCache(DmtxDecode *dec, int x, int y)
{
   //int width, height;

   assert(dec != NULL);

/* if(dec.cacheComplete == DmtxFalse)
      CacheImage(); */

   /*width = dmtxDecodeGetProp(dec, DmtxPropWidth);
   height = dmtxDecodeGetProp(dec, DmtxPropHeight);*/

   if(x < 0 || x >= g_dm_proj_img_size || y < 0 || y >= g_dm_proj_img_size)
      return NULL;
   assert(x >= 0 && x < g_dm_proj_img_size && y >= 0 && y < g_dm_proj_img_size);

   return dec->cache + (y * g_dm_proj_img_size + x);
}

/**
 * \brief  Convert fitted Data Matrix region into a decoded message
 * \param  dec
 * \param  reg
 * \param  fix
 * \return Decoded message
 */
extern DmtxMessage *
dmtxDecodeMatrixRegion(DmtxDecode *dec, DmtxRegion *reg)
{
   DmtxMessage *msg;
   //DmtxVector2 topLeft, topRight, bottomLeft, bottomRight;
   //DmtxPixelLoc pxTopLeft, pxTopRight, pxBottomLeft, pxBottomRight;

   msg = dmtxMessageCreate(reg->sizeIdx, DmtxFormatMatrix);
   if(msg == NULL)
      return NULL;

   if(PopulateArrayFromMatrix(dec, reg, msg) != DmtxPass) {
      //dmtxMessageDestroy(&msg);
      return NULL;
   }

   /* maybe place remaining logic into new dmtxDecodePopulatedArray()
      function so other people can pass in their own arrays */

   ModulePlacementEcc200(msg->array, msg->code,
         reg->sizeIdx, DmtxModuleOnRed | DmtxModuleOnGreen | DmtxModuleOnBlue);

   if(RsDecode(msg->code, reg->sizeIdx) == DmtxFail)
   {
      //dmtxMessageDestroy(&msg);
      return NULL;
   }

   /*topLeft.X = bottomLeft.X = topLeft.Y = topRight.Y = -0.1;
   topRight.X = bottomRight.X = bottomLeft.Y = bottomRight.Y = 1.1;

   dmtxMatrix3VMultiplyBy(&topLeft, reg->fit2raw);
   dmtxMatrix3VMultiplyBy(&topRight, reg->fit2raw);
   dmtxMatrix3VMultiplyBy(&bottomLeft, reg->fit2raw);
   dmtxMatrix3VMultiplyBy(&bottomRight, reg->fit2raw);

   pxTopLeft.X = (int)(0.5 + topLeft.X);
   pxTopLeft.Y = (int)(0.5 + topLeft.Y);
   pxBottomLeft.X = (int)(0.5 + bottomLeft.X);
   pxBottomLeft.Y = (int)(0.5 + bottomLeft.Y);
   pxTopRight.X = (int)(0.5 + topRight.X);
   pxTopRight.Y = (int)(0.5 + topRight.Y);
   pxBottomRight.X = (int)(0.5 + bottomRight.X);
   pxBottomRight.Y = (int)(0.5 + bottomRight.Y);*/

   //CacheFillQuad(dec, pxTopLeft, pxTopRight, pxBottomRight, pxBottomLeft);

   DecodeDataStream(msg, reg->sizeIdx, NULL);

   return msg;
}

/**
 * \brief  Increment counters used to determine module values
 * \param  img
 * \param  reg
 * \param  tally
 * \param  xOrigin
 * \param  yOrigin
 * \param  mapWidth
 * \param  mapHeight
 * \param  dir
 * \return void
 */
/*static*/ void
TallyModuleJumps(DmtxDecode *dec, DmtxRegion *reg, int tally[][24], int xOrigin, int yOrigin, int mapWidth, int mapHeight, DmtxDirection dir)
{
   int extent, weight;
   int travelStep;
   int symbolRow, symbolCol;
   int mapRow, mapCol;
   int lineStart, lineStop;
   int travelStart, travelStop;
   int *line, *travel;
   int jumpThreshold;
   int darkOnLight;
   int color;
   int statusPrev, statusModule;
   int tPrev, tModule;

   assert(dir == DmtxDirUp || dir == DmtxDirLeft || dir == DmtxDirDown || dir == DmtxDirRight);

   travelStep = (dir == DmtxDirUp || dir == DmtxDirRight) ? 1 : -1;

   /* Abstract row and column progress using pointers to allow grid
      traversal in all 4 directions using same logic */

   if((dir & DmtxDirHorizontal) != 0x00) {
      line = &symbolRow;
      travel = &symbolCol;
      extent = mapWidth;
      lineStart = yOrigin;
      lineStop = yOrigin + mapHeight;
      travelStart = (travelStep == 1) ? xOrigin - 1 : xOrigin + mapWidth;
      travelStop = (travelStep == 1) ? xOrigin + mapWidth : xOrigin - 1;
   }
   else {
      assert(dir & DmtxDirVertical);
      line = &symbolCol;
      travel = &symbolRow;
      extent = mapHeight;
      lineStart = xOrigin;
      lineStop = xOrigin + mapWidth;
      travelStart = (travelStep == 1) ? yOrigin - 1: yOrigin + mapHeight;
      travelStop = (travelStep == 1) ? yOrigin + mapHeight : yOrigin - 1;
   }


   darkOnLight = (reg->offColor > reg->onColor);
   jumpThreshold = abs(_spint(0.4 * (reg->offColor - reg->onColor)));

   assert(jumpThreshold >= 0);

   for(*line = lineStart; *line < lineStop; (*line)++) {

      /* Capture tModule for each leading border module as normal but
         decide status based on predictable barcode border pattern */

      *travel = travelStart;
      color = ReadModuleColor(dec, reg, symbolRow, symbolCol, reg->sizeIdx);
      tModule = (darkOnLight) ? reg->offColor - color : color - reg->offColor;

      statusModule = (travelStep == 1 || (*line & 0x01) == 0) ? DmtxModuleOnRGB : DmtxModuleOff;

      weight = extent;

      while((*travel += travelStep) != travelStop) {

         tPrev = tModule;
         statusPrev = statusModule;

         /* For normal data-bearing modules capture color and decide
            module status based on comparison to previous "known" module */

         color = ReadModuleColor(dec, reg, symbolRow, symbolCol, reg->sizeIdx);
         tModule = (darkOnLight) ? reg->offColor - color : color - reg->offColor;

         if(statusPrev == DmtxModuleOnRGB) {
            if(tModule < tPrev - jumpThreshold)
               statusModule = DmtxModuleOff;
            else
               statusModule = DmtxModuleOnRGB;
         }
         else if(statusPrev == DmtxModuleOff) {
            if(tModule > tPrev + jumpThreshold)
               statusModule = DmtxModuleOnRGB;
            else
               statusModule = DmtxModuleOff;
         }

         mapRow = symbolRow - yOrigin;
         mapCol = symbolCol - xOrigin;
         assert(mapRow < 24 && mapCol < 24);

         if(statusModule == DmtxModuleOnRGB)
            tally[mapRow][mapCol] += (2 * weight);

         weight--;
      }

      assert(weight == 0);
   }
}

/**
 * \brief  Populate array with codeword values based on module colors
 * \param  msg
 * \param  img
 * \param  reg
 * \return DmtxPass | DmtxFail
 */
/*static*/ DmtxPassFail
PopulateArrayFromMatrix(DmtxDecode *dec, DmtxRegion *reg, DmtxMessage *msg)
{
   int weightFactor;
   int mapWidth, mapHeight;
   int xRegionTotal, yRegionTotal;
   int xRegionCount, yRegionCount;
   int xOrigin, yOrigin;
   int mapCol, mapRow;
   int colTmp, rowTmp, idx;
   int tally[24][24]; /* Large enough to map largest single region */

/* memset(msg->array, 0x00, msg->arraySize); */

   /* Capture number of regions present in barcode */
   xRegionTotal = dmtxGetSymbolAttribute(DmtxSymAttribHorizDataRegions, reg->sizeIdx);
   yRegionTotal = dmtxGetSymbolAttribute(DmtxSymAttribVertDataRegions, reg->sizeIdx);

   /* Capture region dimensions (not including border modules) */
   mapWidth = dmtxGetSymbolAttribute(DmtxSymAttribDataRegionCols, reg->sizeIdx);
   mapHeight = dmtxGetSymbolAttribute(DmtxSymAttribDataRegionRows, reg->sizeIdx);

   weightFactor = 2 * (mapHeight + mapWidth + 2);
   assert(weightFactor > 0);

   /* Tally module changes for each region in each direction */
   for(yRegionCount = 0; yRegionCount < yRegionTotal; yRegionCount++) {

      /* Y location of mapping region origin in symbol coordinates */
      yOrigin = yRegionCount * (mapHeight + 2) + 1;

      for(xRegionCount = 0; xRegionCount < xRegionTotal; xRegionCount++) {

         /* X location of mapping region origin in symbol coordinates */
         xOrigin = xRegionCount * (mapWidth + 2) + 1;

         memset(tally, 0x00, 24 * 24 * sizeof(int));
         TallyModuleJumps(dec, reg, tally, xOrigin, yOrigin, mapWidth, mapHeight, DmtxDirUp);
         TallyModuleJumps(dec, reg, tally, xOrigin, yOrigin, mapWidth, mapHeight, DmtxDirLeft);
         TallyModuleJumps(dec, reg, tally, xOrigin, yOrigin, mapWidth, mapHeight, DmtxDirDown);
         TallyModuleJumps(dec, reg, tally, xOrigin, yOrigin, mapWidth, mapHeight, DmtxDirRight);

         /* Decide module status based on final tallies */
         for(mapRow = 0; mapRow < mapHeight; mapRow++) {
            for(mapCol = 0; mapCol < mapWidth; mapCol++) {

               rowTmp = (yRegionCount * mapHeight) + mapRow;
               rowTmp = yRegionTotal * mapHeight - rowTmp - 1;
               colTmp = (xRegionCount * mapWidth) + mapCol;
               idx = (rowTmp * xRegionTotal * mapWidth) + colTmp;

               if(2*tally[mapRow][mapCol] >= weightFactor)
                  msg->array[idx] = DmtxModuleOnRGB;
               else
                  msg->array[idx] = DmtxModuleOff;

               msg->array[idx] |= DmtxModuleAssigned;
            }
         }
      }
   }

   return DmtxPass;
}
