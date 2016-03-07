
#ifndef __DMUTILS_H__
#define __DMUTILS_H__

#include "libdmtx/dmtx.h"

#define VARIANCE_YPART_THRESHOLD	1
#define LEAST_SQUARE_LEAST_CNT		10

typedef struct _dmLine
{
	float k;
	float b;
	uint16_t ls_flag;
}dmLine;

typedef struct _dmLinePoints
{
	DmtxPixelLoc line_point[LINE_POINT_NUM];
	uint16_t line_start;
	uint16_t line_end;
	uint16_t ls_flag;	//0表示x与y不互换，1表示互换
	uint16_t line_flag;	//0表示bottom,1表示left
}dmLinePoints;

#ifdef __cplusplus
extern "C" {
#endif

/** 直线拟合
 *
 * 利用最小二乘法进行直线拟合，并获得相关系数
 * @param[in]	p 		边缘点集
 * @param[in] 	count 	点集的大小
 * @param[out] 	b		拟合直线的截距
 * @param[out]	k		拟合直线的斜率
 * @return				判断点集组成直线的相关系数
 */
float LeastSquare(DmtxPixelLocf* p, uint16_t count, float* b, float* k, int ls_flag);

int16_t get_otsu_thresh(const uint8_t* in_img, uint16_t total);

void IMG_histogram_8(const uint8_t* img, int16_t size, int16_t accumulate, int16_t* t_hist, int16_t* hist);

void IMG_histogram_8_discrete(const uint8_t* img, uint16_t* hist);

#ifdef __cplusplus
}
#endif

#endif
