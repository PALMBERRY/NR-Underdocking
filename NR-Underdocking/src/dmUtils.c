#include "dmUtils.h"
#ifdef OMAPL138
#include <img_histogram_8.h>
#endif

//最强二乘法参考点至少需求LEAST_SQUARE_LEAST_CNT个
float LeastSquare(DmtxPixelLocf* p, uint16_t count, float* b, float* k, int ls_flag)
{
	float sum_xy = 0, sum_x = 0, sum_y = 0, sum_x2 = 0, res, numerator, denominator_xpart, denominator_ypart, sum_y2 = 0, variance_ypart;
	float tmp_row, tmp_col;
	uint16_t i;

	//float c, d, e;

#pragma MUST_ITERATE(LEAST_SQUARE_LEAST_CNT)
	for(i = 0;i < count;i++)
	{
		/*tmp_row = _spint(p[i].Y);
		tmp_col = _spint(p[i].X);*/
		tmp_row = p[i].Y;
		tmp_col = p[i].X;
		sum_xy += tmp_row*tmp_col;
		sum_x += tmp_col;
		sum_y += tmp_row;
		sum_x2 += tmp_col*tmp_col;

		sum_y2 += tmp_row*tmp_row;
	}

	/*c = count*sum_xy;
	d = sum_x*sum_y;
	e = c-d;

	numerator = 0;*/
	numerator = count*sum_xy-sum_x*sum_y;
	denominator_xpart = count*sum_x2-sum_x*sum_x;
	denominator_ypart = count*sum_y2-sum_y*sum_y;

	if(ls_flag == 0)	//x,y不互换
	{
		//variance_ypart = denominator_ypart*(_rcpsp((float)(count*count)));
		variance_ypart = denominator_ypart*_rcpsp((float)(count*count));
		//*b = numerator*_rcpsp(denominator_xpart);
		*k = numerator*_rcpsp(denominator_xpart);
		//*a = (sum_y-(*b)*sum_x)*(_rcpsp((float)count));
		*b = (sum_y-(*k)*sum_x)*_rcpsp((float)count);
	}
	else
	{
		variance_ypart = denominator_xpart*_rcpsp((float)(count*count));
		*k = numerator*_rcpsp(denominator_ypart);
		*b = (sum_x-(*k)*sum_y)*_rcpsp((float)count);
	}

	if(variance_ypart < VARIANCE_YPART_THRESHOLD)
		return 1;
	else
	{
		//res = numerator*_rsqrsp(denominator_xpart*denominator_ypart);
		res = numerator*_rsqrsp(denominator_xpart*denominator_ypart);
		//return _fabsf(res);
		return _fabsf(res);
	}
}

short g_hist[256];
//short g_t_hist[1024];
//total = 15*15
int16_t get_otsu_thresh(const uint8_t* in_img, uint16_t total)
{
	/* declare block */
	int32_t sum = 0, sumB = 0;
	int16_t i;
	int16_t start_i, stop_i;
	int32_t wF = 0, wB = 0;
	float max = 0.0;
	float mB, mF;
	float between = 0.0;
	int16_t threshold;
	
	/* buffers must be initialized to zero */

	//memset(g_t_hist, 0, sizeof(g_t_hist));
	memset(g_hist, 0, sizeof(g_hist));
	
	/* use IMGLIB to compute image histogram */
	IMG_histogram_8(in_img, total, 1, NULL, g_hist);

	for (i = 0; i < 256; ++i)
	{
		sum += i*g_hist[i];
	}
	
	/* look for start_i and stop_i */
	for (i = 0;  i < 256; ++i)
	{
		if (g_hist[i] > 0)
		{
			start_i = i;
			break;
		}	
	}
	
	for (i = 255; i >=0; --i)
	{
		if (g_hist[i] > 0)
		{
			stop_i = i;
			break;
		}
	}
	
	for (i = start_i; i < stop_i; ++i)
	{
		wB += g_hist[i];
		wF = total - wB;
		sumB += i*g_hist[i];
		mB = sumB*_rcpsp((float)wB);
		mF = (sum - sumB)*_rcpsp((float)wF);
		between = wB*wF*(mB-mF)*(mB-mF);
		if (between > max)
		{
			threshold = i;
			max = between;
		}
	}
	return threshold;
}

void IMG_histogram_8(const uint8_t* img, int16_t size, int16_t accumulate,
					 int16_t* t_hist, int16_t* hist)
{
	int16_t i, j;
	int cur_index;

	for(i = 0;i < 15;i++)
	{
		cur_index = i*60;
		for(j = 0;j < 15;j++)
		{
			hist[img[cur_index]]++;
			cur_index += 2;
		}
	}
}

void IMG_histogram_8_discrete(const uint8_t* img, uint16_t* hist)
{
	int16_t i, j;
	int cur_index;

	for(i = 0;i < (GRID_SIZE>>1);i++)
	{
		cur_index = i*2*SRC_IMG_X;
		for(j = 0;j < (GRID_SIZE>>1);j++)
		{
			hist[img[cur_index]>>3]++;
			cur_index += 2;
		}
	}
}
