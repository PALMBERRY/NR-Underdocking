/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtxregion.c
 * \brief Detect barcode regions
 */
#include "libdmtx/dmtx.h"
#include "libdmtx/dmtxstatic.h"

#include "dmUtils.h"
#include "project.h"
#include "DMA_configuration.h"

#define DMTX_HOUGH_RES 180

#ifdef WATCH_MODE
unsigned char src_watch[SRC_IMG_Y*SRC_IMG_X];
unsigned char img_watch[DM_PROJ_IMG_SIZE * DM_PROJ_IMG_SIZE];
unsigned char barcode_res[12*12];
#endif

DmtxRegion g_reg;

#ifdef CIRCLE_LINE_CALIB
DmtxRegion g_reg_calib[4];
#endif

#pragma DATA_ALIGN(g_outline, 8)
DmtxPixelLoc g_outline[OUTLINE_POINT_NUM];

#pragma DATA_ALIGN(ground_thres_point, 8)
uint8_t ground_thres_point[2][(SRC_IMG_Y-WHITE_LENGTH_2)/DM_ONE_SIZE+1];

#pragma DATA_ALIGN(avg_point, 8)
uint8_t avg_point[WHITE_LENGTH_2*WHITE_LENGTH_2];

extern Inter_Linear_Table dm_proj_table[DOWN_PROJ_IMG_SIZE*DOWN_PROJ_IMG_SIZE];

int g_camera_id;

uint8_t g_loc_map[16*16];

uint8_t dmAvg(uint8_t* cur_point)
{
	int i, j, total = 0;

	for(i = 0;i < WHITE_LENGTH_2;i++)
	{
		for(j = 0;j < WHITE_LENGTH_2;j++)
		{
			total += *cur_point;
			cur_point++;
		}
		cur_point += SRC_IMG_X-WHITE_LENGTH_2;
	}

	return total/(WHITE_LENGTH_2*WHITE_LENGTH_2);
}

uint8_t dmAvg_DSP(uint8_t* cur_point)
{
	int i, total = 0;

	_nassert((int)(cur_point)%8 == 0);
#pragma MUST_ITERATE(WHITE_LENGTH_2*WHITE_LENGTH_2, , WHITE_LENGTH_2*WHITE_LENGTH_2)
	for(i = 0;i < WHITE_LENGTH_2*WHITE_LENGTH_2;i++)
	{
		total += *cur_point;
		cur_point++;
	}

	return total/(WHITE_LENGTH_2*WHITE_LENGTH_2);
}

extern DmtxPassFail
dmLoc_DSP(uint8_t* srcImg, DmtxVector2* oriLoc, volatile int16_t* new_pic_cnt)
{
	int i, j, k, l, start_row, end_row, start_col, end_col, through_total, through_diff1, through_diff2, through_avg, tmp_row, tmp_col, cur_ground_p;
	//uint16_t black_start, black_length;
	uint16_t white_cnt, black_cnt;
	uint8_t avg;
	uint8_t through_flag, through_index;	//through_flag为1表示在白，为0表示在黑
	DmtxPixelLoc through_point[MAX_THROUGH_POINT], tmp_point;
	DmtxPixelLoc* p_through_point;
	uint8_t* current_point;
	uint8_t* test_point;
	uint8_t* search_img;

	search_img = (uint8_t*)dm_proj_table;

#ifdef WATCH_MODE
	memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif

#ifdef USE_DMA
		configureEDMA3CC(g_PaRAM5, 5, srcImg, ground_thres_point[0], 1, (SRC_IMG_Y-WHITE_LENGTH_2)/DM_ONE_SIZE+1, DM_ONE_SIZE, 1, 0, 0, 1, 0xFFFF);
		startEDMA3(5);

		waitForEDMA3(5);
#else
		for(j = 0;j < (SRC_IMG_Y-WHITE_LENGTH_2)/DM_ONE_SIZE+1;j++)
			ground_thres_point[0][j] = srcImg[j*DM_ONE_SIZE];
#endif

	cur_ground_p = 0;

	for(i = 0;i < SRC_IMG_Y-WHITE_LENGTH_2+1;i+=DM_ONE_SIZE)
	{
		cur_ground_p = (i/DM_ONE_SIZE)&0x1;
		current_point = srcImg+i*SRC_IMG_X;
#ifdef USE_DMA
		configureEDMA3CC(g_PaRAM5, 5, current_point+(DM_ONE_SIZE*SRC_IMG_X), ground_thres_point[1-cur_ground_p], 1, (SRC_IMG_Y-WHITE_LENGTH_2)/DM_ONE_SIZE+1, DM_ONE_SIZE, 1, 0, 0, 1, 0xFFFF);
		startEDMA3(5);
#else
		for(j = 0;j < (SRC_IMG_Y-WHITE_LENGTH_2)/DM_ONE_SIZE+1;j++)
			ground_thres_point[1-cur_ground_p][j] = current_point[j*DM_ONE_SIZE+DM_ONE_SIZE*SRC_IMG_X];
#endif
		for(j = 0;j < (SRC_IMG_Y-WHITE_LENGTH_2)/DM_ONE_SIZE+1;j++)
		{
			if(ground_thres_point[cur_ground_p][j] > GROUND_THRES)
			{
#ifdef WIN32
				if(*new_pic_cnt == 2)
#else
				if(*new_pic_cnt > 0)
#endif
				{
					return DmtxFail;
				}

#ifdef USE_DMA
				configureEDMA3CC(g_PaRAM3, 3, current_point+j*DM_ONE_SIZE, avg_point, WHITE_LENGTH_2, WHITE_LENGTH_2, SRC_IMG_X, WHITE_LENGTH_2, 0, 0, 1, 0xFFFF);
				startEDMA3(3);

				waitForEDMA3(3);
#else
				for(k = 0;k < WHITE_LENGTH_2;k++)
				{
					for(l = 0;l < WHITE_LENGTH_2;l++)
					{
						avg_point[k*WHITE_LENGTH_2+l] = srcImg[(i+k)*SRC_IMG_X+j*DM_ONE_SIZE+l];
					}
				}
#endif
				avg = dmAvg_DSP(avg_point);
				if(avg > GROUND_THRES)
				{
					start_row = i > SEARCH_SIZE ? (i-SEARCH_SIZE):0;
					end_row = i < SRC_IMG_Y-SEARCH_SIZE ? (i+SEARCH_SIZE):SRC_IMG_X;

					end_row -= start_row;
					tmp_row = end_row/(DM_ONE_SIZE*2)+1;

					tmp_col = j*DM_ONE_SIZE;
					start_col = tmp_col > SEARCH_SIZE ? (tmp_col-SEARCH_SIZE):0;
					end_col = tmp_col < SRC_IMG_X-SEARCH_SIZE ? (tmp_col+SEARCH_SIZE):SRC_IMG_X;

					end_col -= start_col;
#ifdef USE_DMA
					configureEDMA3CC(g_PaRAM3, 3, &srcImg[start_row*SRC_IMG_X+start_col], search_img, end_col, tmp_row, SRC_IMG_X*DM_ONE_SIZE*2, end_col, 0, 0, 1, 0xFFFF);
					startEDMA3(3);

					waitForEDMA3(3);
#else
					for(k = 0;k < tmp_row;k++)
					{
						for(l = 0;l < end_col;l++)
						{
							search_img[k*end_col+l] = srcImg[(k*DM_ONE_SIZE*2+start_row)*SRC_IMG_X+l+start_col];
						}
					}
#endif

					through_index = 0;
					test_point = search_img;
					for(k = 0;k < tmp_row;k++)
					{
						//test_point = k*end_col;
						//through_cnt[through_index] = 0;
						through_flag = 1;
						white_cnt = 0;
						black_cnt = 0;

						for(l = 0;l < end_col;l++)
						{
							if(*test_point < DM_THRES)					//为黑
							{
								if(through_flag == 1)
								{
									through_flag = 0;							//在黑
									black_cnt = 1;

									if(white_cnt > (DM_ONE_SIZE-3) && through_index < MAX_THROUGH_POINT)				//白色间隔至少为4
									{
										through_point[through_index].X = l+start_col;
										through_point[through_index].Y = k*(DM_ONE_SIZE*2)+start_row;

										through_index++;
									}
								}
								else
								{
									black_cnt++;
								}
							}
							else												//为白
							{
								if(through_flag == 0)
								{
									through_flag = 1;							//为白
									white_cnt = 1;

									if(through_index%2 == 1)
									{
										if(black_cnt > (DM_ONE_SIZE-3) && black_cnt < (DM_ONE_SIZE*11))		//黑点长度4到65范围
										{
											through_point[through_index].X = l+start_col-1;
											through_point[through_index].Y = k*(DM_ONE_SIZE*2)+start_row;

#ifdef WATCH_MODE
											src_watch[through_point[through_index].Y*SRC_IMG_X+through_point[through_index].X] = 255;
											src_watch[through_point[through_index-1].Y*SRC_IMG_X+through_point[through_index-1].X] = 255;
#endif

											through_index++;
										}
										else
										{
											through_index--;
										}
									}
								}
								else
								{
									white_cnt++;
								}

								//black_length = l - black_start;
								//if(black_length > (DM_ONE_SIZE/2+1) && black_length < (DM_ONE_SIZE*11))		//黑点长度4到65范围
								//{
								//	through_cnt[through_index]++;
								//	through_end[through_index] = l;
								//}
								//through_flag = 0;
							}
							test_point++;
						}
						if(through_index%2 == 1)
							through_index--;
					}

					//根据x坐标删点
					if(through_index > LEAST_POINT)		//剩余点大于16
					{
						p_through_point = through_point;

						for(k = 0;k < through_index-1;k++)
							for(l = k+1;l < through_index;l++)
							{
								if(p_through_point[k].X > p_through_point[l].X)
								{
									tmp_point = p_through_point[k];
									p_through_point[k] = p_through_point[l];
									p_through_point[l] = tmp_point;
								}
							}

						through_total = 0;
#pragma MUST_ITERATE(LEAST_POINT)
						for(k = 0;k < through_index;k++)
							through_total += p_through_point[k].X;

						while(through_index > LEAST_POINT)
						{
							through_avg = through_total/through_index;

							through_diff1 = abs(p_through_point[0].X-through_avg);
							through_diff2 = abs(p_through_point[through_index-1].X-through_avg);

							if(through_diff1 > through_diff2)
							{
								if(through_diff1 > MAX_THROUGH_DIFF)		//
								{
									through_total -= p_through_point[0].X;
									p_through_point++;
									through_index--;
								}
								else
									break;
							}
							else
							{
								if(through_diff2 > MAX_THROUGH_DIFF)		//
								{
									through_total -= p_through_point[through_index-1].X;
									through_index--;
								}
								else
									break;
							}
						}

#ifdef WATCH_MODE
						memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
						for(k = 0;k < through_index;k++)
							src_watch[p_through_point[k].Y*SRC_IMG_X+p_through_point[k].X] = 255;
#endif

						//根据y坐标删点
						if(through_index > LEAST_POINT)		//剩余点大于16
						{
							for(k = 0;k < through_index-1;k++)
								for(l = k+1;l < through_index;l++)
								{
									if(p_through_point[k].Y > p_through_point[l].Y)
									{
										tmp_point = p_through_point[k];
										p_through_point[k] = p_through_point[l];
										p_through_point[l] = tmp_point;
									}
								}

							through_total = 0;
#pragma MUST_ITERATE(LEAST_POINT)
							for(k = 0;k < through_index;k++)
								through_total += p_through_point[k].Y;

							while(through_index > LEAST_POINT)
							{
								through_avg = through_total/through_index;

								through_diff1 = abs(p_through_point[0].Y-through_avg);
								through_diff2 = abs(p_through_point[through_index-1].Y-through_avg);

								if(through_diff1 > through_diff2)
								{
									if(through_diff1 > MAX_THROUGH_DIFF)		//
									{
										through_total -= p_through_point[0].Y;
										p_through_point++;
										through_index--;
									}
									else
										break;
								}
								else
								{
									if(through_diff2 > MAX_THROUGH_DIFF)		//
									{
										through_total -= p_through_point[through_index-1].Y;
										through_index--;
									}
									else
										break;
								}
							}

#ifdef WATCH_MODE
							memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
							for(k = 0;k < through_index;k++)
								src_watch[p_through_point[k].Y*SRC_IMG_X+p_through_point[k].X] = 255;
#endif

							if(through_index > LEAST_POINT)		//剩余点大于16
							{
								oriLoc->Y = through_total*_rcpdp((double)through_index);

								through_total = 0;
#pragma MUST_ITERATE(LEAST_POINT)
								for(k = 0;k < through_index;k++)
									through_total += p_through_point[k].X;
								oriLoc->X = through_total*_rcpdp((double)through_index);

#ifdef WATCH_MODE
								memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif

								return DmtxPass;
							}
						}
					}
#ifdef WATCH_MODE
					memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif
				}
			}
			//cur_point += DM_ONE_SIZE;
		}
#ifdef USE_DMA
		waitForEDMA3(5);
#endif
	}

	return DmtxFail;
}

extern DmtxPassFail
dmLoc(uint8_t* srcImg, DmtxVector2* oriLoc)
{
	int i, j, k, l, cur_point, test_point, start_row, end_row, start_col, end_col, through_total, through_diff1, through_diff2, through_avg;
	//uint16_t black_start, black_length;
	uint16_t white_cnt, black_cnt;
	uint8_t avg;
	uint8_t through_flag, through_index;	//through_flag为1表示在白，为0表示在黑
	DmtxPixelLoc through_point[MAX_THROUGH_POINT], tmp_point;
	DmtxPixelLoc* p_through_point;

#ifdef WATCH_MODE
	memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif
	
	for(i = 0;i < SRC_IMG_Y-WHITE_LENGTH_2+1;i+=DM_ONE_SIZE)
	{
		cur_point = i*SRC_IMG_X;
		for(j = 0;j < SRC_IMG_X-WHITE_LENGTH_2+1;j+=DM_ONE_SIZE)
		{
			if(srcImg[cur_point] > GROUND_THRES)
			{
				avg = dmAvg(&srcImg[cur_point]);
				if(avg > GROUND_THRES)
				{
					start_row = i > SEARCH_SIZE ? (i-SEARCH_SIZE):0;
					end_row = i < SRC_IMG_Y-SEARCH_SIZE ? (i+SEARCH_SIZE):SRC_IMG_X; 

					start_col = j > SEARCH_SIZE ? (j-SEARCH_SIZE):0;
					end_col = j < SRC_IMG_X-SEARCH_SIZE ? (j+SEARCH_SIZE):SRC_IMG_X; 
					
					through_index = 0;
					for(k = start_row;k < end_row;k += DM_ONE_SIZE*2)
					{
						test_point = k*SRC_IMG_X+start_col;
						//through_cnt[through_index] = 0;
						through_flag = 1;
						white_cnt = 0;
						black_cnt = 0;

						for(l = start_col;l < end_col;l++)
						{
							if(srcImg[test_point] < DM_THRES)					//为黑
							{
								if(through_flag == 1)
								{
									through_flag = 0;							//在黑
									black_cnt = 1;

									if(white_cnt > (DM_ONE_SIZE-3) && through_index < MAX_THROUGH_POINT)				//白色间隔至少为4
									{
										through_point[through_index].X = l;
										through_point[through_index].Y = k;

										through_index++;
									}
								}
								else
								{
									black_cnt++;
								}								
							}
							else												//为白
							{
								if(through_flag == 0)
								{
									through_flag = 1;							//为白
									white_cnt = 1;

									if(through_index%2 == 1)
									{
										if(black_cnt > (DM_ONE_SIZE-3) && black_cnt < (DM_ONE_SIZE*11))		//黑点长度4到55范围
										{
											through_point[through_index].X = l-1;
											through_point[through_index].Y = k;

#ifdef WATCH_MODE
											src_watch[through_point[through_index].Y*SRC_IMG_X+through_point[through_index].X] = 255;
											src_watch[through_point[through_index-1].Y*SRC_IMG_X+through_point[through_index-1].X] = 255;
#endif

											through_index++;
										}
										else
										{
											through_index--;
										}
									}
								}
								else
								{
									white_cnt++;
								}								

								//black_length = l - black_start;
								//if(black_length > (DM_ONE_SIZE/2+1) && black_length < (DM_ONE_SIZE*11))		//黑点长度4到55范围
								//{
								//	through_cnt[through_index]++;
								//	through_end[through_index] = l;
								//}
								//through_flag = 0;
							}
							test_point++;
						}
						if(through_index%2 == 1)
							through_index--;
					}

					//根据x坐标删点
					if(through_index > LEAST_POINT)		//剩余点大于20
					{
						p_through_point = through_point;

						for(k = 0;k < through_index-1;k++)
							for(l = k+1;l < through_index;l++)
							{								
								if(p_through_point[k].X > p_through_point[l].X)
								{
									tmp_point = p_through_point[k];
									p_through_point[k] = p_through_point[l];
									p_through_point[l] = tmp_point;
								}
							}

						through_total = 0;
#pragma MUST_ITERATE(LEAST_POINT)
						for(k = 0;k < through_index;k++)
							through_total += p_through_point[k].X;
						
						while(through_index > LEAST_POINT)
						{							
							through_avg = through_total/through_index;

							through_diff1 = abs(p_through_point[0].X-through_avg);
							through_diff2 = abs(p_through_point[through_index-1].X-through_avg);

							if(through_diff1 > through_diff2)
							{
								if(through_diff1 > MAX_THROUGH_DIFF)		//
								{
									through_total -= p_through_point[0].X;
									p_through_point++;
									through_index--;
								}
								else
									break;
							}
							else
							{
								if(through_diff2 > MAX_THROUGH_DIFF)		//
								{
									through_total -= p_through_point[through_index-1].X;
									through_index--;
								}
								else
									break;
							}
						}

#ifdef WATCH_MODE
						memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
						for(k = 0;k < through_index;k++)
							src_watch[p_through_point[k].Y*SRC_IMG_X+p_through_point[k].X] = 255;
#endif

						//根据y坐标删点
						if(through_index > LEAST_POINT)		//剩余点大于16
						{
							for(k = 0;k < through_index-1;k++)
								for(l = k+1;l < through_index;l++)
								{								
									if(p_through_point[k].Y > p_through_point[l].Y)
									{
										tmp_point = p_through_point[k];
										p_through_point[k] = p_through_point[l];
										p_through_point[l] = tmp_point;
									}
								}

							through_total = 0;
#pragma MUST_ITERATE(LEAST_POINT)
							for(k = 0;k < through_index;k++)
								through_total += p_through_point[k].Y;

							while(through_index > LEAST_POINT)
							{
								through_avg = through_total/through_index;

								through_diff1 = abs(p_through_point[0].Y-through_avg);
								through_diff2 = abs(p_through_point[through_index-1].Y-through_avg);

								if(through_diff1 > through_diff2)
								{
									if(through_diff1 > MAX_THROUGH_DIFF)		//
									{
										through_total -= p_through_point[0].Y;
										p_through_point++;
										through_index--;
									}
									else
										break;
								}
								else
								{
									if(through_diff2 > MAX_THROUGH_DIFF)		//
									{
										through_total -= p_through_point[through_index-1].Y;
										through_index--;
									}
									else
										break;
								}
							}

#ifdef WATCH_MODE
							memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
							for(k = 0;k < through_index;k++)
								src_watch[p_through_point[k].Y*SRC_IMG_X+p_through_point[k].X] = 255;
#endif

							if(through_index > LEAST_POINT)		//剩余点大于16
							{
								oriLoc->Y = through_total*_rcpdp((double)through_index);

								through_total = 0;
#pragma MUST_ITERATE(LEAST_POINT)
								for(k = 0;k < through_index;k++)
									through_total += p_through_point[k].X;
								oriLoc->X = through_total*_rcpdp((double)through_index);			

#ifdef WATCH_MODE
								memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif

								return DmtxPass;
							}
						}
					}
#ifdef WATCH_MODE
					memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif
				}
			}
			cur_point += DM_ONE_SIZE;
		}
	}

	return DmtxFail;
}

extern uint16_t calc_ratio(uint8_t* cur_point, int index)
{
	uint16_t hist[32];
	uint16_t sum;
	uint16_t k, min_k, max_k;

	//0表示未计算，255表示已计算
	if(g_loc_map[index] == 255)
		return 0;

	g_loc_map[index] = 255;

	memset(hist, 0, 64);
	IMG_histogram_8_discrete(cur_point, hist);

	sum = hist[0]+hist[1]+hist[2]+hist[3]+hist[4];
	k = MIN_MIN_K;
	while(sum < 23)
	{
		k++;
		sum += hist[k];
	}
	min_k = k;

	sum = hist[31];
	k = 31;
	while(sum < 23)
	{
		k--;
		sum += hist[k];
	}
	max_k = k;

	if(max_k > 3*min_k)
	{
		if(max_k > MIN_MAX_K)		//满足条件，但最大值有问题，可疑！
			return 1;
		else
			return 2;
	}
	else if(max_k > MAX_MAX_K)		//不满足条件，但最大值不错，可疑！
	{
		return 2;
	}
	else
	{
		return 0;
	}
}

extern DmtxPassFail
dmLoc_discrete(uint8_t* srcImg, DmtxVector2* oriLoc)
{
	int i, j, k, l, cur_row, cur_point;// test_point, start_row, end_row, start_col, end_col, through_total, through_diff1, through_diff2, through_avg;
	//uint16_t black_start, black_length;
	//uint16_t white_cnt, black_cnt;
	//uint8_t avg;
	//uint8_t through_flag, through_index;	//through_flag为1表示在白，为0表示在黑
	//DmtxPixelLoc through_point[MAX_THROUGH_POINT], tmp_point;
	//DmtxPixelLoc* p_through_point;
	
	//int adjacency_point[4];
	uint16_t bool_k, stack_cnt, grid_cnt;
	DmtxPixelLoc stack_p[256];
	DmtxPixelLoc grid_p[256];
	DmtxPixelLoc tmp_point;
	DmtxPixelLoc* p_grid_point;
	uint16_t stack_flag[256];
	int tmp_x, tmp_y, tmp_id;
	uint8_t* cur_p;
	int sum_t, avg_t, diff1, diff2;
	//double grid_cnt_inv;

	uint16_t max_grid_cnt = LEAST_POINT_GAI;

	//DmtxVector2 ori_loc[64];
	//uint16_t ori_loc_cnt[64];
	//uint16_t res_cnt = 0;

	memset(g_loc_map, 0, 16*16);

#ifdef WATCH_MODE
	memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif
	
	for(i = GRID_SIZE;i < SRC_IMG_Y-GRID_SIZE+1;i+=GRID_SIZE*2)
	{
		cur_row = i*SRC_IMG_X;
		for(j = GRID_SIZE;j < SRC_IMG_X-GRID_SIZE+1;j+=GRID_SIZE*2)
		{
			cur_point = cur_row+j;
			tmp_id = (i*GRID_CNT+j)/GRID_SIZE;

			bool_k = calc_ratio(srcImg+cur_point, tmp_id);

#ifdef WATCH_MODE
			for(k = 0;k < GRID_SIZE;k++)
				src_watch[cur_point+k] = 255;

			for(k = 0;k < GRID_SIZE;k++)
				src_watch[cur_point+30*SRC_IMG_X+k] = 255;
#endif

			if(bool_k)
			{
				grid_cnt = 0;				

				stack_p[0].X = j;
				stack_p[0].Y = i;
				stack_flag[0] = bool_k;
				stack_cnt = 1;				

				while(stack_cnt)
				{
					stack_cnt--;
					tmp_x = stack_p[stack_cnt].X;
					tmp_y = stack_p[stack_cnt].Y;

					if(stack_flag[stack_cnt] == 1)
					{
						grid_p[grid_cnt].X = tmp_x;
						grid_p[grid_cnt].Y = tmp_y;
						grid_cnt++;

#ifdef WATCH_MODE
					src_watch[(grid_p[grid_cnt-1].Y-1+(GRID_SIZE>>1))*SRC_IMG_X+grid_p[grid_cnt-1].X+(GRID_SIZE>>1)] = 128;
					src_watch[(grid_p[grid_cnt-1].Y+1+(GRID_SIZE>>1))*SRC_IMG_X+grid_p[grid_cnt-1].X+(GRID_SIZE>>1)] = 128;
					src_watch[(grid_p[grid_cnt-1].Y+(GRID_SIZE>>1))*SRC_IMG_X+grid_p[grid_cnt-1].X+(GRID_SIZE>>1)] = 255;
					src_watch[(grid_p[grid_cnt-1].Y+(GRID_SIZE>>1))*SRC_IMG_X+grid_p[grid_cnt-1].X-1+(GRID_SIZE>>1)] = 128;
					src_watch[(grid_p[grid_cnt-1].Y+(GRID_SIZE>>1))*SRC_IMG_X+grid_p[grid_cnt-1].X+1+(GRID_SIZE>>1)] = 128;
#endif
					}

					cur_point = tmp_y*SRC_IMG_X+tmp_x;
					tmp_id = (tmp_y*GRID_CNT+tmp_x)/GRID_SIZE;

					cur_p = srcImg+cur_point;
					//上
					if(tmp_y > GRID_SIZE-1)
					{
						bool_k = calc_ratio(cur_p-GRID_SIZE*SRC_IMG_X, tmp_id-GRID_CNT);						
						if(bool_k)
						{
							stack_p[stack_cnt].X = tmp_x;
							stack_p[stack_cnt].Y = tmp_y-GRID_SIZE;
							stack_flag[stack_cnt] = bool_k;

							stack_cnt++;
						}
					}

					//下
					if(tmp_y < SRC_IMG_Y-GRID_SIZE)
					{
						bool_k = calc_ratio(cur_p+GRID_SIZE*SRC_IMG_X, tmp_id+GRID_CNT);
						if(bool_k)
						{
							stack_p[stack_cnt].X = tmp_x;
							stack_p[stack_cnt].Y = tmp_y+GRID_SIZE;
							stack_flag[stack_cnt] = bool_k;

							stack_cnt++;
						}
					}

					//左
					if(tmp_x > GRID_SIZE-1)
					{
						bool_k = calc_ratio(cur_p-GRID_SIZE, tmp_id-1);
						if(bool_k)
						{
							stack_p[stack_cnt].X = tmp_x-GRID_SIZE;
							stack_p[stack_cnt].Y = tmp_y;
							stack_flag[stack_cnt] = bool_k;

							stack_cnt++;
						}
					}

					//右
					if(tmp_x < SRC_IMG_X-GRID_SIZE)
					{
						bool_k = calc_ratio(cur_p+GRID_SIZE, tmp_id+1);
						if(bool_k)
						{
							stack_p[stack_cnt].X = tmp_x+GRID_SIZE;
							stack_p[stack_cnt].Y = tmp_y;
							stack_flag[stack_cnt] = bool_k;

							stack_cnt++;
						}
					}
				}			

				/*if(grid_cnt > 3)
				{
					sum_x = 0;
					sum_y = 0;
					for(k = 0;k < grid_cnt;k++)
					{
						sum_x += grid_p[k].X;
						sum_y += grid_p[k].Y;
					}

					grid_cnt_inv = _rcpdp(grid_cnt);
					oriLoc->X = sum_x*grid_cnt_inv;
					oriLoc->Y = sum_y*grid_cnt_inv;

					return DmtxPass;
				}*/

				//根据x坐标删点
				if(grid_cnt > LEAST_POINT_GAI)		//剩余点大于20
				{
					p_grid_point = grid_p;

					for(k = 0;k < grid_cnt-1;k++)
						for(l = k+1;l < grid_cnt;l++)
						{								
							if(p_grid_point[k].X > p_grid_point[l].X)
							{
								tmp_point = p_grid_point[k];
								p_grid_point[k] = p_grid_point[l];
								p_grid_point[l] = tmp_point;
							}
						}

					sum_t = 0;

					for(k = 0;k < grid_cnt;k++)
						sum_t += p_grid_point[k].X;
						
					while(grid_cnt > LEAST_POINT_GAI)
					{							
						avg_t = sum_t/grid_cnt;

						diff1 = abs(p_grid_point[0].X-avg_t);
						diff2 = abs(p_grid_point[grid_cnt-1].X-avg_t);

						if(diff1 > diff2)
						{
							if(diff1 > MAX_DIFF)		//
							{
								sum_t -= p_grid_point[0].X;
								p_grid_point++;
								grid_cnt--;
							}
							else
								break;
						}
						else
						{
							if(diff2 > MAX_DIFF || diff2 == 0)		//
							{
								grid_cnt--;
								sum_t -= p_grid_point[grid_cnt].X;							
							}
							else
								break;
						}
					}

#ifdef WATCH_MODE
					memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
					for(k = 0;k < grid_cnt;k++)
						src_watch[p_grid_point[k].Y*SRC_IMG_X+p_grid_point[k].X] = 128;
#endif

					//根据y坐标删点
					if(grid_cnt > LEAST_POINT_GAI)		//剩余点大于16
					{
						for(k = 0;k < grid_cnt-1;k++)
							for(l = k+1;l < grid_cnt;l++)
							{								
								if(p_grid_point[k].Y > p_grid_point[l].Y)
								{
									tmp_point = p_grid_point[k];
									p_grid_point[k] = p_grid_point[l];
									p_grid_point[l] = tmp_point;
								}
							}

						sum_t = 0;

						for(k = 0;k < grid_cnt;k++)
							sum_t += p_grid_point[k].Y;

						while(grid_cnt > LEAST_POINT_GAI)
						{
							avg_t = sum_t/grid_cnt;

							diff1 = abs(p_grid_point[0].Y-avg_t);
							diff2 = abs(p_grid_point[grid_cnt-1].Y-avg_t);

							if(diff1 > diff2)
							{
								if(diff1 > MAX_DIFF)		//
								{
									sum_t -= p_grid_point[0].Y;
									p_grid_point++;
									grid_cnt--;
								}
								else
									break;
							}
							else
							{
								if(diff2 > MAX_DIFF || diff2 == 0)		//
								{
									grid_cnt--;
									sum_t -= p_grid_point[grid_cnt].Y;									
								}
								else
									break;
							}
						}

#ifdef WATCH_MODE
						memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
						for(k = 0;k < grid_cnt;k++)
							src_watch[p_grid_point[k].Y*SRC_IMG_X+p_grid_point[k].X] = 128;
#endif

						if(grid_cnt > max_grid_cnt)		//剩余点大于16
						{
							max_grid_cnt = grid_cnt;

							oriLoc->Y = sum_t*_rcpdp((double)grid_cnt)+(GRID_SIZE>>1);
							//ori_loc[res_cnt].Y = sum_t*_rcpdp((double)grid_cnt)+(GRID_SIZE>>1); 

							sum_t = 0;
#pragma MUST_ITERATE(LEAST_POINT)
							for(k = 0;k < grid_cnt;k++)
								sum_t += p_grid_point[k].X;
							oriLoc->X = sum_t*_rcpdp((double)grid_cnt)+(GRID_SIZE>>1);			
							//ori_loc[res_cnt].X = sum_t*_rcpdp((double)grid_cnt)+(GRID_SIZE>>1);
							
#ifdef WATCH_MODE
							memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
#endif
							//ori_loc_cnt[res_cnt] = grid_cnt;
							//res_cnt++;
							//return DmtxPass;
						}
					}
				}

//				if(0)
//				{
//					start_row = i > SEARCH_SIZE ? (i-SEARCH_SIZE):0;
//					end_row = i < SRC_IMG_Y-SEARCH_SIZE ? (i+SEARCH_SIZE):SRC_IMG_X; 
//
//					start_col = j > SEARCH_SIZE ? (j-SEARCH_SIZE):0;
//					end_col = j < SRC_IMG_X-SEARCH_SIZE ? (j+SEARCH_SIZE):SRC_IMG_X; 
//					
//					through_index = 0;
//					for(k = start_row;k < end_row;k += DM_ONE_SIZE*2)
//					{
//						test_point = k*SRC_IMG_X+start_col;
//						//through_cnt[through_index] = 0;
//						through_flag = 1;
//						white_cnt = 0;
//						black_cnt = 0;
//
//						for(l = start_col;l < end_col;l++)
//						{
//							if(srcImg[test_point] < DM_THRES)					//为黑
//							{
//								if(through_flag == 1)
//								{
//									through_flag = 0;							//在黑
//									black_cnt = 1;
//
//									if(white_cnt > (DM_ONE_SIZE-3) && through_index < MAX_THROUGH_POINT)				//白色间隔至少为4
//									{
//										through_point[through_index].X = l;
//										through_point[through_index].Y = k;
//
//										through_index++;
//									}
//								}
//								else
//								{
//									black_cnt++;
//								}								
//							}
//							else												//为白
//							{
//								if(through_flag == 0)
//								{
//									through_flag = 1;							//为白
//									white_cnt = 1;
//
//									if(through_index%2 == 1)
//									{
//										if(black_cnt > (DM_ONE_SIZE-3) && black_cnt < (DM_ONE_SIZE*11))		//黑点长度4到55范围
//										{
//											through_point[through_index].X = l-1;
//											through_point[through_index].Y = k;
//
//#ifdef WATCH_MODE
//											src_watch[through_point[through_index].Y*SRC_IMG_X+through_point[through_index].X] = 255;
//											src_watch[through_point[through_index-1].Y*SRC_IMG_X+through_point[through_index-1].X] = 255;
//#endif
//
//											through_index++;
//										}
//										else
//										{
//											through_index--;
//										}
//									}
//								}
//								else
//								{
//									white_cnt++;
//								}								
//
//								//black_length = l - black_start;
//								//if(black_length > (DM_ONE_SIZE/2+1) && black_length < (DM_ONE_SIZE*11))		//黑点长度4到55范围
//								//{
//								//	through_cnt[through_index]++;
//								//	through_end[through_index] = l;
//								//}
//								//through_flag = 0;
//							}
//							test_point++;
//						}
//						if(through_index%2 == 1)
//							through_index--;
//					}
//
//					//根据x坐标删点
//					if(through_index > LEAST_POINT)		//剩余点大于20
//					{
//						p_through_point = through_point;
//
//						for(k = 0;k < through_index-1;k++)
//							for(l = k+1;l < through_index;l++)
//							{								
//								if(p_through_point[k].X > p_through_point[l].X)
//								{
//									tmp_point = p_through_point[k];
//									p_through_point[k] = p_through_point[l];
//									p_through_point[l] = tmp_point;
//								}
//							}
//
//						through_total = 0;
//#pragma MUST_ITERATE(LEAST_POINT)
//						for(k = 0;k < through_index;k++)
//							through_total += p_through_point[k].X;
//						
//						while(through_index > LEAST_POINT)
//						{							
//							through_avg = through_total/through_index;
//
//							through_diff1 = abs(p_through_point[0].X-through_avg);
//							through_diff2 = abs(p_through_point[through_index-1].X-through_avg);
//
//							if(through_diff1 > through_diff2)
//							{
//								if(through_diff1 > MAX_THROUGH_DIFF)		//
//								{
//									through_total -= p_through_point[0].X;
//									p_through_point++;
//									through_index--;
//								}
//								else
//									break;
//							}
//							else
//							{
//								if(through_diff2 > MAX_THROUGH_DIFF)		//
//								{
//									through_total -= p_through_point[through_index-1].X;
//									through_index--;
//								}
//								else
//									break;
//							}
//						}
//
//#ifdef WATCH_MODE
//						memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
//						for(k = 0;k < through_index;k++)
//							src_watch[p_through_point[k].Y*SRC_IMG_X+p_through_point[k].X] = 255;
//#endif
//
//						//根据y坐标删点
//						if(through_index > LEAST_POINT)		//剩余点大于16
//						{
//							for(k = 0;k < through_index-1;k++)
//								for(l = k+1;l < through_index;l++)
//								{								
//									if(p_through_point[k].Y > p_through_point[l].Y)
//									{
//										tmp_point = p_through_point[k];
//										p_through_point[k] = p_through_point[l];
//										p_through_point[l] = tmp_point;
//									}
//								}
//
//							through_total = 0;
//#pragma MUST_ITERATE(LEAST_POINT)
//							for(k = 0;k < through_index;k++)
//								through_total += p_through_point[k].Y;
//
//							while(through_index > LEAST_POINT)
//							{
//								through_avg = through_total/through_index;
//
//								through_diff1 = abs(p_through_point[0].Y-through_avg);
//								through_diff2 = abs(p_through_point[through_index-1].Y-through_avg);
//
//								if(through_diff1 > through_diff2)
//								{
//									if(through_diff1 > MAX_THROUGH_DIFF)		//
//									{
//										through_total -= p_through_point[0].Y;
//										p_through_point++;
//										through_index--;
//									}
//									else
//										break;
//								}
//								else
//								{
//									if(through_diff2 > MAX_THROUGH_DIFF)		//
//									{
//										through_total -= p_through_point[through_index-1].Y;
//										through_index--;
//									}
//									else
//										break;
//								}
//							}
//
//#ifdef WATCH_MODE
//							memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
//							for(k = 0;k < through_index;k++)
//								src_watch[p_through_point[k].Y*SRC_IMG_X+p_through_point[k].X] = 255;
//#endif
//
//							if(through_index > LEAST_POINT)		//剩余点大于16
//							{
//								oriLoc->Y = through_total*_rcpdp((double)through_index);
//
//								through_total = 0;
//#pragma MUST_ITERATE(LEAST_POINT)
//								for(k = 0;k < through_index;k++)
//									through_total += p_through_point[k].X;
//								oriLoc->X = through_total*_rcpdp((double)through_index);			
//
//#ifdef WATCH_MODE
//								memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
//#endif
//
//								return DmtxPass;
//							}
//						}
//					}
//#ifdef WATCH_MODE
//					memcpy(src_watch, srcImg, SRC_IMG_X*SRC_IMG_Y);
//#endif
//				}
			}
			//cur_point += DM_ONE_SIZE;
		}
	}
	if(max_grid_cnt > LEAST_POINT_GAI)
		return DmtxPass;

	return DmtxFail;
}

/**
 * \brief  Create copy of existing region struct
 * \param  None
 * \return Initialized DmtxRegion struct
 */
extern DmtxRegion *
dmtxRegionCreate(DmtxRegion *reg)
{
   DmtxRegion *regCopy;

   regCopy = &g_reg;
   if(regCopy == NULL)
      return NULL;

   memcpy(regCopy, reg, sizeof(DmtxRegion));

   return regCopy;
}

extern DmtxRegion *
dmtxRegionFindNext_mt(DmtxDecode *dec, volatile int16_t* new_pic_cnt)
{
	DmtxPassFail err;
   int locStatus;
   DmtxPixelLoc loc;
   DmtxRegion   *reg;

   g_camera_id = cur_camera->camera_id;

   reg = &g_reg;
   //g_total_time = 0;
   /* Continue until we find a region or run out of chances */
   for(;;) {
	   //得到范围内的好点
      locStatus = PopGridLocation(&(dec->grid), &loc);
      if(locStatus == DmtxRangeEnd)
	  {
		 //printf("end!\n");
         break;
	  }

      //tic();
      /* Scan location for presence of valid barcode region */
      err = dmtxRegionScanPixel(dec, reg, loc.X, loc.Y);
      //toc("");

      if(err == DmtxPass)
	  {
    	  //printf("g_total_time = %f\n", g_total_time*_rcpsp(456000));

          return reg;
	  }

      /* Ran out of time? */
#ifdef WIN32
      if(*new_pic_cnt == 2)
#else
      if(*new_pic_cnt > 0)
#endif
	  {
		 //printf("time out!\n");
         break;
	  }
   }

   //printf("time out!\n");
   return NULL;
}

#ifdef CIRCLE_LINE_CALIB
extern DmtxRegion *
dmtxRegionFindNext_mt_calib(DmtxDecode *dec, volatile int16_t* new_pic_cnt, int* reg_cnt)
{
	DmtxPassFail err;
   int locStatus;
   DmtxPixelLoc loc;
   DmtxRegion   *reg;

   int exp_depart, i;
   double dis;
   int res_cnt = 0;
   DmtxVector2 p00, p00_cur;

   reg = g_reg_calib;
   //g_total_time = 0;
   /* Continue until we find a region or run out of chances */
   for(;;) {
	   //得到范围内的好点
      locStatus = PopGridLocation(&(dec->grid), &loc, &exp_depart);
      if(locStatus == DmtxRangeEnd)
	  {
		 //printf("end!\n");
         break;
	  }

      //tic();
      /* Scan location for presence of valid barcode region */
      err = dmtxRegionScanPixel(dec, &reg[res_cnt], loc.X, loc.Y, exp_depart);
      //toc("");

      if(err == DmtxPass)
	  {
		  p00_cur.X = p00_cur.Y = 0.0;
		  dmtxMatrix3VMultiplyBy(&p00_cur, reg[res_cnt].fit2raw);

		  for(i = 0;i < res_cnt;i++)
		  {
			  p00.X = p00.Y = 0.0;
			  dmtxMatrix3VMultiplyBy(&p00, reg[i].fit2raw);

			  dis = (p00.X - p00_cur.X)*(p00.X - p00_cur.X)+(p00.Y - p00_cur.Y)*(p00.Y - p00_cur.Y);
			  if(dis < 100)
				  break;
		  }
		  if(i == res_cnt)
			  res_cnt++;

		  if(res_cnt == 4)
		  {
			  break;
		  }
	  }

      /* Ran out of time? */
#ifdef WIN32
      if(*new_pic_cnt == 2)
#else
      if(*new_pic_cnt > 0)
#endif
	  {
		 //printf("time out!\n");
         break;
	  }
   }

   *reg_cnt = res_cnt;
   printf("检测到的二维码个数为%d个\n", res_cnt); 

   //printf("time out!\n");
   return reg;
}
#endif
/**
 * \brief  Scan individual pixel for presence of barcode edge
 * \param  dec Pointer to DmtxDecode information struct
 * \param  loc Pixel location
 * \return Detected region (if any)
 */
extern
DmtxPassFail dmtxRegionScanPixel(DmtxDecode *dec, DmtxRegion *reg, int x, int y)
{
	DmtxPassFail err;
#ifndef NDEBUG
   DmtxImgInfo *cache;
#endif
   //DmtxRegion reg;
   //DmtxPointFlow flowBegin;
   DmtxPixelLoc loc;

   int status;

   dmLinePoints linePoints[2];

   loc.X = x;
   loc.Y = y;

#ifndef NDEBUG
   cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
   assert(cache != NULL);

   assert((int)(cache->visited & 0x80) == 0x00);
#endif

   /* Test for presence of any reasonable edge at this location */
   //查找种子点
   //tic();
   err = MatrixRegionSeekEdge(dec, loc);
   //toc("");

   //若种子点的梯度量小于77，则直接放弃该种子点
   /*if(flowBegin.mag < (int)(dec->edgeThresh * 7.65 + 0.5))
      return NULL;*/

   if(err == DmtxFail)
	   return DmtxFail;

#ifdef	WATCH_MODE
   if(loc.Y < 2)
	   loc.Y = 2;
   if(loc.Y > DM_PROJ_IMG_SIZE-3)
	   loc.Y = DM_PROJ_IMG_SIZE-3;
   if(loc.X < 2)
	   loc.X = 2;
   if(loc.X > DM_PROJ_IMG_SIZE-3)
	   loc.X = DM_PROJ_IMG_SIZE-3;

	img_watch[loc.Y*DM_PROJ_IMG_SIZE + loc.X] = 255;
	img_watch[(loc.Y-1)*DM_PROJ_IMG_SIZE + loc.X-1] = 255;
	img_watch[(loc.Y-1)*DM_PROJ_IMG_SIZE + loc.X+1] = 255;
	img_watch[(loc.Y+1)*DM_PROJ_IMG_SIZE + loc.X-1] = 255;
	img_watch[(loc.Y+1)*DM_PROJ_IMG_SIZE + loc.X+1] = 255;
	img_watch[(loc.Y-2)*DM_PROJ_IMG_SIZE + loc.X-2] = 255;
	img_watch[(loc.Y-2)*DM_PROJ_IMG_SIZE + loc.X+2] = 255;
	img_watch[(loc.Y+2)*DM_PROJ_IMG_SIZE + loc.X-2] = 255;
	img_watch[(loc.Y+2)*DM_PROJ_IMG_SIZE + loc.X+2] = 255;
#endif

   memset(reg, 0x00, sizeof(DmtxRegion));
   reg->outline = g_outline;
   reg->outline_pos = 1;
   reg->outline_neg = OUTLINE_POINT_NUM-1;

   /* Determine barcode orientation */
   //搜索轮廓并确定左边沿与下边沿
   //tic();
   status = MatrixRegionOrientation3(dec, reg, loc, linePoints);
   //toc("");
   if (status == DmtxFail)
	   return DmtxFail;
  
   //计算变换矩阵
   if(dmtxRegionUpdateXfrms(dec, reg) == DmtxFail)
      return DmtxFail;

   /* Define top edge */
   //tic();
   status = MatrixRegionAlignCalibEdge(dec, reg, DmtxEdgeTop);
   //toc("1");

   if(status == DmtxFail)
      return DmtxFail;
   if(dmtxRegionUpdateXfrms(dec, reg) == DmtxFail)
      return DmtxFail;

   /* Define right edge */
   //tic();
   status = MatrixRegionAlignCalibEdge(dec, reg, DmtxEdgeRight);
   //toc("2");

   if(status == DmtxFail)
      return DmtxFail;
   if(dmtxRegionUpdateXfrms(dec, reg) == DmtxFail)
      return DmtxFail;

   CALLBACK_MATRIX(reg);

   /* Calculate the best fitting symbol size */
   status = MatrixRegionFindSize(dec, reg);

   if(status == DmtxFail)
      return DmtxFail;

   //tic();
   status = preciseCalc(dec, reg, linePoints);
   //toc("");

   if(status == DmtxFail)
      return DmtxFail;

   /* Found a valid matrix region */
   return DmtxPass;
}

/**
 *查找种子点
 *
 */
/*static*/ DmtxPassFail
MatrixRegionSeekEdge(DmtxDecode *dec, DmtxPixelLoc loc)
{
   uint16_t err;
   //DmtxPointFlow flow, flowPlane[3];
   //DmtxPointFlow flowPos, flowPosBack;
   //DmtxPointFlow flowNeg, flowNegBack;

   DmtxImgInfo* cache;
   DmtxImgInfo *cache1, *cache2;
   DmtxPixelLoc loc1, loc2;
   int center_depart, error, exp_depart;

   //channelCount = dec->image->channelCount;

   /* Find whether red, green, or blue shows the strongest edge */
	cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
	//计算该点的梯度量以及梯度方向
    err = GetPointFlow(dec, loc, cache);
	if(err == DmtxFail)
		return DmtxFail;

   //若梯度量小于10，则该点不是种子点（此处应可改为77）
   /*if(flowPlane[strongIdx].mag < 10)
      return dmtxBlankEdge;*/

	if(cache->grad < EDGE_THRESHOLD)
      return DmtxFail;

	if(g_camera_id == 0)
	{
#ifndef FULL_PIC_PROJECT
		if(loc.X < DOWN_PROJ_IMG_SIZE_2)
		{
			if(loc.Y < DOWN_PROJ_IMG_SIZE_2)
				exp_depart = 2;
			else
				exp_depart = 0;
		}
		else
		{
			if(loc.Y < DOWN_PROJ_IMG_SIZE_2)
				exp_depart = 4;
			else
				exp_depart = 6;
		}

		center_depart = cache->depart & 0x07;
		error = (center_depart+8-exp_depart)%8;
		if(error > 2 && error < 6)
			return DmtxFail;
#endif
	}

   //flow = flowPlane[strongIdx];

   //找到正方向的三个领域点中梯度最强点的位置以及其梯度量与梯度方向
	err = FindStrongestNeighbor(dec, cache, loc, +1, &cache1, &loc1);
	if(err == DmtxFail)
		return DmtxFail;
	err = FindStrongestNeighbor(dec, cache1, loc1, -1, &cache2, &loc2);
	if(err == DmtxFail)
		return DmtxFail;
	if(loc2.X  != loc.X || loc2.Y != loc.Y)
		return DmtxFail;
   //找到负方向的三个领域点中梯度最强的梯度量与梯度方向
   err = FindStrongestNeighbor(dec, cache, loc, -1, &cache1, &loc1);
   if(err == DmtxFail)
	   return DmtxFail;
   err = FindStrongestNeighbor(dec, cache1, loc1, +1, &cache2, &loc2);
   if(err == DmtxFail)
	   return DmtxFail;
   if(loc2.X  != loc.X || loc2.Y != loc.Y)
		return DmtxFail;
   //if(flowPos.mag != -1 && flowNeg.mag != -1) {
	  // //从梯度最强的领域点反向搜索梯度最强点的位置以及其梯度量与梯度方向
   //   flowPosBack = FindStrongestNeighbor(dec, flowPos, -1);
   //   flowNegBack = FindStrongestNeighbor(dec, flowNeg, +1);

	  ////若反向搜索到的为原点，则将该点作为种子点
   //   if(flowPos.arrive == (flowPosBack.arrive+4)%8 &&
   //         flowNeg.arrive == (flowNegBack.arrive+4)%8) {
   //      flow.arrive = dmtxNeighborNone;
   //      CALLBACK_POINT_PLOT(flow.loc, 1, 1, 1);
   //      return flow;
   //   }
   //}

   return DmtxPass;
}

//if(dec->edgeMax != DmtxUndefined) {
//	   maxDiagonal = (int)(1.56 * dec->edgeMax + 0.5); /* sqrt(2) + 10% */
//   }
//   else {
//      maxDiagonal = DmtxUndefined;
//   }
int g_maxDiagonal = 312;	/* sqrt(2)*48 + 10% */

//if(dec->edgeMin != DmtxUndefined) {
//      scale = dmtxDecodeGetProp(dec, DmtxPropScale);
//
//      minArea = (dec->edgeMin * dec->edgeMin)/(scale * scale);
//
//      if((reg->boundMax.X - reg->boundMin.X) * (reg->boundMax.Y - reg->boundMin.Y) < minArea) {
//         return DmtxFail;
//      }
//   }
extern int g_minArea;
/**
 *
 *
 */
/*static*/ DmtxPassFail
	MatrixRegionOrientation3(DmtxDecode *dec, DmtxRegion *reg, DmtxPixelLoc locBegin, dmLinePoints* LinePoints)
{
   int cross;
   //int symbolShape;
   DmtxPassFail err;
   DmtxBestLine line1x, line2x;
   DmtxBestLine line2n, line2p;
   //DmtxFollow fTmp;
   int next_step;
   
   //tic();
   /* Follow to end in both directions */
   //搜索轮廓
   err = TrailBlazeContinuous(dec, reg, locBegin, g_maxDiagonal);
   //toc("");
#ifdef PRINT_LOC
   printf("{%d,\t%d}\t%d\n", begin.loc.X, begin.loc.Y, reg->stepsTotal);
#endif

   if(err == DmtxFail || reg->stepsTotal < 40) {
      return DmtxFail;
   }

   /* Filter out region candidates that are smaller than expected */
   if((reg->boundMax.X - reg->boundMin.X) * (reg->boundMax.Y - reg->boundMin.Y) < g_minArea)
	   return DmtxFail;
   //tic();
   //line1x = FindBestSolidLine(dec, reg, 0, 0, +1, DmtxUndefined);
   line1x = FindBestSolidLine_angleLimit(dec, reg, 0, 0, dec->last_bottom_angle, 0);
   //toc("");
   if(line1x.mag < 5) {
      return DmtxFail;
   }

   //err = FindTravelLimits(dec, reg, &line1x);
   err = FindTravelLimits3(dec, reg, &line1x, LinePoints[0].line_point, &LinePoints[0].line_start, &LinePoints[0].line_end);

   //line_len = line_end-line_start;
   //若线段长度过短或者缺陷程度过大，将返回失败
   if(line1x.distSq < 100 || line1x.devn * 10 >= sqrtdp_i((double)line1x.distSq)) {
      return DmtxFail;
   }
   //assert(line1x.stepPos < line1x.stepNeg);

   //从正方向搜索垂线
   /*sign_changed = 0;
   line2p.mag = 0;
   fTmp = FollowSeek3(dec, reg, line1x.stepPos + 5, &sign_changed);
   if(!sign_changed)*/
   line2p.mag = 0;
   next_step = line1x.stepPos+5;		//此处stepPos至少为0
   if(next_step < reg->outline_pos)
		line2p = FindBestSolidLine_angleLimit(dec, reg, next_step, +1, line1x.angle, DmtxUndefined);

   //从负方向搜索垂线
   /*sign_changed = 0;
   line2n.mag = 0;
   fTmp = FollowSeek3(dec, reg, line1x.stepNeg - 5, &sign_changed);
   if(!sign_changed)*/
   line2n.mag = 0;
   next_step = (line1x.stepNeg+OUTLINE_POINT_NUM-5)%OUTLINE_POINT_NUM;	//此处stepNeg最多为0
   if(next_step > reg->outline_neg)
		line2n = FindBestSolidLine_angleLimit(dec, reg, next_step, -1, line1x.angle, DmtxUndefined);
   if(max(line2p.mag, line2n.mag) < 5)
      return DmtxFail;

   //利用最小二乘来精确拟合直线
   LinePoints[0].ls_flag = abs(90 - line1x.angle) > 45 ? 0:1;	//0表示x与y不呼唤，1表示互换

   //正方向左下，负方向右上
   if(line2p.mag > line2n.mag) {
	   //line1为
      line2x = line2p;
      //err = FindTravelLimits(dec, reg, &line2x);
	  err = FindTravelLimits3(dec, reg, &line2x, LinePoints[1].line_point, &LinePoints[1].line_start, &LinePoints[1].line_end);
	  //line_len = line_end-line_start;

      if(line2x.distSq < 100 || line2x.devn * 10 >= sqrtdp_i((double)line2x.distSq))
         return DmtxFail;

	  LinePoints[1].ls_flag = abs(90 - line2x.angle) > 45 ? 0:1;	//0表示x与y不呼唤，1表示互换

      cross = ((line1x.locPos.X - line1x.locNeg.X) * (line2x.locPos.Y - line2x.locNeg.Y)) -
            ((line1x.locPos.Y - line1x.locNeg.Y) * (line2x.locPos.X - line2x.locNeg.X));
      if(cross < 0) {
         /* Condition 2 */
         reg->polarity = +1;
         reg->locR = line2x.locPos;
         reg->stepR = line2x.stepPos;
         reg->locT = line1x.locNeg;
         reg->stepT = line1x.stepNeg;
         reg->leftLoc = line1x.locBeg;
         reg->leftAngle = line1x.angle;
         reg->bottomLoc = line2x.locBeg;
         reg->bottomAngle = line2x.angle;
         reg->leftLine = line1x;
         reg->bottomLine = line2x;

		 //bottom_line = &line_t[1];
		 //left_line = &line_t[0];
		 LinePoints[0].line_flag = 1;
		 LinePoints[1].line_flag = 0;
      }
      else {
         /* Condition 3 */
         reg->polarity = -1;
         reg->locR = line1x.locNeg;
         reg->stepR = line1x.stepNeg;
         reg->locT = line2x.locPos;
         reg->stepT = line2x.stepPos;
         reg->leftLoc = line2x.locBeg;
         reg->leftAngle = line2x.angle;
         reg->bottomLoc = line1x.locBeg;
         reg->bottomAngle = line1x.angle;
         reg->leftLine = line2x;
         reg->bottomLine = line1x;

		 LinePoints[0].line_flag = 0;
		 LinePoints[1].line_flag = 1;
      }
   }
   else {
      line2x = line2n;
      //err = FindTravelLimits(dec, reg, &line2x);
	  err = FindTravelLimits3(dec, reg, &line2x, LinePoints[1].line_point, &LinePoints[1].line_start, &LinePoints[1].line_end);
	  //line_len = line_end-line_start;

      if(line2x.distSq < 100 || line2x.devn *10 >= sqrtdp_i((double)line2x.distSq))
         return DmtxFail;

	  LinePoints[1].ls_flag = abs(90 - line2x.angle) > 45 ? 0:1;	//0表示x与y不呼唤，1表示互换

      cross = ((line1x.locNeg.X - line1x.locPos.X) * (line2x.locNeg.Y - line2x.locPos.Y)) -
            ((line1x.locNeg.Y - line1x.locPos.Y) * (line2x.locNeg.X - line2x.locPos.X));
      if(cross < 0) {
         /* Condition 1 */
         reg->polarity = -1;
         reg->locR = line2x.locNeg;
         reg->stepR = line2x.stepNeg;
         reg->locT = line1x.locPos;
         reg->stepT = line1x.stepPos;
         reg->leftLoc = line1x.locBeg;
         reg->leftAngle = line1x.angle;
         reg->bottomLoc = line2x.locBeg;
         reg->bottomAngle = line2x.angle;
         reg->leftLine = line1x;
         reg->bottomLine = line2x;

		 LinePoints[0].line_flag = 1;
		 LinePoints[1].line_flag = 0;
      }
      else {
         /* Condition 4 */
         reg->polarity = +1;
         reg->locR = line1x.locPos;
         reg->stepR = line1x.stepPos;
         reg->locT = line2x.locNeg;
         reg->stepT = line2x.stepNeg;
         reg->leftLoc = line2x.locBeg;
         reg->leftAngle = line2x.angle;
         reg->bottomLoc = line1x.locBeg;
         reg->bottomAngle = line1x.angle;
         reg->leftLine = line2x;
         reg->bottomLine = line1x;

		 LinePoints[0].line_flag = 0;
		 LinePoints[1].line_flag = 1;
      }
   }

   reg->leftKnown = reg->bottomKnown = 1;

   return DmtxPass;
}

uint16_t convert2subPixel(DmtxDecode *dec, DmtxPixelLoc* p, uint16_t count, DmtxPixelLocf* resLoc)
{
	DmtxImgInfo *cache, *cache1, *cache2;
	DmtxPixelLoc loc, loc1, loc2;
	int i, res_cnt;
	uint16_t cur_depart;
	int16_t y1, y2, y3, tmp1, tmp2;
	float b;
	//float a, c, res_y;

	DmtxPassFail err;

	res_cnt = 0;
	for(i = 0;i < count;i++)
	{
		loc = p[i];
		cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
		assert(cache->depart & 0x80);
		cur_depart = (cache->depart & 0x07)%4;
		y2 = cache->grad;

		switch(cur_depart)
		{
		case 0:
			loc1.X = loc.X+1;
			loc1.Y = loc.Y-1;
			loc2.X = loc.X-1;
			loc2.Y = loc.Y+1;
			cache1 = dmtxDecodeGetCache(dec, loc1.X, loc1.Y);
			cache2 = dmtxDecodeGetCache(dec, loc2.X, loc2.Y);
			err = GetPointFlow(dec, loc1, cache1);
			if(err == DmtxFail)
				break;
			err = GetPointFlow(dec, loc2, cache2);
			if(err == DmtxFail)
				break;
			y1 = cache1->grad;
			y3 = cache2->grad;
			tmp1 = y1-y3;
			tmp2 = (y1+y3-2*y2)*2;
			if(tmp2 == 0)
				break;
			b = tmp1*_rcpsp((float)tmp2);
			if(b > 1|| b < -1)
				break;
			resLoc[res_cnt].X = loc.X-b;
			resLoc[res_cnt].Y = loc.Y+b;
			res_cnt++;

			break;
		case 1:
			loc1.X = loc.X-1;
			loc1.Y = loc.Y;
			loc2.X = loc.X+1;
			loc2.Y = loc.Y;
			cache1 = dmtxDecodeGetCache(dec, loc1.X, loc1.Y);
			cache2 = dmtxDecodeGetCache(dec, loc2.X, loc2.Y);
			err = GetPointFlow(dec, loc1, cache1);
			if(err == DmtxFail)
				break;
			err = GetPointFlow(dec, loc2, cache2);
			if(err == DmtxFail)
				break;
			y1 = cache1->grad;
			y3 = cache2->grad;
			tmp1 = y1-y3;
			tmp2 = (y1+y3-2*y2)*2;
			if(tmp2 == 0)
				break;
			b = tmp1*_rcpsp((float)tmp2);
			if(b > 1|| b < -1)
				break;
			resLoc[res_cnt].X = loc.X+b;
			resLoc[res_cnt].Y = loc.Y;
			res_cnt++;

			/*a = (y1-y2)/(2*b+1);
			c = y2-a*b*b;
			res_y = a*(b+1)*(b+1)+c;
			res_y = a*b*b+c;
			res_y = a*(b-1)*(b-1)+c;*/
				
			break;
		case 2:
			loc1.X = loc.X-1;
			loc1.Y = loc.Y-1;
			loc2.X = loc.X+1;
			loc2.Y = loc.Y+1;
			cache1 = dmtxDecodeGetCache(dec, loc1.X, loc1.Y);
			cache2 = dmtxDecodeGetCache(dec, loc2.X, loc2.Y);
			err = GetPointFlow(dec, loc1, cache1);
			if(err == DmtxFail)
				break;
			err = GetPointFlow(dec, loc2, cache2);
			if(err == DmtxFail)
				break;
			y1 = cache1->grad;
			y3 = cache2->grad;
			tmp1 = y1-y3;
			tmp2 = (y1+y3-2*y2)*2;
			if(tmp2 == 0)
				break;
			b = tmp1*_rcpsp((float)tmp2);
			if(b > 1|| b < -1)
				break;
			resLoc[res_cnt].X = loc.X+b;
			resLoc[res_cnt].Y = loc.Y+b;
			res_cnt++;

			break;
		case 3:
			loc1.X = loc.X;
			loc1.Y = loc.Y-1;
			loc2.X = loc.X;
			loc2.Y = loc.Y+1;
			cache1 = dmtxDecodeGetCache(dec, loc1.X, loc1.Y);
			cache2 = dmtxDecodeGetCache(dec, loc2.X, loc2.Y);
			err = GetPointFlow(dec, loc1, cache1);
			if(err == DmtxFail)
				break;
			err = GetPointFlow(dec, loc2, cache2);
			if(err == DmtxFail)
				break;
			y1 = cache1->grad;
			y3 = cache2->grad;
			tmp1 = y1-y3;
			tmp2 = (y1+y3-2*y2)*2;
			if(tmp2 == 0)
				break;
			b = tmp1*_rcpsp((float)tmp2);
			if(b > 1|| b < -1)
				break;
			resLoc[res_cnt].X = loc.X;
			resLoc[res_cnt].Y = loc.Y+b;
			res_cnt++;

			break;

		default:
			break;
		}
	}

	return res_cnt;
}

DmtxPassFail preciseCalc(DmtxDecode *dec, DmtxRegion *reg, dmLinePoints* LinePoints)
{
	double bottom_angle;
    //dmLine line_t[2];
    float tmp, k1, k2, b1, b2;
	uint16_t line_len, bottom_ls_flag, left_ls_flag, line_point_f_cnt;
	DmtxPixelLocf line_point_f[140];

   //利用最小二乘来精确拟合直线
   //line_t[0].ls_flag = abs(line1x.angle) < 45 ? 0:1;	//0表示x与y不呼唤，1表示互换
   if(LinePoints[0].line_flag == 0)			//bottom
   {
	   line_len = LinePoints[0].line_end-LinePoints[0].line_start;
		//若线段长度过短或者缺陷程度过大，将返回失败
		if(line_len < 20) {
			return DmtxFail;
		}
		bottom_ls_flag = LinePoints[0].ls_flag;

		line_point_f_cnt = convert2subPixel(dec, LinePoints[0].line_point+LinePoints[0].line_start+5, line_len-10, line_point_f);
		if(line_point_f_cnt < LEAST_SQUARE_LEAST_CNT)
			return DmtxFail;
		LeastSquare(line_point_f, line_point_f_cnt, &b1, &k1, bottom_ls_flag);

	   line_len = LinePoints[1].line_end-LinePoints[1].line_start;
		//若线段长度过短或者缺陷程度过大，将返回失败
		if(line_len < 20) {
			return DmtxFail;
		}
		left_ls_flag = LinePoints[1].ls_flag;

		line_point_f_cnt = convert2subPixel(dec, LinePoints[1].line_point+LinePoints[1].line_start+5, line_len-10, line_point_f);
		if(line_point_f_cnt < LEAST_SQUARE_LEAST_CNT)
			return DmtxFail;
		LeastSquare(line_point_f, line_point_f_cnt, &b2, &k2, left_ls_flag);
   }
   else                  //left
   {
	   line_len = LinePoints[0].line_end-LinePoints[0].line_start;
		//若线段长度过短或者缺陷程度过大，将返回失败
		if(line_len < 20) {
			return DmtxFail;
		}
		left_ls_flag = LinePoints[0].ls_flag;

		line_point_f_cnt = convert2subPixel(dec, LinePoints[0].line_point+LinePoints[0].line_start+5, line_len-10, line_point_f);
		if(line_point_f_cnt < LEAST_SQUARE_LEAST_CNT)
			return DmtxFail;
		LeastSquare(line_point_f, line_point_f_cnt, &b2, &k2, left_ls_flag);

	   line_len = LinePoints[1].line_end-LinePoints[1].line_start;
		//若线段长度过短或者缺陷程度过大，将返回失败
		if(line_len < 20) {
			return DmtxFail;
		}
		bottom_ls_flag = LinePoints[1].ls_flag;

		line_point_f_cnt = convert2subPixel(dec, LinePoints[1].line_point+LinePoints[1].line_start+5, line_len-10, line_point_f);
		if(line_point_f_cnt < LEAST_SQUARE_LEAST_CNT)
			return DmtxFail;
		LeastSquare(line_point_f, line_point_f_cnt, &b1, &k1, bottom_ls_flag);
   }

   bottom_angle = atansp_i(k1);
   if(bottom_ls_flag == 0)
   {
	   reg->res_angle = bottom_angle;

	   if(left_ls_flag == 0)
	   {
		   tmp = k1-k2;
		   reg->res_loc.X = (b2-b1)*_rcpsp(tmp);
		   reg->res_loc.Y = (k1*b2-k2*b1)*_rcpsp(tmp);
	   }
	   else
	   {
		   tmp = 1-k1*k2;
		   reg->res_loc.X = (k2*b1+b2)*_rcpsp(tmp);
		   reg->res_loc.Y = (k1*b2+b1)*_rcpsp(tmp);
	   }
   }
   else
   {
	   if(bottom_angle < 0)
		   reg->res_angle = (-PI/2)-bottom_angle;
	   else
		   reg->res_angle = (PI/2)-bottom_angle;

	   if(left_ls_flag == 0)
	   {
		   tmp = 1-k1*k2;
		   reg->res_loc.X = (k1*b2+b1)*_rcpsp(tmp);
		   reg->res_loc.Y = (k2*b1+b2)*_rcpsp(tmp);
	   }
	   else
	   {
		   tmp = k1-k2;
		   reg->res_loc.Y = (b2-b1)*_rcpsp(tmp);
		   reg->res_loc.X = (k1*b2-k2*b1)*_rcpsp(tmp);
	   }
   }

   return DmtxPass;
}

/**
 *
 *
 */
/*static*/ long
DistanceSquared(DmtxPixelLoc a, DmtxPixelLoc b)
{
   long xDelta, yDelta;

   xDelta = a.X - b.X;
   yDelta = a.Y - b.Y;

   return (xDelta * xDelta) + (yDelta * yDelta);
}

/**
 *
 *
 */
extern DmtxPassFail
dmtxRegionUpdateCorners(DmtxDecode *dec, DmtxRegion *reg, DmtxVector2 p00,
      DmtxVector2 p10, DmtxVector2 p11, DmtxVector2 p01)
{
   double xMax, yMax;
   double tx, ty, phi, shx, scx, scy, skx, sky;
   double dimOT, dimOR, dimTX, dimRX, ratio;
   DmtxVector2 vOT, vOR, vTX, vRX, vTmp;
   DmtxMatrix3 m, mtxy, mphi, mshx, mscx, mscy, mscxy, msky, mskx;

   xMax = (double)(dmtxDecodeGetProp(dec, DmtxPropWidth) - 1);
   yMax = (double)(dmtxDecodeGetProp(dec, DmtxPropHeight) - 1);

   if(p00.X < 0.0 || p00.Y < 0.0 || p00.X > xMax || p00.Y > yMax ||
         p01.X < 0.0 || p01.Y < 0.0 || p01.X > xMax || p01.Y > yMax ||
         p10.X < 0.0 || p10.Y < 0.0 || p10.X > xMax || p10.Y > yMax)
      return DmtxFail;

   dimOT = dmtxVector2Mag(dmtxVector2Sub(&vOT, &p01, &p00)); /* XXX could use MagSquared() */
   dimOR = dmtxVector2Mag(dmtxVector2Sub(&vOR, &p10, &p00));
   dimTX = dmtxVector2Mag(dmtxVector2Sub(&vTX, &p11, &p01));
   dimRX = dmtxVector2Mag(dmtxVector2Sub(&vRX, &p11, &p10));

   /* Verify that sides are reasonably long */
   if(dimOT <= 8.0 || dimOR <= 8.0 || dimTX <= 8.0 || dimRX <= 8.0)
      return DmtxFail;

   /* Verify that the 4 corners define a reasonably fat quadrilateral */
   ratio = dimOT *_rcpdp(dimRX);
   if(ratio <= 0.5 || ratio >= 2.0)
      return DmtxFail;

   ratio = dimOR *_rcpdp(dimTX);
   if(ratio <= 0.5 || ratio >= 2.0)
      return DmtxFail;

   /* Verify this is not a bowtie shape */
   if(dmtxVector2Cross(&vOR, &vRX) > 0.0 ||
         dmtxVector2Cross(&vOT, &vTX) < 0.0)
      return DmtxFail;

   //判断三点是否组成直角
   if(RightAngleTrueness(p00, p10, p11, M_PI_2) <= dec->squareDevn)
      return DmtxFail;
   if(RightAngleTrueness(p10, p11, p01, M_PI_2) <= dec->squareDevn)
      return DmtxFail;

   /* Calculate values needed for transformations */
   //平移变换
   tx = -1 * p00.X;
   ty = -1 * p00.Y;
   dmtxMatrix3Translate(mtxy, tx, ty);

   //旋转变换
   phi = atan2dp_i(vOT.X, vOT.Y);
   dmtxMatrix3Rotate(mphi, phi);
   dmtxMatrix3Multiply(m, mtxy, mphi);

   //剪切变换
   dmtxMatrix3VMultiply(&vTmp, &p10, m);

   scx = _rcpdp(vTmp.X);

   shx = -vTmp.Y * scx;
   dmtxMatrix3Shear(mshx, 0.0, shx);
   dmtxMatrix3MultiplyBy(m, mshx);

   //缩放变换
   //scx = 1.0/vTmp.X;
   dmtxMatrix3Scale(mscx, scx, 1.0);
   dmtxMatrix3MultiplyBy(m, mscx);

   dmtxMatrix3VMultiply(&vTmp, &p11, m);
   scy = _rcpdp(vTmp.Y);
   dmtxMatrix3Scale(mscy, 1.0, scy);
   dmtxMatrix3MultiplyBy(m, mscy);

   //边沿倾斜变换
   dmtxMatrix3VMultiply(&vTmp, &p11, m);
   skx = vTmp.X;
   //dmtxMatrix3LineSkewSide(mskx, 1.0, skx, 1.0);
   dmtxMatrix3LineSkewSide(mskx, skx);
   dmtxMatrix3MultiplyBy(m, mskx);

   //顶部倾斜变换
   dmtxMatrix3VMultiply(&vTmp, &p01, m);
   sky = vTmp.Y;
   //dmtxMatrix3LineSkewTop(msky, sky, 1.0, 1.0);
   dmtxMatrix3LineSkewTop(msky, sky);
   dmtxMatrix3Multiply(reg->raw2fit, m, msky);

   /* Create inverse matrix by reverse (avoid straight matrix inversion) */
   //dmtxMatrix3LineSkewTopInv(msky, sky, 1.0, 1.0);
   dmtxMatrix3LineSkewTopInv(msky, sky);
   //dmtxMatrix3LineSkewSideInv(mskx, 1.0, skx, 1.0);
   dmtxMatrix3LineSkewSideInv(mskx, skx);
   dmtxMatrix3Multiply(m, msky, mskx);

   dmtxMatrix3Scale(mscxy, _rcpdp(scx), _rcpdp(scy));
   dmtxMatrix3MultiplyBy(m, mscxy);

   dmtxMatrix3Shear(mshx, 0.0, -shx);
   dmtxMatrix3MultiplyBy(m, mshx);

   dmtxMatrix3Rotate(mphi, -phi);
   dmtxMatrix3MultiplyBy(m, mphi);

   dmtxMatrix3Translate(mtxy, -tx, -ty);
   dmtxMatrix3Multiply(reg->fit2raw, m, mtxy);

   return DmtxPass;
}

/**
 *
 *
 */
extern DmtxPassFail
dmtxRegionUpdateXfrms(DmtxDecode *dec, DmtxRegion *reg)
{
   double radians;
   DmtxRay2 rLeft, rBottom, rTop, rRight;
   DmtxVector2 p00, p10, p11, p01;

   assert(reg->leftKnown != 0 && reg->bottomKnown != 0);

   /* Build ray representing left edge */
   rLeft.p.X = (double)reg->leftLoc.X;
   rLeft.p.Y = (double)reg->leftLoc.Y;
   radians = reg->leftAngle * (M_PI/DMTX_HOUGH_RES);
   rLeft.v.X = cosdp_i(radians);
   rLeft.v.Y = sindp_i(radians);
   rLeft.tMin = 0.0;
   rLeft.tMax = dmtxVector2Norm(&rLeft.v);

   /* Build ray representing bottom edge */
   rBottom.p.X = (double)reg->bottomLoc.X;
   rBottom.p.Y = (double)reg->bottomLoc.Y;
   radians = reg->bottomAngle * (M_PI/DMTX_HOUGH_RES);
   rBottom.v.X = cosdp_i(radians);
   rBottom.v.Y = sindp_i(radians);
   rBottom.tMin = 0.0;
   rBottom.tMax = dmtxVector2Norm(&rBottom.v);

   /* Build ray representing top edge */
   if(reg->topKnown != 0) {
      rTop.p.X = (double)reg->topLoc.X;
      rTop.p.Y = (double)reg->topLoc.Y;
      radians = reg->topAngle * (M_PI/DMTX_HOUGH_RES);
      rTop.v.X = cosdp_i(radians);
      rTop.v.Y = sindp_i(radians);
      rTop.tMin = 0.0;
      rTop.tMax = dmtxVector2Norm(&rTop.v);
   }
   else {
      rTop.p.X = (double)reg->locT.X;
      rTop.p.Y = (double)reg->locT.Y;
      radians = reg->bottomAngle * (M_PI/DMTX_HOUGH_RES);
      rTop.v.X = cosdp_i(radians);
      rTop.v.Y = sindp_i(radians);
      rTop.tMin = 0.0;
      rTop.tMax = rBottom.tMax;
   }

   /* Build ray representing right edge */
   if(reg->rightKnown != 0) {
      rRight.p.X = (double)reg->rightLoc.X;
      rRight.p.Y = (double)reg->rightLoc.Y;
      radians = reg->rightAngle * (M_PI/DMTX_HOUGH_RES);
      rRight.v.X = cosdp_i(radians);
      rRight.v.Y = sindp_i(radians);
      rRight.tMin = 0.0;
      rRight.tMax = dmtxVector2Norm(&rRight.v);
   }
   else {
      rRight.p.X = (double)reg->locR.X;
      rRight.p.Y = (double)reg->locR.Y;
      radians = reg->leftAngle * (M_PI/DMTX_HOUGH_RES);
      rRight.v.X = cosdp_i(radians);
      rRight.v.Y = sindp_i(radians);
      rRight.tMin = 0.0;
      rRight.tMax = rLeft.tMax;
   }

   /* Calculate 4 corners, real or imagined */
   if(dmtxRay2Intersect(&p00, &rLeft, &rBottom) == DmtxFail)	//根据线段的整型坐标来定直线，并不是非常准确
      return DmtxFail;

   if(dmtxRay2Intersect(&p10, &rBottom, &rRight) == DmtxFail)
      return DmtxFail;

   if(dmtxRay2Intersect(&p11, &rRight, &rTop) == DmtxFail)
      return DmtxFail;

   if(dmtxRay2Intersect(&p01, &rTop, &rLeft) == DmtxFail)
      return DmtxFail;

   if(dmtxRegionUpdateCorners(dec, reg, p00, p10, p11, p01) != DmtxPass)
      return DmtxFail;

   return DmtxPass;
}

/**
 *
 *
 */
/*static*/ double
RightAngleTrueness(DmtxVector2 c0, DmtxVector2 c1, DmtxVector2 c2, double angle)
{
   DmtxVector2 vA, vB;
   DmtxMatrix3 m;

   dmtxVector2Norm(dmtxVector2Sub(&vA, &c0, &c1));
   dmtxVector2Norm(dmtxVector2Sub(&vB, &c2, &c1));

   dmtxMatrix3Rotate(m, angle);
   dmtxMatrix3VMultiplyBy(&vA, m);

   return dmtxVector2Dot(&vA, &vB);
}

/**
 * \brief  Read color of Data Matrix module location
 * \param  dec
 * \param  reg
 * \param  symbolRow
 * \param  symbolCol
 * \param  sizeIdx
 * \return Averaged module color
 */
/*static*/ int
ReadModuleColor(DmtxDecode *dec, DmtxRegion *reg, int symbolRow, int symbolCol,
      int sizeIdx)
{
   //int err;
   int i;
   int symbolRows, symbolCols;
   int color, colorTmp;
   double sampleX[] = { 0.5, 0.4, 0.5, 0.6, 0.5 };
   double sampleY[] = { 0.5, 0.5, 0.4, 0.5, 0.6 };
   DmtxVector2 p;
   uint8_t* img;

   symbolRows = dmtxGetSymbolAttribute(DmtxSymAttribSymbolRows, sizeIdx);
   symbolCols = dmtxGetSymbolAttribute(DmtxSymAttribSymbolCols, sizeIdx);

   color = 0;
   img = dec->image->pxl;
   for(i = 0; i < 5; i++) {

      p.X = (_rcpdp((double)symbolCols)) * (symbolCol + sampleX[i]);
      p.Y = (_rcpdp((double)symbolRows)) * (symbolRow + sampleY[i]);

      dmtxMatrix3VMultiplyBy(&p, reg->fit2raw);

	  //dmtxDecodeGetPixelValue(dec->image, _dpint(p.X), _dpint(p.Y), &colorTmp);
	  colorTmp = img[_dpint(p.Y)*g_dm_proj_img_size+_dpint(p.X)];

      color += colorTmp;
   }

   return color/5;
}

/**
 * \brief  Determine barcode size, expressed in modules
 * \param  image
 * \param  reg
 * \return DmtxPass | DmtxFail
 */
/*static*/ DmtxPassFail
MatrixRegionFindSize(DmtxDecode *dec, DmtxRegion *reg)
{
   int row, col;
   int sizeIdxBeg, sizeIdxEnd;
   int sizeIdx, bestSizeIdx;
   int symbolRows, symbolCols;
   int jumpCount, errors;
   int color;
   int colorOnAvg, bestColorOnAvg;
   int colorOffAvg, bestColorOffAvg;
   int contrast, bestContrast;
   //DmtxImage *img;

#ifdef WATCH_MODE
   for(row = -1;row < 11;row++)
	   for(col = -1;col < 11;col++)
			barcode_res[(10-row)*12+col+1] = ReadModuleColor(dec, reg, row, col, 0);
#endif

   //img = dec->image;
   bestSizeIdx = DmtxUndefined;
   bestContrast = 0;
   bestColorOnAvg = bestColorOffAvg = 0;

   if(dec->sizeIdxExpected == DmtxSymbolShapeAuto) {
      sizeIdxBeg = 0;
      sizeIdxEnd = DmtxSymbolSquareCount + DmtxSymbolRectCount;
   }
   else if(dec->sizeIdxExpected == DmtxSymbolSquareAuto) {
      sizeIdxBeg = 0;
      sizeIdxEnd = DmtxSymbolSquareCount;
   }
   else if(dec->sizeIdxExpected == DmtxSymbolRectAuto) {
      sizeIdxBeg = DmtxSymbolSquareCount;
      sizeIdxEnd = DmtxSymbolSquareCount + DmtxSymbolRectCount;
   }
   else {
      sizeIdxBeg = dec->sizeIdxExpected;
      sizeIdxEnd = dec->sizeIdxExpected + 1;
   }

   /* Test each barcode size to find best contrast in calibration modules */
   for(sizeIdx = sizeIdxBeg; sizeIdx < sizeIdxEnd; sizeIdx++) {

      symbolRows = dmtxGetSymbolAttribute(DmtxSymAttribSymbolRows, sizeIdx);
      symbolCols = dmtxGetSymbolAttribute(DmtxSymAttribSymbolCols, sizeIdx);
      colorOnAvg = colorOffAvg = 0;

      /* Sum module colors along horizontal calibration bar */
      row = symbolRows - 1;
      for(col = 0; col < symbolCols; col++) {
         color = ReadModuleColor(dec, reg, row, col, sizeIdx);
         if((col & 0x01) != 0x00)
            colorOffAvg += color;
         else
            colorOnAvg += color;
      }

      /* Sum module colors along vertical calibration bar */
      col = symbolCols - 1;
      for(row = 0; row < symbolRows; row++) {
         color = ReadModuleColor(dec, reg, row, col, sizeIdx);
         if((row & 0x01) != 0x00)
            colorOffAvg += color;
         else
            colorOnAvg += color;
      }

      colorOnAvg = (colorOnAvg * 2)/(symbolRows + symbolCols);
      colorOffAvg = (colorOffAvg * 2)/(symbolRows + symbolCols);

      contrast = abs(colorOnAvg - colorOffAvg);
      if(contrast < 20)
         continue;

      if(contrast > bestContrast) {
         bestContrast = contrast;
         bestSizeIdx = sizeIdx;
         bestColorOnAvg = colorOnAvg;
         bestColorOffAvg = colorOffAvg;
      }
   }

   /* If no sizes produced acceptable contrast then call it quits */
   if(bestSizeIdx == DmtxUndefined || bestContrast < 20)
      return DmtxFail;

   reg->sizeIdx = bestSizeIdx;
   reg->onColor = bestColorOnAvg;
   reg->offColor = bestColorOffAvg;

   reg->symbolRows = dmtxGetSymbolAttribute(DmtxSymAttribSymbolRows, reg->sizeIdx);
   reg->symbolCols = dmtxGetSymbolAttribute(DmtxSymAttribSymbolCols, reg->sizeIdx);
   reg->mappingRows = dmtxGetSymbolAttribute(DmtxSymAttribMappingMatrixRows, reg->sizeIdx);
   reg->mappingCols = dmtxGetSymbolAttribute(DmtxSymAttribMappingMatrixCols, reg->sizeIdx);

   /* Tally jumps on horizontal calibration bar to verify sizeIdx */
   jumpCount = CountJumpTally(dec, reg, 0, reg->symbolRows - 1, DmtxDirRight);
   errors = abs(1 + jumpCount - reg->symbolCols);
   if(jumpCount < 0 || errors > 2)
      return DmtxFail;

   /* Tally jumps on vertical calibration bar to verify sizeIdx */
   jumpCount = CountJumpTally(dec, reg, reg->symbolCols - 1, 0, DmtxDirUp);
   errors = abs(1 + jumpCount - reg->symbolRows);
   if(jumpCount < 0 || errors > 2)
      return DmtxFail;

   /* Tally jumps on horizontal finder bar to verify sizeIdx */
   errors = CountJumpTally(dec, reg, 0, 0, DmtxDirRight);
   if(jumpCount < 0 || errors > 2)
      return DmtxFail;

   /* Tally jumps on vertical finder bar to verify sizeIdx */
   errors = CountJumpTally(dec, reg, 0, 0, DmtxDirUp);
   if(errors < 0 || errors > 2)
      return DmtxFail;

   /* Tally jumps on surrounding whitespace, else fail */
   errors = CountJumpTally(dec, reg, 0, -1, DmtxDirRight);
   if(errors < 0 || errors > 2)
      return DmtxFail;

   errors = CountJumpTally(dec, reg, -1, 0, DmtxDirUp);
   if(errors < 0 || errors > 2)
      return DmtxFail;

   errors = CountJumpTally(dec, reg, 0, reg->symbolRows, DmtxDirRight);
   if(errors < 0 || errors > 2)
      return DmtxFail;

   errors = CountJumpTally(dec, reg, reg->symbolCols, 0, DmtxDirUp);
   if(errors < 0 || errors > 2)
      return DmtxFail;

   return DmtxPass;
}

/**
 * \brief  Count the number of number of transitions between light and dark
 * \param  img
 * \param  reg
 * \param  xStart
 * \param  yStart
 * \param  dir
 * \return Jump count
 */
/*static*/ int
CountJumpTally(DmtxDecode *dec, DmtxRegion *reg, int xStart, int yStart, DmtxDirection dir)
{
   int x, xInc = 0;
   int y, yInc = 0;
   int state = DmtxModuleOn;
   int jumpCount = 0;
   int jumpThreshold;
   int tModule, tPrev;
   int darkOnLight;
   int color;

   assert(xStart == 0 || yStart == 0);
   assert(dir == DmtxDirRight || dir == DmtxDirUp);

   if(dir == DmtxDirRight)
      xInc = 1;
   else
      yInc = 1;

   if(xStart == -1 || xStart == reg->symbolCols ||
         yStart == -1 || yStart == reg->symbolRows)
      state = DmtxModuleOff;

   darkOnLight = (int)(reg->offColor > reg->onColor);
   jumpThreshold = abs(_spint(0.4 * (reg->onColor - reg->offColor)));
   color = ReadModuleColor(dec, reg, yStart, xStart, reg->sizeIdx);
   tModule = (darkOnLight) ? reg->offColor - color : color - reg->offColor;

   for(x = xStart + xInc, y = yStart + yInc;
         (dir == DmtxDirRight && x < reg->symbolCols) ||
         (dir == DmtxDirUp && y < reg->symbolRows);
         x += xInc, y += yInc) {

      tPrev = tModule;
      color = ReadModuleColor(dec, reg, y, x, reg->sizeIdx);
      tModule = (darkOnLight) ? reg->offColor - color : color - reg->offColor;

      if(state == DmtxModuleOff) {
         if(tModule > tPrev + jumpThreshold) {
            jumpCount++;
            state = DmtxModuleOn;
         }
      }
      else {
         if(tModule < tPrev - jumpThreshold) {
            jumpCount++;
            state = DmtxModuleOff;
         }
      }
   }

   return jumpCount;
}

/**
 *
 *获取参考点的梯度量以及梯度方向
 */
/*static*/ DmtxPassFail
GetPointFlow(DmtxDecode *dec, DmtxPixelLoc loc, DmtxImgInfo* cache)
{
   static const int coefficient[] = {  0,  1,  2,  1,  0, -1, -2, -1 };
   //DmtxPassFail err;
   int patternIdx, coefficientIdx;
   int compass, compassMax;
   int mag[4] = { 0 };
   int xAdjust, yAdjust;
   int color, colorPattern[8];
   uint8_t* img;

   if(cache->depart & 0x80)
   {
	   /*flow.plane = colorPlane;
	   flow.arrive = arrive;
	   flow.depart = cache->depart & 0x07;
	   flow.mag = cache->grad;
	   flow.loc = loc;*/

	   return DmtxPass;
   }

   if(loc.X < 1 || loc.X > g_dm_proj_img_size-2 || loc.Y < 1 || loc.Y > g_dm_proj_img_size-2)
	   return DmtxFail;

   img = dec->image->pxl;
   //提取参考点周围8个点的像素值
#pragma MUST_ITERATE(8, , 8)
   for(patternIdx = 0; patternIdx < 8; patternIdx++) {
      xAdjust = loc.X + dmtxPatternX[patternIdx];
      yAdjust = loc.Y + dmtxPatternY[patternIdx];
      //err = dmtxDecodeGetPixelValue(dec->image, xAdjust, yAdjust, &colorPattern[patternIdx]);
      /*if(err == DmtxFail)
         return DmtxFail;*/

      colorPattern[patternIdx] = img[yAdjust*g_dm_proj_img_size+xAdjust];
   }

   /* Calculate this pixel's flow intensity for each direction (-45, 0, 45, 90) */
   //计算参考点四个方向的梯度量
   compassMax = 0;
   for(compass = 0; compass < 4; compass++) {

      /* Add portion from each position in the convolution matrix pattern */
      for(patternIdx = 0; patternIdx < 8; patternIdx++) {

         coefficientIdx = (patternIdx - compass + 8) % 8;
         if(coefficient[coefficientIdx] == 0)
            continue;

         color = colorPattern[patternIdx];

         switch(coefficient[coefficientIdx]) {
            case 2:
               mag[compass] += color;
               /* Fall through */
            case 1:
               mag[compass] += color;
               break;
            case -2:
               mag[compass] -= color;
               /* Fall through */
            case -1:
               mag[compass] -= color;
               break;
         }
      }

      /* Identify strongest compass flow */
      if(compass != 0 && abs(mag[compass]) > abs(mag[compassMax]))
         compassMax = compass;
   }

   /* Convert signed compass direction into unique flow directions (0-7) */
   /*flow.plane = colorPlane;
   flow.arrive = arrive;
   flow.depart = (mag[compassMax] > 0) ? compassMax + 4 : compassMax;
   flow.mag = abs(mag[compassMax]);
   flow.loc = loc;*/

   cache->depart = (mag[compassMax] > 0) ? compassMax + 4 : compassMax;
   cache->depart |= 0x80;
   cache->grad = (int16_t)abs(mag[compassMax]);

   return DmtxPass;
}

/**
 *根据参考点的梯度方向在三个领域点中搜索梯度最强点的位置以及其梯度量与梯度方向
 *
 */
/*static*/ DmtxPassFail
	FindStrongestNeighbor(DmtxDecode *dec, DmtxImgInfo* center_cache, DmtxPixelLoc center_loc, int sign, DmtxImgInfo** max_cache, DmtxPixelLoc* max_loc)
{
	DmtxPassFail err;
   int i;
   //int strongIdx;
   int attempt, attemptDiff;
   int occupied;
   DmtxImgInfo *cache;
   DmtxPixelLoc loc;
   uint16_t arrive;

   int depart;

   DmtxImgInfo* m_cache;
   DmtxPixelLoc m_loc;

#ifndef FULL_PIC_PROJECT
   if(g_camera_id == 0)
   {
	   if(sign > 0)
		   arrive = center_cache->arrive;
	   else
		   arrive = center_cache->arrive_;
	   if(arrive & 0x8000)
	   {
		   max_loc->X = arrive & 0x7F;
		   max_loc->Y = (arrive & 0x3F80)>>7;

		   cache = dmtxDecodeGetCache(dec, max_loc->X, max_loc->Y);
		   if(cache->visited == 1)
			   return DmtxFail;

		   *max_cache = cache;
		   return DmtxPass;
	   }
   }
#endif

   depart = center_cache->depart & 0x07;
   attempt = (sign < 0) ? depart : (depart+4)%8;

   occupied = 0;
   //strongIdx = DmtxUndefined;
   m_cache = NULL;

   for(i = 0; i < 8; i++) {
	   //生成参考点周围8个点的坐标
      loc.X = center_loc.X + dmtxPatternX[i];
      loc.Y = center_loc.Y + dmtxPatternY[i];

      cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
	  assert(cache != NULL);
      /*if(cache == NULL)
         continue;*/

	  //8邻域中有两个已访问过的点，则结束搜索
      if(cache->visited == 1) {
         if(++occupied > 2)
            return DmtxFail;
         else
            continue;
      }

	  //根据梯度方向与正负方向决定需要考察的三个周围点
      attemptDiff = abs(attempt - i);
      if(attemptDiff > 4)
         attemptDiff = 8 - attemptDiff;
      if(attemptDiff > 1)
         continue;

      err = GetPointFlow(dec, loc, cache);
	  if(err == DmtxFail)
		  continue;

      if(m_cache == NULL || cache->grad > m_cache->grad || (cache->grad == m_cache->grad && ((i & 0x01) != 0))) 
	  {
         m_cache = cache;
		 m_loc = loc;
      }
   }

   if(m_cache == NULL)
	   return DmtxFail;
   else
   {
	   if(sign > 0)
		   center_cache->arrive = m_loc.X+(m_loc.Y<<7)+0x8000;
	   else
		   center_cache->arrive_ = m_loc.X+(m_loc.Y<<7)+0x8000;

	   *max_cache = m_cache;
	   *max_loc = m_loc;
   }

   return DmtxPass;
}

/**搜索轮廓
 * vaiiiooo
 * --------
 * 0x80 v = visited bit
 * 0x38 u = 3 bits points upstream 0-7
 * 0x07 d = 3 bits points downstream 0-7
 */
/*static*/ DmtxPassFail
TrailBlazeContinuous(DmtxDecode *dec, DmtxRegion *reg, DmtxPixelLoc locBegin, int maxDiagonal)
{
	DmtxPassFail err;
   int sign, cur_index;
   int steps;
   DmtxImgInfo *cache, *cacheNext, *cacheBeg;
   //DmtxPointFlow flow, flowNext;
   DmtxPixelLoc boundMin, boundMax, loc, locNext;
   DmtxPixelLoc* outline;

   uint8_t departBeg, cur_depart, tmp_depart, vert_depart;

   boundMin = boundMax = locBegin;
   cacheBeg = dmtxDecodeGetCache(dec, locBegin.X, locBegin.Y);
   if(cacheBeg == NULL)
      return DmtxFail;
   cacheBeg->visited = 1; /* Mark location as visited and assigned */

   departBeg = cacheBeg->depart & 0x07;

   //reg->flowBegin = flowBegin;
   
   outline = reg->outline;
   outline[0] = locBegin;

   for(sign = 1; sign >= -1; sign -= 2) {

	   loc = locBegin;
      //flow = flowBegin;
      cache = cacheBeg;

	  vert_depart = 8;

	  cur_index = sign > 0 ? 1 : OUTLINE_POINT_NUM-1;

      for(steps = 0; ; steps++) {

         if(maxDiagonal != DmtxUndefined && (boundMax.X - boundMin.X > maxDiagonal ||
               boundMax.Y - boundMin.Y > maxDiagonal))
            break;

         //tic();
         /* Find the strongest eligible neighbor */
		 //根据参考点的梯度方向在三个领域点中搜索梯度最强点的位置以及其梯度量与梯度方向
         err = FindStrongestNeighbor(dec, cache, loc, sign, &cacheNext, &locNext);
         //toc("");
		 //若轮廓下一个点的梯度量小于50，则截止
		 if(err == DmtxFail || cacheNext->grad < 50)
            break;

         /* Get the neighbor's cache location */
         //cacheNext = dmtxDecodeGetCache(dec, flowNext.loc.X, flowNext.loc.Y);

        /* if(cacheNext == NULL)
            break;*/

		 cur_depart = cacheNext->depart & 0x07;
		 tmp_depart = (cur_depart+8-departBeg)%8;
		 if(tmp_depart > 4)
			 tmp_depart = 8-tmp_depart;
		 if(tmp_depart == 4)
			 break;
		 else if(tmp_depart == 3 && cur_depart != vert_depart)
		 {
			 if(vert_depart == 8)
				 vert_depart = cur_depart;
			 else
				 break;
		 }

		 assert(!(cacheNext->visited));

         /* Mark departure from current location. If flowing downstream
          * (sign < 0) then departure vector here is the arrival vector
          * of the next location. Upstream flow uses the opposite rule. */
         //cache->outline |= (sign < 0) ? flowNext.arrive : flowNext.arrive << 3;

         /* Mark known direction for next location */
         /* If testing downstream (sign < 0) then next upstream is opposite of next arrival */
         /* If testing upstream (sign > 0) then next downstream is opposite of next arrival */
         //cacheNext->outline = (sign < 0) ? (((flowNext.arrive + 4)%8) << 3) : ((flowNext.arrive + 4)%8);
         cacheNext->visited = 1; /* Mark location as visited and assigned */

         cache = cacheNext;
         outline[cur_index] = loc = locNext;
		 cur_index += sign;
		 if(cur_index > OUTLINE_POINT_NUM-1 || cur_index < 0)
			 break;

		 //更新轮廓的外接矩形
         if(loc.X > boundMax.X)
            boundMax.X = loc.X;
         else if(loc.X < boundMin.X)
            boundMin.X = loc.X;
         if(loc.Y > boundMax.Y)
            boundMax.Y = loc.Y;
         else if(loc.Y < boundMin.Y)
            boundMin.Y = loc.Y;

#ifdef	WATCH_MODE
		 img_watch[loc.Y*DM_PROJ_IMG_SIZE + loc.X] = 255;
#endif
/*       CALLBACK_POINT_PLOT(flow.loc, (sign > 0) ? 2 : 3, 1, 2); */
      }

      if(sign > 0) {
		  reg->outline_pos = cur_index-1;
         //reg->finalPos = flow.loc;
         //reg->jumpToNeg = steps;
      }
      else {
		  reg->outline_neg = cur_index+1;
         //reg->finalNeg = flow.loc;
         //reg->jumpToPos = steps;
      }
   }
   //assert(reg->outline_pos < reg->outline_neg);
   reg->stepsTotal = reg->outline_pos + 1 + OUTLINE_POINT_NUM - reg->outline_neg;
   reg->boundMin = boundMin;
   reg->boundMax = boundMax;

   /* Clear "visited" bit from trail */
   //清除0x80标志
   TrailClear(dec, reg);

#ifdef WATCH_MODE
   memcpy(img_watch, dec->image->pxl, DM_PROJ_IMG_SIZE*DM_PROJ_IMG_SIZE);
#endif // WATCH_MODE

   if(reg->outline_pos >= reg->outline_neg)
	   return DmtxFail;

   /* XXX clean this up ... redundant test above */
   if(maxDiagonal != DmtxUndefined && (boundMax.X - boundMin.X > maxDiagonal ||
         boundMax.Y - boundMin.Y > maxDiagonal))
      return DmtxFail;

   /*if(dec->edgeMin != DmtxUndefined && (boundMax.X - boundMin.X < dec->edgeMin ||
         boundMax.Y - boundMin.Y < dec->edgeMin))
      return DmtxFail;*/

   return DmtxPass;
}

/**
 * recives bresline, and follows strongest neighbor unless it involves
 * ratcheting bresline inward or backward (although back + outward is allowed).
 *
 */
/*static*/ int
TrailBlazeGapped(DmtxDecode *dec, DmtxRegion *reg, DmtxBresLine line, int streamDir)
{
	DmtxPassFail err;
   DmtxImgInfo *beforeCache, *afterCache, *cache, *cacheNext;
   DmtxBoolean onEdge;
   int distSq, distSqMax;
   int travel, outward;
   int xDiff, yDiff;
   int steps;
   //int stepDir, dirMap[] = { 0, 1, 2, 7, 8, 3, 6, 5, 4 };
   //DmtxPassFail err;
   DmtxPixelLoc beforeStep, afterStep;
   //DmtxPointFlow flow, flowNext;
   DmtxPixelLoc loc0, loc, locNext;
   //int xStep, yStep;
   DmtxPixelLoc* outline;
   uint16_t valid_point;

   outline = reg->outline;

   loc = loc0 = line.loc;
   beforeStep = loc0;
   cache = beforeCache = dmtxDecodeGetCache(dec, loc0.X, loc0.Y);
   assert(beforeCache != NULL);
   beforeCache->visited = 0x00; /* probably should just overwrite one direction */

   err = GetPointFlow(dec, loc0, beforeCache);
   if(err == DmtxFail)
	   return 0;
   distSqMax = (line.xDelta * line.xDelta) + (line.yDelta * line.yDelta);
   steps = 0;
   onEdge = DmtxTrue; 

   do {
	   valid_point = 0;
      if(onEdge == DmtxTrue) {
         err = FindStrongestNeighbor(dec, cache, loc, streamDir, &cacheNext, &locNext);
		 if(err == DmtxFail)
            break;

         BresLineGetStep(line, locNext, &travel, &outward);
		 if(cacheNext->grad < 50 || outward < 0 || (outward == 0 && travel < 0)) {
            onEdge = DmtxFalse;
         }
         else {
			 valid_point = 1;
            BresLineStep(&line, travel, outward);
            //flow = flowNext;
			loc = locNext;
			cache = cacheNext;
         }
      }

      if(onEdge == DmtxFalse) {
         BresLineStep(&line, 1, 0);

		 loc = line.loc;
		 cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
		 if(cache == NULL)
			 break;

         err = GetPointFlow(dec, loc, cache);
		 if(err == DmtxPass && cache->grad > 50)
            onEdge = DmtxTrue;
      }

      afterStep = line.loc;
      afterCache = dmtxDecodeGetCache(dec, afterStep.X, afterStep.Y);
      if(afterCache == NULL)
         break;

      /* Determine step direction using pure magic */
      //xStep = afterStep.X - beforeStep.X;
      //yStep = afterStep.Y - beforeStep.Y;
      //assert(abs(xStep) <= 1 && abs(yStep) <= 1);
      //stepDir = dirMap[3 * yStep + xStep + 4];
      //assert(stepDir != 8);

      /*if(streamDir < 0) {
		  beforeCache->outline |= stepDir;
		  afterCache->outline = (((stepDir + 4)%8) << 3);
      }
      else {
         beforeCache->outline |= (stepDir << 3);
         afterCache->outline = ((stepDir + 4)%8);
      }*/

      /* Guaranteed to have taken one step since top of loop */
      xDiff = line.loc.X - loc0.X;
      yDiff = line.loc.Y - loc0.Y;
      distSq = (xDiff * xDiff) + (yDiff * yDiff);

	  if(valid_point == 1)
	  {
		 beforeStep = line.loc;
		 beforeCache = afterCache;
		 steps++;

		outline[steps] = beforeStep;

#ifdef	WATCH_MODE
		 img_watch[beforeStep.Y*DM_PROJ_IMG_SIZE + beforeStep.X] = 255;
#endif
	  }

   } while(distSq < distSqMax);

#ifdef WATCH_MODE
   memcpy(img_watch, dec->image->pxl, DM_PROJ_IMG_SIZE*DM_PROJ_IMG_SIZE);
#endif // WATCH_MODE

   return steps;
}

/**
 *
 *
 */
/*static*/ void
TrailClear(DmtxDecode *dec, DmtxRegion *reg)
{
	DmtxPixelLoc* outline;
	DmtxPixelLoc loc;
	uint16_t neg_p, pos_p;
	DmtxImgInfo* cache;
	int i;

	outline = reg->outline;
	pos_p = reg->outline_pos+1;
	neg_p = reg->outline_neg;

	for(i = 0;i < pos_p;i++)
	{
		loc = outline[i];
		cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
		cache->visited = 0;
	}

	for(i = neg_p;i < OUTLINE_POINT_NUM;i++)
	{
		loc = outline[i];
		cache = dmtxDecodeGetCache(dec, loc.X, loc.Y);
		cache->visited = 0;
	}

   /* Clear "visited" bit from trail */
   /*clears = 0;
   follow = FollowSeek(dec, reg, 0);
   while(abs(follow.step) <= reg->stepsTotal) {
	   assert((int)(follow.ptr->outline & clearMask) != 0x00);
      follow.ptr->outline &= (clearMask ^ 0xff);
      follow = FollowStep(dec, reg, follow, +1);
      clears++;
   }*/

   return;
}

/**
 *streamDir为0时搜索全部，为-1时往neg方向搜索，为1时往pos方向搜索，其中，step0为搜索起始点
 *is_Avoid不为-1时表示当前检索的是line1x，否则检索的不是line1x
 */
/*static*/ DmtxBestLine
FindBestSolidLine_angleLimit(DmtxDecode *dec, DmtxRegion *reg, int step0, int streamDir, int houghAvoid, int is_Avoid)
{
   int hough[3][DMTX_HOUGH_RES] = { { 0 } };
   //int houghMin, houghMax;
   //char houghTest[DMTX_HOUGH_RES];
   int i, j;
   int step;
   int angleBest;
   int hOffset, hOffsetBest;
   int xDiff, yDiff;
   int dH;
   int start_step1, end_step1, start_step2, end_step2;
   DmtxRay2 rH;
   //DmtxFollow follow;
   DmtxBestLine line;
   DmtxPixelLoc rHp;
   DmtxPixelLoc* outline;

   int start_angle[3], end_angle[3], start_ang, end_ang;

   outline = reg->outline;

   memset(&line, 0x00, sizeof(DmtxBestLine));
   memset(&rH, 0x00, sizeof(DmtxRay2));
   angleBest = 0;
   hOffset = hOffsetBest = 0;

   /* Always follow path flowing away from the trail start */
   if(streamDir == 0)
   {
		start_step1 = 0;
		end_step1 = reg->outline_pos+1;
		
		start_step2 = reg->outline_neg;
		end_step2 = OUTLINE_POINT_NUM;
   }
   else if(streamDir == -1)
   {
	    assert(step0 > reg->outline_neg);
		start_step1 = end_step1 = 0;
		   
		start_step2 = reg->outline_neg;
		end_step2 = step0; 
   }
   else
   {
	    assert(step0 < reg->outline_pos);
		start_step1 = step0;
		end_step1 = reg->outline_pos+1;

		start_step2 = end_step2 = OUTLINE_POINT_NUM;
   }

   //follow = FollowSeek(dec, reg, step0);
   //rHp = follow.loc;
   rHp = outline[step0];

   line.stepBeg = line.stepPos = line.stepNeg = step0;
   line.locBeg = rHp;
   line.locPos = rHp;
   line.locNeg = rHp;

   /* Predetermine which angles to test */
   if(houghAvoid == DmtxUndefined)
   {
	   //搜索全范围
	   start_angle[0] = 0;
	   end_angle[0] = DMTX_HOUGH_RES;
	   start_angle[1] = end_angle[1] = start_angle[2] = end_angle[2] = 0;
   }
   else if(is_Avoid == DmtxUndefined)
   {
	   //仅搜索与之垂直的30度范围
	   if(houghAvoid <= (90-ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = houghAvoid+(90-ANGLE_SEARCH_RANGE);
		   end_angle[0] = houghAvoid+(90+ANGLE_SEARCH_RANGE);
		   start_angle[1] = end_angle[1] = 0;
	   }
	   else if(houghAvoid <= (90+ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = 0;
		   end_angle[0] = houghAvoid+(ANGLE_SEARCH_RANGE-90);

		   start_angle[1] = houghAvoid+(90-ANGLE_SEARCH_RANGE);
		   end_angle[1] = DMTX_HOUGH_RES;
	   }
	   else
	   {
		   start_angle[0] = houghAvoid-(90+ANGLE_SEARCH_RANGE);
		   end_angle[0] = houghAvoid+(ANGLE_SEARCH_RANGE-90);
		   start_angle[1] = end_angle[1] = 0;
	   }
	   start_angle[2] = end_angle[2] = 0;
   }
   else
   {
	   //仅搜索30度范围以及与之垂直的30度范围
	   if(houghAvoid <= ANGLE_SEARCH_RANGE)
	   {
		   start_angle[0] = 0;
		   end_angle[0] = houghAvoid+ANGLE_SEARCH_RANGE;
		   
		   start_angle[1] = houghAvoid+(90-ANGLE_SEARCH_RANGE);
		   end_angle[1] = houghAvoid+(90+ANGLE_SEARCH_RANGE);

		   start_angle[2] = houghAvoid+(DMTX_HOUGH_RES-ANGLE_SEARCH_RANGE);
		   end_angle[2] = DMTX_HOUGH_RES;
	   }
	   else if(houghAvoid <= (90-ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = houghAvoid-ANGLE_SEARCH_RANGE;
		   end_angle[0] = houghAvoid+ANGLE_SEARCH_RANGE;

		   start_angle[1] = houghAvoid+(90-ANGLE_SEARCH_RANGE);
		   end_angle[1] = houghAvoid+(90+ANGLE_SEARCH_RANGE);

		   start_angle[2] = end_angle[2] = 0;
	   }
	   else if(houghAvoid <= (90+ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = 0;
		   end_angle[0] = houghAvoid+(ANGLE_SEARCH_RANGE-90);

		   start_angle[1] = houghAvoid-ANGLE_SEARCH_RANGE;
		   end_angle[1] = houghAvoid+ANGLE_SEARCH_RANGE;

		   start_angle[2] = houghAvoid+(90-ANGLE_SEARCH_RANGE);
		   end_angle[2] = DMTX_HOUGH_RES;
	   }
	   else if(houghAvoid <= (DMTX_HOUGH_RES-ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = houghAvoid-(90+ANGLE_SEARCH_RANGE);
		   end_angle[0] = houghAvoid+(ANGLE_SEARCH_RANGE-90);

		   start_angle[1] = houghAvoid-ANGLE_SEARCH_RANGE;
		   end_angle[1] = houghAvoid+ANGLE_SEARCH_RANGE;

		   start_angle[2] = end_angle[2] = 0;
	   }
	   else
	   {
		   start_angle[0] = 0;
		   end_angle[0] = houghAvoid+(ANGLE_SEARCH_RANGE-DMTX_HOUGH_RES);

		   start_angle[1] = houghAvoid-(90+ANGLE_SEARCH_RANGE);
		   end_angle[1] = houghAvoid+(ANGLE_SEARCH_RANGE-90);

		   start_angle[2] = houghAvoid-ANGLE_SEARCH_RANGE;
		   end_angle[2] = DMTX_HOUGH_RES;
	   }
   }

   /*for(i = 0; i < DMTX_HOUGH_RES; i++) {
      if(houghAvoid == DmtxUndefined) {
         houghTest[i] = 1;
      }
      else if(is_Avoid == DmtxUndefined)
	  {
		 houghMin = (houghAvoid + ANGLE_VETICAL_RANGE) % DMTX_HOUGH_RES;
         houghMax = (houghAvoid - ANGLE_VETICAL_RANGE + DMTX_HOUGH_RES) % DMTX_HOUGH_RES;
         if(houghMin > houghMax)
            houghTest[i] = (i > houghMin || i < houghMax) ? 1 : 0;
         else
            houghTest[i] = (i > houghMin && i < houghMax) ? 1 : 0;
      }
	  else
	  {
		  houghMin = (houghAvoid - ANGLE_SEARCH_RANGE + 90) % 90;
		  houghMax = (houghAvoid + ANGLE_SEARCH_RANGE) % 90;
		  tmp_i = i%90;
		  if(houghMin < houghMax)
			  houghTest[i] = (tmp_i > houghMin && tmp_i < houghMax) ? 1 : 0;
		  else
			  houghTest[i] = (tmp_i > houghMin || tmp_i < houghMax) ? 1 : 0;
	  }
   }*/

   /* Test each angle for steps along path */
   for(step = start_step1; step < end_step1; step++) {
      xDiff = outline[step].X - rHp.X;
      yDiff = outline[step].Y - rHp.Y;

      /* Increment Hough accumulator */
      //for(i = 0; i < DMTX_HOUGH_RES; i++) {
	  for(j = 0; j < 3;j++)
	  {
		    start_ang = start_angle[j];
			end_ang = end_angle[j];
			for(i = start_ang; i < end_ang;i++)
			{
				/*if((int)houghTest[i] == 0)
				continue;*/

				dH = (rHvX[i] * yDiff) - (rHvY[i] * xDiff);	//此处若除了考察点与起始点的距离应该会效果更佳(2015.9.24)
				if(dH >= -384 && dH <= 384) {

				if(dH > 128)
					hOffset = 2;
				else if(dH >= -128)
					hOffset = 1;
				else
					hOffset = 0;

				hough[hOffset][i]++;

				/* New angle takes over lead */
				if(hough[hOffset][i] > hough[hOffsetBest][angleBest]) {
					angleBest = i;
					hOffsetBest = hOffset;
					}
				}
			}
		  }

/*    CALLBACK_POINT_PLOT(follow.loc, (sign > 1) ? 4 : 3, 1, 2); */

      //follow = FollowStep(dec, reg, follow, sign);
   }

   for(step = start_step2; step < end_step2; step++) {
      xDiff = outline[step].X - rHp.X;
      yDiff = outline[step].Y - rHp.Y;

      /* Increment Hough accumulator */
      /*for(i = 0; i < DMTX_HOUGH_RES; i++) {

         if((int)houghTest[i] == 0)
            continue;*/
	  for(j = 0; j < 3;j++)
	  {
				start_ang = start_angle[j];
				end_ang = end_angle[j];
				for(i = start_ang; i < end_ang;i++)
				{

				dH = (rHvX[i] * yDiff) - (rHvY[i] * xDiff);	//此处若除了考察点与起始点的距离应该会效果更佳(2015.9.24)
				if(dH >= -384 && dH <= 384) {

				if(dH > 128)
					hOffset = 2;
				else if(dH >= -128)
					hOffset = 1;
				else
					hOffset = 0;

				hough[hOffset][i]++;

				/* New angle takes over lead */
				if(hough[hOffset][i] > hough[hOffsetBest][angleBest]) {
					angleBest = i;
					hOffsetBest = hOffset;
				}
				}
			}
	  }

/*    CALLBACK_POINT_PLOT(follow.loc, (sign > 1) ? 4 : 3, 1, 2); */

      //follow = FollowStep(dec, reg, follow, sign);
   }

   line.angle = angleBest;
   line.hOffset = hOffsetBest;
   line.mag = hough[hOffsetBest][angleBest];

   return line;
}

/**
 *
 *
 */
/*static*/ DmtxBestLine
FindBestSolidLine2(DmtxDecode *dec, DmtxRegion *reg, int tripSteps, int sign, int houghAvoid)
{
   int hough[3][DMTX_HOUGH_RES] = { { 0 } };
   //int houghMin, houghMax;
   //char houghTest[DMTX_HOUGH_RES];
   int i, j;
   int step;
   int angleBest;
   int hOffset, hOffsetBest;
   int xDiff, yDiff;
   int dH;
   DmtxRay2 rH;
   DmtxBestLine line;
   DmtxPixelLoc rHp;
   //DmtxFollow follow;
   DmtxPixelLoc* outline;

   int start_angle[2], end_angle[2], start_ang, end_ang;

   memset(&line, 0x00, sizeof(DmtxBestLine));
   memset(&rH, 0x00, sizeof(DmtxRay2));
   angleBest = 0;
   hOffset = hOffsetBest = 0;

   outline = reg->outline;

   /* Predetermine which angles to test */
   if(houghAvoid == DmtxUndefined)
   {
	   //搜索全范围
	   start_angle[0] = 0;
	   end_angle[0] = DMTX_HOUGH_RES;
	   start_angle[1] = end_angle[1] = 0;
   }
   else
   {
	   //仅搜索与之垂直的4度范围
	   if(houghAvoid <= (90-VERTICAL_ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = houghAvoid+(90-VERTICAL_ANGLE_SEARCH_RANGE);
		   end_angle[0] = houghAvoid+(90+VERTICAL_ANGLE_SEARCH_RANGE);
		   start_angle[1] = end_angle[1] = 0;
	   }
	   else if(houghAvoid <= (90+VERTICAL_ANGLE_SEARCH_RANGE))
	   {
		   start_angle[0] = 0;
		   end_angle[0] = houghAvoid+(VERTICAL_ANGLE_SEARCH_RANGE-90);

		   start_angle[1] = houghAvoid+(90-VERTICAL_ANGLE_SEARCH_RANGE);
		   end_angle[1] = DMTX_HOUGH_RES;
	   }
	   else
	   {
		   start_angle[0] = houghAvoid-(90+VERTICAL_ANGLE_SEARCH_RANGE);
		   end_angle[0] = houghAvoid+(VERTICAL_ANGLE_SEARCH_RANGE-90);
		   start_angle[1] = end_angle[1] = 0;
	   }
   }

	line.locBeg = line.locPos = line.locNeg = rHp = outline[0];
	line.stepBeg = line.stepPos = line.stepNeg = 0;

	/* Test each angle for steps along path */
	for(step = 0; step < tripSteps; step++) {

		xDiff = outline[step].X - rHp.X;
		yDiff = outline[step].Y - rHp.Y;

		/* Increment Hough accumulator */
		/*for(i = 0; i < DMTX_HOUGH_RES; i++) {

			if((int)houghTest[i] == 0)
			continue;*/

		for(j = 0; j < 2;j++)
		{
			start_ang = start_angle[j];
			end_ang = end_angle[j];
			for(i = start_ang; i < end_ang;i++)
			{

				dH = (rHvX[i] * yDiff) - (rHvY[i] * xDiff);
				if(dH >= -384 && dH <= 384) {
				if(dH > 128)
					hOffset = 2;
				else if(dH >= -128)
					hOffset = 1;
				else
					hOffset = 0;

				hough[hOffset][i]++;

				/* New angle takes over lead */
				if(hough[hOffset][i] > hough[hOffsetBest][angleBest]) {
					angleBest = i;
					hOffsetBest = hOffset;
				}
				}
			}
		}

/*    CALLBACK_POINT_PLOT(follow.loc, (sign > 1) ? 4 : 3, 1, 2); */

		//follow = FollowStep2(dec, follow, sign);
	}

	line.angle = angleBest;
	line.hOffset = hOffsetBest;
	line.mag = hough[hOffsetBest][angleBest];

   return line;
}

DmtxPassFail FindTravelLimits3(DmtxDecode *dec, DmtxRegion *reg, DmtxBestLine *line, DmtxPixelLoc *line_point, uint16_t *start, uint16_t *end)
{
   int i;
   int distSq, distSqMax;
   int xDiff, yDiff;
   int posRunning, negRunning;
   int posTravel, negTravel;
   int posWander, posWanderMin, posWanderMax, posWanderMinLock, posWanderMaxLock;
   int negWander, negWanderMin, negWanderMax, negWanderMinLock, negWanderMaxLock;
   int cosAngle, sinAngle;
   //DmtxFollow followPos, followNeg;
   DmtxPixelLoc loc0, posMax, negMax, locPos, locNeg;
   DmtxPixelLoc* outline;

   int stepPos, stepNeg, pos_end, neg_end;

   int posCurLoc = 75, negCurLoc = 74;

   /* line->stepBeg is already known to sit on the best Hough line */
   //followPos = followNeg = FollowSeek(dec, reg, line->stepBeg);
   //loc0 = followPos.loc;
   outline = reg->outline;
   line->stepNeg = stepPos = stepNeg = line->stepBeg;
   line->locNeg = posMax = negMax = locPos = locNeg = loc0 = outline[line->stepBeg];

   cosAngle = rHvX[line->angle];
   sinAngle = rHvY[line->angle];

   distSqMax = -1;
   //posMax = negMax = followPos.loc;
   pos_end = reg->outline_pos;
   neg_end = reg->outline_neg == OUTLINE_POINT_NUM ? 0:reg->outline_neg;

   posTravel = negTravel = 0;
   posWander = posWanderMin = posWanderMax = posWanderMinLock = posWanderMaxLock = 0;
   negWander = negWanderMin = negWanderMax = negWanderMinLock = negWanderMaxLock = 0;

	posRunning = negRunning = 1;

   for(i = 0; i < reg->stepsTotal; i++) {         
	  
      if(posRunning != 0) {
		  locPos = outline[stepPos];

         xDiff = locPos.X - loc0.X;
         yDiff = locPos.Y - loc0.Y;
         posTravel = (cosAngle * xDiff) + (sinAngle * yDiff);
         posWander = (cosAngle * yDiff) - (sinAngle * xDiff);

         if(posWander >= -3*256 && posWander <= 3*256) {
            distSq = DistanceSquared(locPos, negMax);
            if(distSq > distSqMax) {
               posMax = locPos;
               distSqMax = distSq;
               line->stepPos = stepPos;
               line->locPos = locPos;

			   if (posCurLoc < LINE_POINT_NUM)
				   line_point[posCurLoc++] = locPos;

#ifdef	WATCH_MODE
			   img_watch[locPos.Y*DM_PROJ_IMG_SIZE + locPos.X] = 255;
#endif

               posWanderMinLock = posWanderMin;
               posWanderMaxLock = posWanderMax;
            }
         }
         else {
            posWanderMin = min(posWanderMin, posWander);
            posWanderMax = max(posWanderMax, posWander);
         }
      }
      else if(!negRunning) {
         break;
      }

      if(negRunning != 0) {
		  locNeg = outline[stepNeg];

         xDiff = locNeg.X - loc0.X;
         yDiff = locNeg.Y - loc0.Y;
         negTravel = (cosAngle * xDiff) + (sinAngle * yDiff);
         negWander = (cosAngle * yDiff) - (sinAngle * xDiff);

         if(negWander >= -3*256 && negWander < 3*256) {
            distSq = DistanceSquared(locNeg, posMax);
            if(distSq > distSqMax) {
               negMax = locNeg;
               distSqMax = distSq;
               line->stepNeg = stepNeg;
               line->locNeg = locNeg;

			   if (negCurLoc >= 0)
				   line_point[negCurLoc--] = locNeg;

#ifdef	WATCH_MODE
			   img_watch[locNeg.Y*DM_PROJ_IMG_SIZE + locNeg.X] = 255;
#endif

               negWanderMinLock = negWanderMin;
               negWanderMaxLock = negWanderMax;
            }
         }
         else {
            negWanderMin = min(negWanderMin, negWander);
            negWanderMax = max(negWanderMax, negWander);
         }
      }
      else if(!posRunning) {
         break;
      }

/*  CALLBACK_POINT_PLOT(followPos.loc, 2, 1, 2);
    CALLBACK_POINT_PLOT(followNeg.loc, 4, 1, 2); */
	
	if(stepPos == pos_end)
		posRunning = 0;
	else
	{
		stepPos++;
		if(stepPos == OUTLINE_POINT_NUM)
			stepPos = 0;

		posRunning = (int)(i < 10 || abs(posWander) < abs(posTravel));
	}
	
	if(stepNeg == neg_end)
		negRunning = 0;
	else
	{		
		stepNeg--;
		if(stepNeg == -1)
			stepNeg = OUTLINE_POINT_NUM-1;

		negRunning = (int)(i < 10 || abs(negWander) < abs(negTravel));
	}

      /*followPos = FollowStep(dec, reg, followPos, +1);
      followNeg = FollowStep(dec, reg, followNeg, -1);*/
   }
   line->devn = max(posWanderMaxLock - posWanderMinLock, negWanderMaxLock - negWanderMinLock)>>8;	//直线的缺陷量
   line->distSq = distSqMax;

/* CALLBACK_POINT_PLOT(posMax, 2, 1, 1);
   CALLBACK_POINT_PLOT(negMax, 2, 1, 1); */

   *start = negCurLoc+1;
   *end = posCurLoc;

#ifdef WATCH_MODE
   memcpy(img_watch, dec->image->pxl, DM_PROJ_IMG_SIZE*DM_PROJ_IMG_SIZE);
#endif // WATCH_MODE

   return DmtxPass;
}

/**
 *
 *
 */
/*static*/ DmtxPassFail
MatrixRegionAlignCalibEdge(DmtxDecode *dec, DmtxRegion *reg, int edgeLoc)
{
   int streamDir;
   int steps;
   int avoidAngle;
   int symbolShape;
   DmtxVector2 pTmp;
   DmtxPixelLoc loc0, loc1, locOrigin;
   DmtxBresLine line;
   //DmtxFollow follow;
   DmtxBestLine bestLine;

   /* Determine pixel coordinates of origin */
   pTmp.X = 0.0;
   pTmp.Y = 0.0;
   dmtxMatrix3VMultiplyBy(&pTmp, reg->fit2raw);
   locOrigin.X = _dpint(pTmp.X);
   locOrigin.Y = _dpint(pTmp.Y);

   if(dec->sizeIdxExpected == DmtxSymbolSquareAuto ||
         (dec->sizeIdxExpected >= DmtxSymbol10x10 &&
         dec->sizeIdxExpected <= DmtxSymbol144x144))
      symbolShape = DmtxSymbolSquareAuto;
   else if(dec->sizeIdxExpected == DmtxSymbolRectAuto ||
         (dec->sizeIdxExpected >= DmtxSymbol8x18 &&
         dec->sizeIdxExpected <= DmtxSymbol16x48))
      symbolShape = DmtxSymbolRectAuto;
   else
      symbolShape = DmtxSymbolShapeAuto;

   /* Determine end locations of test line */
   if(edgeLoc == DmtxEdgeTop) {
      streamDir = reg->polarity * -1;
      avoidAngle = reg->leftLine.angle;
      //follow = FollowSeekLoc(dec, reg->locT);
	  loc0 = reg->locT;
      pTmp.X = 0.8;
      pTmp.Y = (symbolShape == DmtxSymbolRectAuto) ? 0.2 : 0.6;
   }
   else {
      assert(edgeLoc == DmtxEdgeRight);
      streamDir = reg->polarity;
      avoidAngle = reg->bottomLine.angle;
      //follow = FollowSeekLoc(dec, reg->locR);
	  loc0 = reg->locR;
      pTmp.X = (symbolShape == DmtxSymbolSquareAuto) ? 0.7 : 0.9;
	  //改成这样是否更合理?
	  //pTmp.X = (symbolShape == DmtxSymbolSquareAuto) ? 0.6 : 0.9;
      pTmp.Y = 0.8;
   }

   dmtxMatrix3VMultiplyBy(&pTmp, reg->fit2raw);
   loc1.X = _dpint(pTmp.X);
   loc1.Y = _dpint(pTmp.Y);

   reg->outline[0] = loc0;

#ifdef	WATCH_MODE
		img_watch[loc0.Y*DM_PROJ_IMG_SIZE + loc0.X] = 255;
		img_watch[(loc0.Y-1)*DM_PROJ_IMG_SIZE + loc0.X-1] = 255;
		img_watch[(loc0.Y-1)*DM_PROJ_IMG_SIZE + loc0.X+1] = 255;
		img_watch[(loc0.Y+1)*DM_PROJ_IMG_SIZE + loc0.X-1] = 255;
		img_watch[(loc0.Y+1)*DM_PROJ_IMG_SIZE + loc0.X+1] = 255;
		img_watch[(loc0.Y-2)*DM_PROJ_IMG_SIZE + loc0.X-2] = 255;
		img_watch[(loc0.Y-2)*DM_PROJ_IMG_SIZE + loc0.X+2] = 255;
		img_watch[(loc0.Y+2)*DM_PROJ_IMG_SIZE + loc0.X-2] = 255;
		img_watch[(loc0.Y+2)*DM_PROJ_IMG_SIZE + loc0.X+2] = 255;
#endif

   line = BresLineInit(loc0, loc1, locOrigin);
   //搜索轮廓
   steps = TrailBlazeGapped(dec, reg, line, streamDir);

   bestLine = FindBestSolidLine2(dec, reg, steps, streamDir, avoidAngle);
   if(bestLine.mag < 5) {
      return DmtxFail;
   }

   if(edgeLoc == DmtxEdgeTop) {
      reg->topKnown = 1;
      reg->topAngle = bestLine.angle;
      reg->topLoc = bestLine.locBeg;
   }
   else {
      reg->rightKnown = 1;
      reg->rightAngle = bestLine.angle;
      reg->rightLoc = bestLine.locBeg;
   }

   return DmtxPass;
}

/**
 *
 *
 */
/*static*/ DmtxBresLine
BresLineInit(DmtxPixelLoc loc0, DmtxPixelLoc loc1, DmtxPixelLoc locInside)
{
   int cp;
   DmtxBresLine line;
   DmtxPixelLoc *locBeg, *locEnd;

   /* XXX Verify that loc0 and loc1 are inbounds */

   /* Values that stay the same after initialization */
   line.loc0 = loc0;
   line.loc1 = loc1;
   line.xStep = (loc0.X < loc1.X) ? +1 : -1;
   line.yStep = (loc0.Y < loc1.Y) ? +1 : -1;
   line.xDelta = abs(loc1.X - loc0.X);
   line.yDelta = abs(loc1.Y - loc0.Y);
   line.steep = (int)(line.yDelta > line.xDelta);

   /* Take cross product to determine outward step */
   if(line.steep != 0) {
      /* Point first vector up to get correct sign */
      if(loc0.Y < loc1.Y) {
         locBeg = &loc0;
         locEnd = &loc1;
      }
      else {
         locBeg = &loc1;
         locEnd = &loc0;
      }
      cp = (((locEnd->X - locBeg->X) * (locInside.Y - locEnd->Y)) -
            ((locEnd->Y - locBeg->Y) * (locInside.X - locEnd->X)));

      line.xOut = (cp > 0) ? +1 : -1;
      line.yOut = 0;
   }
   else {
      /* Point first vector left to get correct sign */
      if(loc0.X > loc1.X) {
         locBeg = &loc0;
         locEnd = &loc1;
      }
      else {
         locBeg = &loc1;
         locEnd = &loc0;
      }
      cp = (((locEnd->X - locBeg->X) * (locInside.Y - locEnd->Y)) -
            ((locEnd->Y - locBeg->Y) * (locInside.X - locEnd->X)));

      line.xOut = 0;
      line.yOut = (cp > 0) ? +1 : -1;
   }

   /* Values that change while stepping through line */
   line.loc = loc0;
   line.travel = 0;
   line.outward = 0;
   line.error = (line.steep) ? (line.yDelta>>1) : (line.xDelta>>1);

/* CALLBACK_POINT_PLOT(loc0, 3, 1, 1);
   CALLBACK_POINT_PLOT(loc1, 3, 1, 1); */

   return line;
}

/**
 *
 *
 */
/*static*/ DmtxPassFail
BresLineGetStep(DmtxBresLine line, DmtxPixelLoc target, int *travel, int *outward)
{
   /* Determine necessary step along and outward from Bresenham line */
   if(line.steep != 0) {
      *travel = (line.yStep > 0) ? target.Y - line.loc.Y : line.loc.Y - target.Y;
      BresLineStep(&line, *travel, 0);
      *outward = (line.xOut > 0) ? target.X - line.loc.X : line.loc.X - target.X;
      assert(line.yOut == 0);
   }
   else {
      *travel = (line.xStep > 0) ? target.X - line.loc.X : line.loc.X - target.X;
      BresLineStep(&line, *travel, 0);
      *outward = (line.yOut > 0) ? target.Y - line.loc.Y : line.loc.Y - target.Y;
      assert(line.xOut == 0);
   }

   return DmtxPass;
}

/**
 *
 *
 */
/*static*/ DmtxPassFail
BresLineStep(DmtxBresLine *line, int travel, int outward)
{
   int i;
   DmtxBresLine lineNew;

   lineNew = *line;

   assert(abs(travel) < 2);
   assert(abs(outward) >= 0);

   /* Perform forward step */
   if(travel > 0) {
      lineNew.travel++;
      if(lineNew.steep != 0) {
         lineNew.loc.Y += lineNew.yStep;
         lineNew.error -= lineNew.xDelta;
         if(lineNew.error < 0) {
            lineNew.loc.X += lineNew.xStep;
            lineNew.error += lineNew.yDelta;
         }
      }
      else {
         lineNew.loc.X += lineNew.xStep;
         lineNew.error -= lineNew.yDelta;
         if(lineNew.error < 0) {
            lineNew.loc.Y += lineNew.yStep;
            lineNew.error += lineNew.xDelta;
         }
      }
   }
   else if(travel < 0) {
      lineNew.travel--;
      if(lineNew.steep != 0) {
         lineNew.loc.Y -= lineNew.yStep;
         lineNew.error += lineNew.xDelta;
         if(lineNew.error >= lineNew.yDelta) {
            lineNew.loc.X -= lineNew.xStep;
            lineNew.error -= lineNew.yDelta;
         }
      }
      else {
         lineNew.loc.X -= lineNew.xStep;
         lineNew.error += lineNew.yDelta;
         if(lineNew.error >= lineNew.xDelta) {
            lineNew.loc.Y -= lineNew.yStep;
            lineNew.error -= lineNew.xDelta;
         }
      }
   }

   for(i = 0; i < outward; i++) {
      /* Outward steps */
      lineNew.outward++;
      lineNew.loc.X += lineNew.xOut;
      lineNew.loc.Y += lineNew.yOut;
   }

   *line = lineNew;

   return DmtxPass;
}


