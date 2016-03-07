#include <stdio.h>
#include "DataMatrix.h"
#include "libdmtx/dmtx.h"
#include "project.h"

#pragma DATA_ALIGN(dstImg_down, 8)
uint8_t dstImg_down[DOWN_PROJ_IMG_SIZE*DOWN_PROJ_IMG_SIZE];

#pragma DATA_SECTION(dstImg_up, ".img_dst");
#pragma DATA_ALIGN(dstImg_up, 8)
uint8_t dstImg_up[UP_PROJ_IMG_SIZE*UP_PROJ_IMG_SIZE];		//目标图像

//检测结果存放的结构体
//AdjustInfo result;
#ifndef STATIC_PIC_TEST
extern volatile int16_t has_new_frame;
#endif

int16_t g_last_bottom_angle = -1;

DmtxVector2 g_v_x_;

volatile int16_t g_retain = 0;

int dmInit()
{
/*
#ifdef SINGLE_TEST
	if(!LoadCamParam(PARAM_PATH))
		return 0;

	SetCamParam();
	calcObjCoeffs();
	GeneratePrjTable();
#endif
*/

	g_v_x_.X = -250;
	g_v_x_.Y = 0;

	return 1;
}

int dmStart(uint8_t* src, AdjustInfo_vision* result)
{
	int err_code = 0;
	DmtxPassFail err;
	DmtxVector2 ori_loc;
	DmtxPixelLoc start_point_project;

	DmtxImage      *img;
	DmtxDecode     *dec;
	DmtxRegion     *reg;
	DmtxMessage    *msg;

	uint8_t* pxl;

	DmtxVector2 p00, p10, p00_t, v2, v2_t;
	double b_angle, v_m, tmp_d_angle, cost1, sint1, xx, yy;

	tic();

	result->delta_x = 0;
	result->delta_y = 0;
	result->delta_angle = 0;
	result->quick_mark_name[0] = '-';
	result->quick_mark_name[1] = '1';
	result->quick_mark_name[2] = '\0';

	if(cur_camera->camera_id == 0)
	{
		pxl = dstImg_down;

#ifdef STATIC_PIC_TEST
		err = dmLoc_DSP(src, &ori_loc, &tmp_x);
#else
		err = dmLoc_DSP(src, &ori_loc, &has_new_frame);
#endif
		if(err == DmtxFail)
		{
			result->detect_quickMark_used_time = toc("dmLoc");
			return -1;
		}

#ifdef TIME_TEST
	toc("dmLoc");
	tic();
#endif

		Project3_one_step_DSP(src, pxl, ori_loc, &start_point_project);
#ifdef TIME_TEST
		toc("project");
		tic();
#endif
	}
	else
	{
		pxl = dstImg_up;

		Project2_one_step(src, pxl);
		start_point_project.X = 0;
		start_point_project.Y = 0;
	}

	img = dmtxImageCreate(pxl, g_dm_proj_img_size, g_dm_proj_img_size, DmtxPack8bppK);
	assert(img != NULL);

#ifdef STATIC_PIC_TEST
	dec = dmtxDecodeCreate(img, -1);
#else
	dec = dmtxDecodeCreate(img, g_last_bottom_angle);
#endif
	assert(dec != NULL);

#ifdef TIME_TEST
	toc("dmCreate");
	tic();
#endif

#ifdef STATIC_PIC_TEST
	reg = dmtxRegionFindNext_mt(dec, &g_retain);
#else
	if(cur_camera->camera_id == 0)
		reg = dmtxRegionFindNext_mt(dec, &has_new_frame);		//超时或者检索完毕则返回
	else
		reg = dmtxRegionFindNext_mt(dec, &g_retain);
#endif

#ifdef TIME_TEST
	toc("dmtxRegionFindNext");
	tic();
#endif

	if (reg != NULL) {

		msg = dmtxDecodeMatrixRegion(dec, reg);
		if (msg != NULL)
		{
			p00.X = p00.Y = p10.Y = 0.0;
			p10.X = 1.0;
			dmtxMatrix3VMultiplyBy(&p00,reg->fit2raw);
			dmtxMatrix3VMultiplyBy(&p10,reg->fit2raw);

			p00.Y = p00.Y;	//P0
			p10.Y = p10.Y;	//P2

			//粗略下边沿方向
			v2.X = p10.X-p00.X;
			v2.Y = p10.Y-p00.Y;

			b_angle = reg->res_angle;			//加负号是因为y坐标翻转

			//下边沿(正或负)方向
			v2_t.X = cosdp_i(b_angle);
			v2_t.Y = sindp_i(b_angle);

			v_m = v2_t.X*v2.X+v2_t.Y*v2.Y;
			if(v_m < 0)
			{
				//使reg->res_angle的范围在-PI到PI之间，并指向正确方向
				if(reg->res_angle > 0)
					b_angle -= PI;
				else
					b_angle += PI;

				v2_t.X = -v2_t.X;
				v2_t.Y = -v2_t.Y;
			}

			tmp_d_angle = b_angle - p_dir_angle;
			if(tmp_d_angle > PI)
				tmp_d_angle -= 2*PI;
			else if(tmp_d_angle <= -PI)
				tmp_d_angle += 2*PI;
			result->delta_angle = tmp_d_angle*(180/PI);

			cost1 = (g_v_x_.X*v2_t.X+g_v_x_.Y*v2_t.Y)*0.004;			//1/250
			sint1 = (g_v_x_.X*v2_t.Y-v2_t.X*g_v_x_.Y)*0.004;


			p00_t.X = reg->res_loc.X+start_point_project.X;
			p00_t.Y = reg->res_loc.Y+start_point_project.Y;

			xx = -g_center_p.X+p00_t.X;
			yy = g_center_p.Y-p00_t.Y;

			result->delta_x = (xx*cost1-yy*sint1)*g_proj_ratio;
			result->delta_y = (xx*sint1+yy*cost1)*g_proj_ratio;
			//以二维码下边沿为x轴，以二维码左边沿为y轴
			memcpy(result->quick_mark_name, msg->output, 80);

			g_last_bottom_angle = reg->bottomAngle;

			err_code = 1;
		}
		else
			err_code = -3;
	}
	else
		err_code = -2;

	result->detect_quickMark_used_time = toc("dmDecode");

	return err_code;
}
