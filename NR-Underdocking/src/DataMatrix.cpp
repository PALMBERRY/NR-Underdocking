/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file simple_test.c
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <assert.h>
#include "libdmtx/dmtx.h"

#include <iostream>

#include <time.h>
#include "tic_toc.h"

#include "USB/cyusb_interface.h"
#include "calib.h"

#include "DataMatrix.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

using namespace std;

extern vector<cv::Point> resultPoint;

#ifdef WATCH_MODE
extern "C" extern unsigned char src_watch[SRC_IMG_Y*SRC_IMG_X];
extern "C" extern unsigned char img_watch[UP_PROJ_IMG_SIZE * UP_PROJ_IMG_SIZE];
extern "C" extern unsigned char barcode_res[12*12];
Mat m_watch(UP_PROJ_IMG_SIZE, UP_PROJ_IMG_SIZE, CV_8UC1, img_watch);
Mat s_watch(SRC_IMG_Y, SRC_IMG_X, CV_8UC1, src_watch);
Mat b_res(12, 12, CV_8UC1, barcode_res);

extern "C" extern Inter_Linear_Table dm_proj_table[UP_PROJ_IMG_SIZE*UP_PROJ_IMG_SIZE];
Mat m_cache(UP_PROJ_IMG_SIZE, UP_PROJ_IMG_SIZE, CV_16SC4, dm_proj_table);

#endif

#ifdef WATCH_MODE_UP
extern "C" DmtxImgInfo g_cache[UP_PROJ_IMG_SIZE*UP_PROJ_IMG_SIZE];
Mat m_cache2(UP_PROJ_IMG_SIZE, UP_PROJ_IMG_SIZE, CV_16SC4, g_cache);
#endif

#ifdef PRINT_LEN
double g_len;
#endif

uint8_t dstImg_up[UP_PROJ_IMG_SIZE*UP_PROJ_IMG_SIZE];		//目标图像

uint8_t dstImg_down[DOWN_PROJ_IMG_SIZE*DOWN_PROJ_IMG_SIZE];

uint8_t srcImg[SRC_IMG_Y*SRC_IMG_X];
Mat m_srcImg(SRC_IMG_Y, SRC_IMG_X, CV_8UC1, srcImg);
uint8_t tmp_mem[sizeof(AdjustInfo_vision)+SRC_IMG_Y*SRC_IMG_X];

//检测结果存放的结构体
AdjustInfo_vision result;

#ifdef SHOW_LOCATION
cv::Point2d g_center_point;	//为了在原图上显示中心点
cv::Point2d g_pp;
#endif

Camera_Param g_camera[2];

cv::Point2d g_v_x_;
//cv::Point2d g_v_p_dir;	//前进正方向

int16_t g_last_bottom_angle = -1;

#ifdef CIRCLE_LINE_CALIB
//	记录二维码中心
ofstream circle_out;
ofstream line_out;
#endif;

#ifdef LOG_OUT
ofstream log_out;
#endif

//图像存储索引
int rec_index;
int rx_index;

//是否检测到的标志位
bool detect;

#ifdef STATIC_PIC_TEST
	ifstream file_path;
#endif

#ifdef STATIC_PIC_TEST_RUNNING
	ifstream file_path;

	int img_end;
#endif

HANDLE h_USB_getPic;
CRITICAL_SECTION  g_csPicIndex;

char g_img_name[100];

volatile int16_t new_pic_cnt = 0;
uint16_t rx_flag = 0;
uint16_t rec_flag;

unsigned char frame_uch[2][SRC_IMG_X*SRC_IMG_Y];
Mat frame[2];
double g_usb_time[2];
char* dsp_wait_time[2];
LARGE_INTEGER g_start[2];

#ifdef CIRCLE_LINE_CALIB
DmtxVector2 g_calib_Point;

DmtxVector2 g_calib_distort[2000];
int g_calib_cnt = 0;
int last_calib_flag = 0;

//extern Inter_Linear_Table proj_table[PROJ_IMG_Y*PROJ_IMG_X];			//投影表
#endif

int g_src_img_save;

volatile int16_t g_retain = 1;

volatile uint8_t g_camera_flag = 0;

volatile bool continue_USB_Thread = true;

void calc_param(double object_size, double dm_obj_size, double label_obj_size, double best_param)
{
	double dm_size, size, dm_proj_img_size, dm_one_size, max_through_diff, label_size, search_size;
	int size_res, dm_proj_img_size_res, dm_one_size_res, white_length_2_res, max_through_diff_res, search_size_res;
	
	label_size = label_obj_size*best_param;

	dm_size = dm_obj_size*best_param;
	size = object_size*best_param;
	size_res = ((int)(size/4+0.5))*4;
	cout<<"PROJ_IMG_X = PROJ_IMG_Y = "<<size_res<<endl;

	dm_proj_img_size = size_res/352.0*88.0*dm_obj_size/22.0;
	dm_proj_img_size_res = ((int)ceil(dm_proj_img_size/4))*4;
	cout<<"MAX_FULL_PROJ_IMG_SIZE = "<<dm_proj_img_size_res<<endl;

	dm_one_size = size_res/165.0*dm_obj_size/10;
	dm_one_size_res = (int)(dm_one_size+1.5);
	cout<<"DM_ONE_SIZE = "<<dm_one_size_res<<endl;

	white_length_2_res = (int)(best_param*(label_obj_size-dm_obj_size)/4.0);
	cout<<"WHITE_LENGTH_2 = "<<white_length_2_res<<endl;

	max_through_diff = dm_size*sqrt(2.0)/2.0;
	max_through_diff_res = (int)ceil(max_through_diff);
	cout<<"MAX_THROUGH_DIFF = "<<max_through_diff_res<<endl;

	search_size = label_size*sqrt(2.0);
	search_size_res = ((int)(search_size/4.0+0.5))*4;
	cout<<"SEARCH_SIZE = "<<search_size_res<<endl;

	cout<<"DM_DISTORT_IMG_SIZE在GeneratePrjTable中统计获得！"<<endl;
}

int cameraSelect(U8 cmos_id, uint8_t CV_mode)
{
	int RetVal;

	g_camera_flag = cmos_id;	

	if(CV_mode == PICONLY || CV_mode == PICONLY_PICSAVE)
	{
		while(g_camera_flag != 0);

		SetCamParam();
		//calcObjCoeffs();
		//GeneratePrjTable();

		DmtxVector2 center_distort_point, pp_undistort_point, pp_distort_point;
		project2distortPoint(g_center_p, &center_distort_point);

#ifdef SHOW_LOCATION
		g_center_point = Point2d(center_distort_point.X, center_distort_point.Y);

		pp_undistort_point.X = g_center_p.X + cosdp_i(p_dir_angle)*100;
		pp_undistort_point.Y = g_center_p.Y + sindp_i(p_dir_angle)*100;
		project2distortPoint(pp_undistort_point, &pp_distort_point);

		g_pp = Point2d(pp_distort_point.X, pp_distort_point.Y);
#endif
	}
	else
	{
		if(g_camera_flag != 0)
		{
			cur_camera = &g_camera[g_camera_flag-1];
#ifndef STATIC_PIC_TEST
			RetVal = Select_Camera(g_camera_flag);
			if(RetVal < 0)
			{ 
				cout<<"Select Camera Failed!\n";
				system("pause");
				return 0;
			}
#endif

			g_camera_flag = 0;
		}
	}

#ifndef STATIC_PIC_TEST
	dmStart(NON_CALIB, CV_mode);
	dmStart(NON_CALIB, CV_mode);
	dmStart(NON_CALIB, CV_mode);
#endif

	//int cur_index = rec_index;

	//while(rec_index-cur_index < 4);

	g_last_bottom_angle = -1;

	return 1;
}

int dmInitial(uint8_t CV_mode)
{
	int status;
	uint8_t dsp_mode;

	continue_USB_Thread = true;
	g_camera_flag = 0;
	rx_flag = 0;
	new_pic_cnt = 0;

	//calc_param(OBJECT_X, 20.5, 35, 2.29);

	switch(CV_mode)
	{
	case PICONLY:
		g_src_img_save = 0;
		dsp_mode = CV_NOT_NEEDED;
		break;
	case PICONLY_PICSAVE:
		g_src_img_save = 1;
		dsp_mode = CV_NOT_NEEDED;
		break;
	case ADJRES:
	case ADJRES_PICSAVE:
		dsp_mode = CV_NEEDED;
		break;
	case ADJRES_ERRORPIC:
		dsp_mode = ERROR_PIC;
		break;
	default:
		printf("usb init failed!\n");
		return 0;
		break;
	}

	frame[0] = Mat(SRC_IMG_Y, SRC_IMG_X, CV_8UC1, frame_uch[0]);
	frame[1] = Mat(SRC_IMG_Y, SRC_IMG_X, CV_8UC1, frame_uch[1]);
	dsp_wait_time[0] = (char*)frame_uch[0];
	dsp_wait_time[1] = (char*)frame_uch[1];

	cur_camera = &g_camera[1];
	cur_camera->camera_id = 1;
	if(!(LoadCamParam(CAMERA2_PARAM_PATH)))
	{
		printf("load param failed!\n");
		return 0;
	}
	SetCamParam();
	calcObjCoeffs();
	GeneratePrjTable();

	cur_camera = &g_camera[0];
	cur_camera->camera_id = 0;
	if(!(LoadCamParam(CAMERA1_PARAM_PATH)))
	{
		printf("load param failed!\n");
		return 0;
	}
	SetCamParam();
	calcObjCoeffs();
	GeneratePrjTable();

#if defined(STATIC_PIC_TEST)||defined(STATIC_PIC_TEST_RUNNING)
	status = 0;
#else
	//默认选择相机1
	status = Init_Usb_Device(dsp_mode);
#endif

	if(status != 0)
	{
		printf("usb init failed!\n");
		return 0;
	}

#ifdef CARMERA_2
	cur_camera = &g_camera[1];
	SetCamParam();
#endif
		
#ifdef CIRCLE_LINE_CALIB
	circle_out.open(POINT_CIRCLE_PATH);
	line_out.open(POINT_LINE_PATH);
#endif

#ifdef LOG_OUT
	log_out.open(LOG_PATH);
#endif

#if defined(STATIC_PIC_TEST)||defined(STATIC_PIC_TEST_RUNNING)
	file_path.open(TEST_IMG_PATH);
#endif
#ifdef STATIC_PIC_TEST_RUNNING
	int img_cnt = 0;
	while(1)
	{
		file_path>>g_img_name;
		if(g_img_name[0] == '\0')
			break;
		else
			img_cnt++;
	}
	img_end = img_cnt;
#endif

	rx_index = 1;//0;
	rec_index = 1;
	detect = false;

	tic_init();

	g_v_x_ = Point2d(-250, 0);
	//g_v_p_dir = Point2d(g_pp.x-g_center_p.x,g_pp.y-g_center_p.y);	//前进正方向
	//p_dir_angle = atan2(g_v_p_dir.y, g_v_p_dir.x);
#ifdef CIRCLE_LINE_CALIB
	p_dir_angle = -PI/2;
	g_center_p.X = g_dm_proj_img_size_2;
	g_center_p.Y = g_dm_proj_img_size_2;
#endif

#ifdef SHOW_LOCATION
	//DmtxVector2 center_undistort_point, center_distort_point;
	//project2undistortPoint(g_center_p, &center_undistort_point);
	//undistort2distortPoint(center_undistort_point, &center_distort_point);

#ifdef FULL_PIC_PROJECT
	g_center_point = Point2d(g_center_p.X, g_center_p.Y);

	g_pp.x = g_center_point.x + cosdp_i(p_dir_angle) * 100;
	g_pp.y = g_center_point.y + sindp_i(p_dir_angle) * 100;
#else
	DmtxVector2 center_distort_point, pp_undistort_point, pp_distort_point;
	project2distortPoint(g_center_p, &center_distort_point);

	g_center_point = Point2d(center_distort_point.X, center_distort_point.Y);

	pp_undistort_point.X = g_center_p.X + cosdp_i(p_dir_angle)*100;
	pp_undistort_point.Y = g_center_p.Y + sindp_i(p_dir_angle)*100;
	project2distortPoint(pp_undistort_point, &pp_distort_point);

	g_pp = Point2d(pp_distort_point.X, pp_distort_point.Y);

#endif

#endif

	if(CV_mode == PICONLY || CV_mode ==  PICONLY_PICSAVE)
	{
		InitializeCriticalSection(&g_csPicIndex);
		h_USB_getPic = CreateThread(NULL, 0, USB_getPic, NULL,0 ,NULL);
	}

#ifdef CARMERA_2
	/*cout<<"输入相机ID：";
	int tmp_id;
	cin>>tmp_id;*/
	cameraSelect(2, dsp_mode);
#endif

	return 1;
}

DWORD WINAPI USB_getPic(LPVOID lpParameter)
{
	int status = 0;
	LARGE_INTEGER tmp_start;
	int RetVal;

	while(continue_USB_Thread)
	{
		rx_index++;

		while(new_pic_cnt == 2)
		{
			if(g_camera_flag != 0)
			{
				cur_camera = &g_camera[g_camera_flag-1];
#ifndef STATIC_PIC_TEST
				RetVal = Select_Camera(g_camera_flag);
				if(RetVal < 0)
				{ 
					cout<<"Select Camera Failed!\n";
					system("pause");
					return 0;
				}
#endif

				g_camera_flag = 0;
			}
		}

#ifdef STATIC_PIC_TEST

#ifdef CHECK_MODE
		cout<<"check_num:";
		cin>>rx_index;
		sprintf(g_img_name, RUNNING_IMG_PATH, rx_index);
#else
		file_path>>g_img_name;
		if(g_img_name[0] == '\0')
		{
			cout<<"Finished!"<<endl;
			return 0;
		}
#endif

		frame[rx_flag] = imread(g_img_name, CV_8UC1);
		/*if(frame[rx_flag].data == NULL)
			cout<<"Error!"<<endl;*/
		g_usb_time[rx_flag] = 17+20+10;
		
		Sleep(19);
#elif defined(STATIC_PIC_TEST_RUNNING)
		if(rx_index <= img_end)
		{
			cout<<rx_index<<endl;
			sprintf(g_img_name, RUNNING_IMG_PATH, rx_index);
			frame[rx_flag] = imread(g_img_name, CV_8UC1);
			g_usb_time[rx_flag] = 17+20+10;
		}
		else
		{
			cout<<"Finished!"<<endl;
			return 0;
		}

		Sleep(19);
#else

		status = GetOnePic(frame_uch[rx_flag], &g_usb_time[rx_flag]);	

		while(status != 0)
		{
			printf("get pic failed!\n");

			status = restartUSB();
			if(status >= 0)
			{
				status = GetOnePic(frame_uch[rx_flag], &g_usb_time[rx_flag]);
				if(status == 0)
					printf("get pic recovery!\n");
			}
		}

		//system("pause");
		g_usb_time[rx_flag] += (*dsp_wait_time[rx_flag])+20;

		//printf("%d, %lf\n", *dsp_wait_time, usb_time);
#endif		

		if(g_src_img_save == 1)
		{
			tic();
			sprintf(g_img_name, SRC_IMG_PATH, rx_index);
			imwrite(g_img_name, frame[rx_flag]);
			g_usb_time[rx_flag] += toc("");
		}

		tmp_start = tic_1();
		EnterCriticalSection(&g_csPicIndex);
		g_start[rx_flag] = tmp_start;
		rec_flag = rx_flag;
		rx_flag = 1-rx_flag;	
		new_pic_cnt++;
		//log_out<<"new_pic_cnt++ "<<new_pic_cnt<<endl;
		LeaveCriticalSection(&g_csPicIndex);

		//imshow("test", frame);
		//cvWaitKey(0);
	}
}

void dmExit(uint8_t CV_mode)
{
	if(CV_mode == PICONLY || CV_mode ==  PICONLY_PICSAVE)
	{
		continue_USB_Thread = false;
		WaitForSingleObject(h_USB_getPic, INFINITE);

		CloseHandle(h_USB_getPic);
		DeleteCriticalSection(&g_csPicIndex);
	}

#ifdef SHOW_LOCATION
	destroyAllWindows();
#endif

	closeUSB();
}

#ifndef CIRCLE_LINE_CALIB

int dmStart(int calib_flag, uint8_t CV_mode)
{
	int status = 0;
	double usb_time;
	double wait_time;
	Mat* cur_frame;
	double interval_time;
#ifdef TIME_TEST
    double roi_time, project_time, imshow_time;
#endif

	uint8_t* pxl;

	detect = false;

	if(CV_mode == PICONLY || CV_mode == PICONLY_PICSAVE)
	{
		//tic();
		//log_out<<"new_pic_cnt == 0 "<<new_pic_cnt<<endl;
		while(new_pic_cnt == 0);
		//log_out<<toc("")<<endl;

		EnterCriticalSection(&g_csPicIndex);	
		usb_time = g_usb_time[rec_flag];

		wait_time = (double)(*dsp_wait_time[rec_flag]);

		interval_time = toc_1(g_start[rec_flag]);
		cur_frame = &frame[rec_flag];
		LeaveCriticalSection(&g_csPicIndex);
	
#ifdef SHOW_RESULT
		printf("%d\n", index);
#endif

	//-------------------------------------
		tic();

#ifdef LOG_OUT
		//log_out<<new_pic_cnt<<'\t';
		result.timeStamp = getTimeStamp();
		log_out<<rec_index++<<" timeStamp = "<<result.timeStamp<<", usb_time = "<<usb_time<<"("<<wait_time<<")";
		log_out<<", interval_time = "<<interval_time;
#endif

#ifdef TAKE_A_PICTURE
		imshow("ShowLocation", *cur_frame);
		int y = waitKey(2);
		if(y == 'y' || y == 'Y')
		{
			sprintf(g_img_name, SRC_IMG_PATH, rec_index);
			imwrite(g_img_name, *cur_frame);
		}

		log_out<<endl;
		new_pic_cnt--;
		return 0;
#endif		

		DmtxPixelLoc start_point_project;

		if(cur_camera->camera_id == 0)
		{
			pxl = dstImg_down;
#ifdef FULL_PIC_PROJECT		
			Project2_one_step(cur_frame->data, pxl);
			start_point_project.X = 0;
			start_point_project.Y = 0;
#else
			DmtxPassFail err;
			DmtxVector2 ori_loc;

			//err = dmLoc(cur_frame->data, &ori_loc);
			err = dmLoc_discrete(cur_frame->data, &ori_loc);

#ifdef TIME_TEST
			roi_time = toc("roi");

#ifdef LOG_OUT
			log_out<<", roi_time = "<<roi_time;
#endif

			tic();
#endif

			if(err == DmtxFail)
			{

#ifdef SHOW_LOCATION
				line(*cur_frame, g_center_point, g_pp, Scalar(255,0,0),2);
				circle(*cur_frame,
						g_center_point,
						5,
						Scalar(255,0,0),
						2);

				imshow("ShowLocation", *cur_frame);
				waitKey(2);

#ifdef TIME_TEST
				imshow_time = toc("imshow");
				usb_time += imshow_time;
#ifdef LOG_OUT
				log_out<<", imshow_time = "<<imshow_time;
#endif

#endif
				tic();
#endif
#ifdef LOG_OUT
				log_out<<endl;
#endif
				new_pic_cnt--;
				return 0;
				//printf("dmLoc error!\n");
			}

			Project3_one_step(cur_frame->data, pxl, ori_loc, &start_point_project);
			//Project3_one_step_DSP(cur_frame->data, dstImg, ori_loc, &start_point_project, 0);
#endif
		}
		else
		{
			pxl = dstImg_up;

			Project2_one_step(cur_frame->data, pxl);
			start_point_project.X = 0;
			start_point_project.Y = 0;
		}
		
		Mat dstImage = Mat(g_dm_proj_img_size, g_dm_proj_img_size, CV_8UC1, pxl);

#ifdef SHOW_LOCATION
		//imshow("dst", dstImage);
		//waitKey(2);
#endif
#ifdef CHECK_MODE
		Mat dstImage = Mat(g_dm_proj_img_size, g_dm_proj_img_size, CV_8UC1, dstImg);
		imshow("dstImage", dstImage);
		cvWaitKey(2);
#endif

#ifdef WATCH_MODE
		memcpy(img_watch, pxl, g_dm_proj_img_size*g_dm_proj_img_size);
#endif

#ifdef TIME_TEST
		project_time = toc("Project");

#ifdef LOG_OUT
		log_out<<", project_time = "<<project_time;
#endif

		tic();
#endif

		DmtxImage      *img;
		DmtxDecode     *dec;
		DmtxRegion     *reg;
		DmtxMessage    *msg;

		img = dmtxImageCreate(pxl, g_dm_proj_img_size, g_dm_proj_img_size, DmtxPack8bppK);
		assert(img != NULL);

	//#ifdef TIME_TEST
	//	time_once += toc("ImageCreate");
	//	tic();
	//#endif
#ifdef STATIC_PIC_TEST
		dec = dmtxDecodeCreate(img, -1);
#else
		dec = dmtxDecodeCreate(img, g_last_bottom_angle);
#endif
		assert(dec != NULL);

	//#ifdef TIME_TEST
	//	time_once += toc("DecodeCreate");
	//	tic();
	//#endif

		//int expectTime = 20000;	//预期解码算法最多给20ms，超时就返回
		//DmtxTime timeout = dmtxTimeNow();
		//if (timeout.usec>1000000 - expectTime)
		//{
		//	timeout.sec++;
		//	timeout.usec = timeout.usec + expectTime - 1000000;
		//}
		//else
		//	timeout.usec += expectTime;

#ifdef _DEBUG
		reg = dmtxRegionFindNext_mt(dec, &g_retain);
#else
		if(cur_camera->camera_id == 0)
			reg = dmtxRegionFindNext_mt(dec, &new_pic_cnt);		//超时或者检索完毕则返回
		else
			reg = dmtxRegionFindNext_mt(dec, &g_retain);
#endif
		//log_out<<"time out "<<new_pic_cnt<<endl;
		//reg = dmtxRegionFindNext(dec, NULL);

	//#ifdef TIME_TEST
	//	time_once += toc("RegionFindeNext");
	//	tic();
	//#endif

		if (reg != NULL) {

#ifdef WATCH_MODE
			DmtxVector2 p00,p01,p11,p10;
			p00.X = p00.Y = p10.Y = p01.X = 0.0;
			p10.X = p01.Y = p11.X = p11.Y = 1.0;
			dmtxMatrix3VMultiplyBy(&p00,reg->fit2raw);
			dmtxMatrix3VMultiplyBy(&p01,reg->fit2raw);
			dmtxMatrix3VMultiplyBy(&p10,reg->fit2raw);
			dmtxMatrix3VMultiplyBy(&p11,reg->fit2raw);

			line(m_watch, Point2d(p00.X, p00.Y), Point2d(p01.X, p01.Y), Scalar(255, 0, 255));
			line(m_watch, Point2d(p00.X, p00.Y), Point2d(p10.X, p10.Y), Scalar(255, 0, 255));
			line(m_watch, Point2d(p01.X, p01.Y), Point2d(p11.X, p11.Y), Scalar(255, 0, 255));
			line(m_watch, Point2d(p10.X, p10.Y), Point2d(p11.X, p11.Y), Scalar(255, 0, 255));
#endif
		
			msg = dmtxDecodeMatrixRegion(dec, reg);
			if (msg != NULL)
			{
				//right_cnt++;

				DmtxVector2 p00,p01,p11,p10;
				p00.X = p00.Y = p10.Y = p01.X = 0.0;
				p10.X = p01.Y = p11.X = p11.Y = 1.0;
				dmtxMatrix3VMultiplyBy(&p00,reg->fit2raw);
				dmtxMatrix3VMultiplyBy(&p10,reg->fit2raw);
				//dmtxMatrix3VMultiplyBy(&p11,reg->fit2raw);

				Point2d P0 = Point2d(p00.X, p00.Y);
				Point2d P2 = Point2d(p10.X, p10.Y);

				Point2d P0_t = Point2d(reg->res_loc.X+start_point_project.X, reg->res_loc.Y+start_point_project.Y);	//P0精确位置+偏差

				double xx = -g_center_p.X+P0_t.x;
				double yy = g_center_p.Y-P0_t.y;

				Point2d v2(P2.x-P0.x,P2.y-P0.y);			//粗略下边沿方向
#ifdef PRINT_LEN
				double xxx = P2.x-P0_t.x;
				double yyy = P2.y-P0_t.y;
				g_len = sqrt(xxx*xxx+yyy*yyy);
#endif

				//double b_angle = -reg->res_angle;			//加负号是因为y坐标翻转
				double b_angle = reg->res_angle;
				Point2d v2_t(cosdp_i(b_angle), sindp_i(b_angle));	//下边沿(正或负)方向
				double v_m = v2_t.x*v2.x+v2_t.y*v2.y;
				if(v_m < 0)
				{
					//使reg->res_angle的范围在-PI到PI之间，并指向正确方向
					if(reg->res_angle > 0)
						b_angle -= PI; 
					else
						b_angle += PI;

					v2_t.x = -v2_t.x;
					v2_t.y = -v2_t.y;
				}
				//reg->res_angle = -reg->res_angle;	

				double tmp_d_angle;
				tmp_d_angle = b_angle - p_dir_angle;
				if(tmp_d_angle > PI)
					tmp_d_angle -= 2*PI;
				else if(tmp_d_angle <= -PI)
					tmp_d_angle += 2*PI;
				result.delta_angle = tmp_d_angle*(180/PI);

				//Point v3(P1.x - P0.x, P1.y - P0.y);					//左边沿方向
				//下边沿与x轴负方向的夹角
				double cost1 = (g_v_x_.x*v2_t.x+g_v_x_.y*v2_t.y)*0.004;			//1/250	
				double sint1 = (g_v_x_.x*v2_t.y-v2_t.x*g_v_x_.y)*0.004;
				result.delta_x = (xx*cost1-yy*sint1)*g_proj_ratio;
				result.delta_y = (xx*sint1+yy*cost1)*g_proj_ratio;
				//以二维码下边沿为x轴，以二维码左边沿为y轴

				switch(msg->output[0])
				{
				case '0':
					result.delta_x += -56;
					result.delta_y += 35.5;
					break;
				case '1':
					result.delta_x += -25.5;
					result.delta_y += 35.5;
					break;
				case '2':
					result.delta_x += 5;
					result.delta_y += 35.5;
					break;
				case '3':
					result.delta_x += 35.5;
					result.delta_y += 35.5;
					break;
				case '4':
					result.delta_x += -56;
					result.delta_y += 5;
					break;
				case '5':
					result.delta_x += -25.5;
					result.delta_y += 5;
					break;
				case '6':
					result.delta_x += 5;
					result.delta_y += 5;
					break;
				case '7':
					result.delta_x += 35.5;
					result.delta_y += 5;
					break;
				case '8':
					result.delta_x += -56;
					result.delta_y += -25.5;
					break;
				case '9':
					result.delta_x += -25.5;
					result.delta_y += -25.5;
					break;
				case 'A':
					result.delta_x += 5;
					result.delta_y += -25.5;
					break;
				case 'B':
					result.delta_x += 35.5;
					result.delta_y += -25.5;
					break;
				case 'C':
					result.delta_x += -56;
					result.delta_y += -56;
					break;
				case 'D':
					result.delta_x += -25.5;
					result.delta_y += -56;
					break;
				case 'E':
					result.delta_x += 5;
					result.delta_y += -56;
					break;
				case 'F':
					result.delta_x += 35.5;
					result.delta_y += -56;
					break;
				default:
					break;
				}
#ifdef SHOW_RESULT
 	 			fputs("output: \"", stdout);
 	 			fwrite(msg->output, sizeof(unsigned char), msg->outputIdx, stdout);
 	 			fputs("\"\n", stdout);
#endif

				memcpy(result.quick_mark_name, msg->output, 80);

#ifdef SHOW_LOCATION
				char loc_str[100]; 
				sprintf(loc_str, "%.2lf, %.2lf, %.2lf, %s", result.delta_x, result.delta_y, result.delta_angle, (char*)msg->output);
#ifdef FULL_PIC_PROJECT
				putText(dstImage, loc_str, Point(20, 20), 1, 1, Scalar(255, 0, 255), 2);
#else
				putText(*cur_frame, loc_str, Point(20, 20), 1, 1, Scalar(255, 0, 255), 2);
#endif

#ifdef PRINT_LEN
				sprintf(loc_str, "len: %.2lf", g_len);
#ifdef FULL_PIC_PROJECT
				putText(dstImage, loc_str, Point(20, 50), 1, 1, Scalar(255, 0, 255), 2);
#else
				putText(*cur_frame, loc_str, Point(20, 50), 1, 1, Scalar(255, 0, 255), 2);
#endif
#endif

#endif

#ifdef LOG_OUT
				log_out<<", output = "<<result.quick_mark_name;
				log_out<<", delta_angle = "<<result.delta_angle;
				log_out<<", delta_x = "<<result.delta_x;
				log_out<<", delta_y = "<<result.delta_y;
#endif
				g_last_bottom_angle = reg->bottomAngle;

				detect = true;
			}
#ifdef LOG_OUT
			else
			{
				log_out<<", output = "<<-1;
			}
#endif
		}
#ifdef LOG_OUT
		else
		{
			log_out<<", output = "<<-1;
		}
#endif

		result.detect_quickMark_used_time = toc("DecodeMatrixRegion");

#ifdef LOG_OUT
		log_out<<", detect_quickMark_used_time = "<<result.detect_quickMark_used_time;
#endif

#ifdef SHOW_LOCATION
		tic();
#endif

#ifdef DST_IMG_SAVE
		sprintf(g_img_name, DST_IMG_PATH, index);
		imwrite(g_img_name, dstImage);
#endif

#ifdef SHOW_LOCATION

#ifdef FULL_PIC_PROJECT
		line(dstImage, g_center_point, g_pp, Scalar(255,0,0),2);
		circle(dstImage,
				g_center_point,
				5,
				Scalar(255,0,0),
				2);

		imshow("ShowLocation", dstImage);
#else
		line(*cur_frame, g_center_point, g_pp, Scalar(255,0,0),1);
		circle(*cur_frame,
				g_center_point,
				5,
				Scalar(255,0,0),
				2);

		imshow("ShowLocation", *cur_frame);
#endif
		waitKey(2);	

#ifdef TIME_TEST
		imshow_time = toc("imshow");
		usb_time += imshow_time;
#ifdef LOG_OUT
		log_out<<", imshow_time = "<<imshow_time;
#endif

#endif

#endif

		new_pic_cnt--;

#ifdef TIME_TEST
		result.feedback_time = usb_time+interval_time+roi_time+project_time+result.detect_quickMark_used_time;
#else
		result.feedback_time = toc("")+usb_time+interval_time+result.detect_quickMark_used_time;
#endif

#ifdef SHOW_RESULT
		printf("本次总耗时: %.3f\n\n", result.feedback_time);
#endif

#ifdef LOG_OUT
		log_out<<", total_time = "<<result.feedback_time<<endl;
#endif
	}
	else
	{
		char img_name[100];
		rec_index++;

		if(CV_mode == ADJRES)
		{
			status = GetAdjRes((uint8_t*)(&result), &wait_time);
			if(status != 0)
			{
				printf("get res failed!\n");
				return 0;
			}
			result.feedback_time += 20+wait_time;
			//feedback_time = 获得USB命令的时间-新的一帧到达的时间+USB等待的时间+20ms曝光时间
		}
		else if(CV_mode == ADJRES_ERRORPIC)
		{
			status = GetAdjRes((uint8_t*)(&result), &wait_time);
			if(status != 0)
			{
				printf("get res failed!\n");
				return 0;
			}
			result.feedback_time += 20+wait_time;
			//feedback_time = 获得USB命令的时间-新的一帧到达的时间+USB等待的时间+20ms曝光时间

			if(result.quick_mark_name[0] == '-' || result.detect_quickMark_used_time > 10)
			{
				tic();

				status = GetOnePic(srcImg, &wait_time);
				if(status != 0)
				{
					printf("get pic failed!\n");
					return 0;
				}
				sprintf(img_name, SRC_IMG_PATH, rec_index);
				imwrite(img_name, m_srcImg);

				/*imshow("m_srcImg", m_srcImg);
				cvWaitKey(2);*/

				result.feedback_time += toc("add");
			}
		}
		else if(CV_mode == ADJRES_PICSAVE)
		{
			status = GetAdjRes_OnePic((uint8_t*)(&tmp_mem), &wait_time);
			if(status != 0)
			{
				printf("get res failed!\n");
				return 0;
			}

			tic();

			memcpy(&result, tmp_mem, sizeof(AdjustInfo_vision));
			memcpy(srcImg, tmp_mem+sizeof(AdjustInfo_vision), SRC_IMG_Y*SRC_IMG_X);

			sprintf(img_name, SRC_IMG_PATH, rec_index);
			imwrite(img_name, m_srcImg);

			char loc_str[100]; 
			sprintf(loc_str, "%.2lf, %.2lf, %.2lf, %.2lf, %s", result.delta_x, result.delta_y, result.delta_angle, result.detect_quickMark_used_time, result.quick_mark_name);
			putText(m_srcImg, loc_str, Point(20, 20), 1, 1, Scalar(255, 0, 255), 2);

#ifdef SHOW_LOCATION
			line(m_srcImg, g_center_point, g_pp, Scalar(255,0,0),1);
			circle(m_srcImg,
					g_center_point,
					5,
					Scalar(255,0,0),
					2);

			imshow("m_srcImg", m_srcImg);
			cvWaitKey(2);
#endif

			result.feedback_time += 20+wait_time+toc("add");
		}
		else	
		{
			cout<<"Error CV_mode!"<<endl;
			return 0;
		}

		//printf("%lf\t%lf\t%lf\t%lf\t%lf\t%s\n", res.delta_x, res.delta_y, res.delta_angle, res.detect_quickMark_used_time, res.timestamp, res.quick_mark_name);
		log_out<<rec_index;
		log_out<<"\ttimeStamp = "<<result.timeStamp;
		log_out<<"\tdelta_x = "<<result.delta_x<<"\tdelta_y = "<<result.delta_y<<"\tdelta_angle = "<<result.delta_angle;
		log_out<<"\tdetect_quickMark_used_time = "<<result.detect_quickMark_used_time<<"\twait_time = "<<wait_time<<"\tfeedback_time = "<<result.feedback_time;
		log_out<<"\tquick_mark_name = "<<result.quick_mark_name<<"\terr_code = "<<result.err_code<<endl;

		if(result.quick_mark_name[0] != '-')
			detect = 1;
	}

	/*if(_isnan(result.delta_x))
	{
		while(1)
			cout<<"ERROR!\n";
	}*/

	return detect;
}

#else

uint8_t calib_msg = 0xFF;

int dmStart(int calib_flag, uint8_t CV_mode)
{
	int status = 0;
	double usb_time;
	int wait_time;
	Mat* cur_frame;
	double interval_time;
#ifdef TIME_TEST
    double roi_time, project_time, DecodeMatrixRegion_time, imshow_time;
#endif

	detect = false;

	if(CV_mode != PICONLY)
	{
		cout<<"标定模式不正确！"<<endl;
		return 0;
	}

	//tic();
	//log_out<<"new_pic_cnt == 0 "<<new_pic_cnt<<endl;
	while(new_pic_cnt == 0);
	//log_out<<toc("")<<endl;

	EnterCriticalSection(&g_csPicIndex);	
	usb_time = g_usb_time[rec_flag];

	wait_time = (int)(*dsp_wait_time[rec_flag]);

	interval_time = toc_1(g_start[rec_flag]);
	cur_frame = &frame[rec_flag];
	LeaveCriticalSection(&g_csPicIndex);
	
#ifdef SHOW_RESULT
	printf("%d\n", index);
#endif

//-------------------------------------
	tic();

#ifdef LOG_OUT
	//log_out<<new_pic_cnt<<'\t';
	result.timeStamp = getTimeStamp();
	log_out<<rec_index++<<" timeStamp = "<<result.timeStamp<<", usb_time = "<<usb_time<<"("<<wait_time<<")";
	log_out<<", interval_time = "<<interval_time;
#endif

	/*if(calib_flag == NON_CALIB)
	{
#ifdef SHOW_LOCATION
		line(*cur_frame, g_center_point, g_pp, Scalar(255,0,0),2);
		circle(*cur_frame,
				g_center_point,
				5,
				Scalar(255,0,0),
				2);

		imshow("ShowLocation", *cur_frame);
		waitKey(2);
#endif

		log_out<<endl;
		new_pic_cnt--;
		return 0;
	}*/

	DmtxPixelLoc start_point_project;
	uint8_t* pxl;

#ifdef FULL_PIC_PROJECT
	if(cur_camera->camera_id == 0)
	{
		pxl = dstImg_down;
	}
	else
	{
		pxl = dstImg_up;
	}
	Project2_one_step(cur_frame->data, pxl);
	start_point_project.X = 0;
	start_point_project.Y = 0;
	Mat dstImage = Mat(g_dm_proj_img_size, g_dm_proj_img_size, CV_8UC1, pxl);
#else
	DmtxPassFail err;
	DmtxVector2 ori_loc;
	
	err = dmLoc(cur_frame->data, &ori_loc);

#ifdef TIME_TEST
	roi_time = toc("roi");

#ifdef LOG_OUT
	log_out<<", roi_time = "<<roi_time;
#endif

	tic();
#endif

	if(err == DmtxFail)
	{

#ifdef SHOW_LOCATION
		line(*cur_frame, g_center_point, g_pp, Scalar(255,0,0),2);
		circle(*cur_frame,
				g_center_point,
				5,
				Scalar(255,0,0),
				2);

		imshow("ShowLocation", *cur_frame);
		waitKey(2);

#ifdef TIME_TEST
		imshow_time = toc("imshow");
		usb_time += imshow_time;
#ifdef LOG_OUT
		log_out<<", imshow_time = "<<imshow_time;
#endif

#endif
		tic();
#endif
#ifdef LOG_OUT
		log_out<<endl;
#endif
		new_pic_cnt--;
		return 0;
		//printf("dmLoc error!\n");
	}

	Project3_one_step(cur_camera, cur_frame->data, dstImg, ori_loc, &start_point_project);
	//Project3_one_step_DSP(cur_frame->data, dstImg, ori_loc, &start_point_project, 0);
#endif

	//Mat dstImage = Mat(g_dm_proj_img_size, g_dm_proj_img_size, CV_8UC1, dstImg);
#ifdef CHECK_MODE
	Mat dstImage = Mat(g_dm_proj_img_size, g_dm_proj_img_size, CV_8UC1, dstImg);
	imshow("dstImage", dstImage);
	cvWaitKey(2);
#endif

#ifdef WATCH_MODE
   memcpy(img_watch, dstImg, g_dm_proj_img_size*g_dm_proj_img_size);
#endif

#ifdef TIME_TEST
	project_time = toc("Project");

#ifdef LOG_OUT
	log_out<<", project_time = "<<project_time;
#endif

	tic();
#endif

	DmtxImage      *img;
	DmtxDecode     *dec;
	DmtxRegion     *reg;
	DmtxMessage    *msg;

	img = dmtxImageCreate(pxl, g_dm_proj_img_size, g_dm_proj_img_size, DmtxPack8bppK);
	assert(img != NULL);

//#ifdef TIME_TEST
//	time_once += toc("ImageCreate");
//	tic();
//#endif
	dec = dmtxDecodeCreate(img, -1);

	assert(dec != NULL);

//#ifdef TIME_TEST
//	time_once += toc("DecodeCreate");
//	tic();
//#endif

	//int expectTime = 20000;	//预期解码算法最多给20ms，超时就返回
	//DmtxTime timeout = dmtxTimeNow();
	//if (timeout.usec>1000000 - expectTime)
	//{
	//	timeout.sec++;
	//	timeout.usec = timeout.usec + expectTime - 1000000;
	//}
	//else
	//	timeout.usec += expectTime;

	volatile int16_t tmp_x = 1;
	int reg_cnt = 0;

	reg = dmtxRegionFindNext_mt_calib(dec, &tmp_x, &reg_cnt);

	//log_out<<"time out "<<new_pic_cnt<<endl;
	//reg = dmtxRegionFindNext(dec, NULL);

//#ifdef TIME_TEST
//	time_once += toc("RegionFindeNext");
//	tic();
//#endif

	if (reg != NULL) {

		if(calib_flag == EX_PARAM_CALIB && reg_cnt == 4)
		{
			Mat m_inParam = Mat::zeros(3,3,CV_32F);
			Mat m_distortion = Mat::zeros(4,1,CV_32F);
			
			for(int i = 0;i < 3;i++)
			{
				for(int j = 0;j < 3;j++)
					m_inParam.at<float>(i, j) = cur_camera->inParam[i*3+j];
			}
			for(int i = 0;i < 4;i++)
				m_distortion.at<float>(i, 0) = cur_camera->distortion[i];

			/*m_inParam.at<float>(0, 0) = 4.2028368926570874e+02;
			m_inParam.at<float>(0, 1) = 0.0000000000000000e+00;
			m_inParam.at<float>(0, 2) = 2.8582941665304310e+02;
			m_inParam.at<float>(1, 0) = 0.0000000000000000e+00;
			m_inParam.at<float>(1, 1) = 4.2024124476719049e+02;
			m_inParam.at<float>(1, 2) = 2.2742918627137584e+02;
		   m_inParam.at<float>(2, 0) = 0.0000000000000000e+00;
		   m_inParam.at<float>(2, 1) = 0.0000000000000000e+00;
		   m_inParam.at<float>(2, 2) = 1.0000000000000000e+00;

		   m_distortion.at<float>(0, 0) =   -3.5060980703192135e-01;
		   m_distortion.at<float>(1, 0) = 1.1791822697442690e-01;
			  m_distortion.at<float>(2, 0) =  0.0000000000000000e+00;
			  m_distortion.at<float>(3, 0) = 0.0000000000000000e+00;*/


			DmtxVector2 p00, p00_distort;
			std::vector<Point2f> img_points;
			std::vector<Point3f> obj_points;

			obj_points.push_back(Point3f(0, 22, 0));
			obj_points.push_back(Point3f(48.7, 22, 0));
			obj_points.push_back(Point3f(0, 65.9, 0));
			obj_points.push_back(Point3f(48.7, 65.9, 0));

			Point2f P0[4], tmp_p;

			int j;
			for(int i = 0;i < 4;i++)
			{
				msg = dmtxDecodeMatrixRegion(dec, &reg[i]);
				if(msg != NULL)
				{
					switch(msg->output[0])
					{
					case '1':
						j = 0;
						break;
					case '2':
						j = 1;
						break;
					case '3':
						j = 2;
						break;
					case '4':
						j = 3;
						break;
					default:
						cout<<"错误的标定图！\n";
						break;
					}
				}

				/*p00.X = p00.Y = 0.0;
				dmtxMatrix3VMultiplyBy(&p00, reg[i].fit2raw);*/
				p00.X = reg[i].res_loc.X+start_point_project.X;
				p00.Y = reg[i].res_loc.Y+start_point_project.Y;

				P0[j] = Point2f(p00.X, p00.Y);
				cv::circle(dstImage, P0[j], 5, Scalar(255,0,0), 2);

				project2distortPoint(p00, &p00_distort);				
				P0[j] = Point2f(p00_distort.X, p00_distort.Y);

				/*DmtxVector3 tmp_1;
				distort2worldPoint(p00_distort, &tmp_1);
				cout<<tmp_1.X<<endl;*/
			}

			/*P0[0].x = 235;
			P0[0].y = 407;
			P0[1].x = 291;
			P0[1].y = 200;
			P0[2].x = 392;
			P0[2].y = 420;
			P0[3].x = 450;
			P0[3].y = 240;*/

			for(int i = 0;i < 4;i++)
			{
				img_points.push_back(P0[i]); 
			}
			
			Mat rvec, tvec, rvec_R, rvec_R_;
			solvePnP(obj_points, img_points, m_inParam, m_distortion, rvec, tvec);

			Rodrigues(rvec, rvec_R);

			rvec_R_ = rvec_R.inv();

			ofstream ex_file;
			char filename[100];
			sprintf(filename, TMP_PARAM_PATH, "extrinsicT.txt");
			ex_file.open(filename);

			for(int i = 0;i < 3;i++)
				ex_file<<tvec.at<double>(i, 0)<<'\t';
			ex_file.close();

			sprintf(filename, TMP_PARAM_PATH, "extrinsicR.txt");
			ex_file.open(filename);

			for(int i = 0;i < 3;i++)
			{
				for(int j = 0;j < 3;j++)
					ex_file<<rvec_R.at<double>(i, j)<<'\t';
				ex_file<<endl;
			}
			for(int i = 0;i < 3;i++)
			{
				for(int j = 0;j < 3;j++)
					ex_file<<rvec_R_.at<double>(i, j)<<'\t';
				ex_file<<endl;
			}
			ex_file.close();

			if(!LoadCamParam_calib(TMP_PARAM_PATH))
			{
				printf("load param failed!\n");
				return 0;
			}

			//SetCamParam();	//并没有外参相关设置
			calcObjCoeffs();
			GeneratePrjTable();

			detect = true;
		}
		else if(calib_flag == UP_CALIB)
		{
			int i;
			for(i = 0;i < reg_cnt;i++)
			{
				msg = dmtxDecodeMatrixRegion(dec, &reg[i]);
				if(msg != NULL)
				{
					if(msg->output[0] == '2')
					{
						
						break;
					}
				}
			}

			if (i != reg_cnt)
			{
				DmtxVector2 p00,p01,p11,p10;
				p00.X = p00.Y = p10.Y = p01.X = 0.0;
				p10.X = p01.Y = p11.X = p11.Y = 1.0;
				dmtxMatrix3VMultiplyBy(&p00, reg[i].fit2raw);
				dmtxMatrix3VMultiplyBy(&p10, reg[i].fit2raw);
				//dmtxMatrix3VMultiplyBy(&p11,reg->fit2raw);

				Point2d P0 = Point2d(p00.X, p00.Y);
				Point2d P2 = Point2d(p10.X, p10.Y);

				Point2d P0_t = Point2d(reg[i].res_loc.X+start_point_project.X, reg[i].res_loc.Y+start_point_project.Y);	//P0精确位置+偏差

				//记录当前位置作为中心点
				ofstream out;
				out.open(POINT_CIRCLE_SAVE_PATH);
				out<<P0_t.x<<'\t'<<P0_t.y<<endl;
				out.close();

				Point2d v2(P2.x-P0.x,P2.y-P0.y);			//粗略下边沿方向

				//double b_angle = -reg->res_angle;			//加负号是因为y坐标翻转
				double b_angle = reg[i].res_angle;
				Point2d v2_t(cosdp_i(b_angle), sindp_i(b_angle));	//下边沿(正或负)方向
				double v_m = v2_t.x*v2.x+v2_t.y*v2.y;
				if(v_m < 0)
				{
					//使reg->res_angle的范围在-PI到PI之间，并指向正确方向
					if(reg[i].res_angle > 0)
						b_angle -= PI; 
					else
						b_angle += PI;

					v2_t.x = -v2_t.x;
					v2_t.y = -v2_t.y;
				}
				//reg->res_angle = -reg->res_angle;	

				out.open(POINT_LINE_SAVE_PATH);
				out<<b_angle<<endl;
				out.close();

				Point2d P0_tt;
				P0_tt.x = P0_t.x + cosdp_i(b_angle)*100;
				P0_tt.y = P0_t.y + sindp_i(b_angle)*100;

				line(dstImage, P0_t, P0_tt, Scalar(255,0,0),1);
				circle(dstImage,
						P0_t,
						5,
						Scalar(255,0,0),
						2);

				detect = true;
			}
			else
				printf("No No.2 found!\n");
		}
		else if(calib_flag == NON_CALIB || calib_flag == CIRCLE_CALIB || calib_flag == LINE_CALIB)
		{
			//标定时另一侧一定不能贴1、2、3、4
			/*if (calib_flag == LINE_CALIB && reg_cnt != 0)
				cout << "ininini" << endl;*/

			if (last_calib_flag != calib_flag)
				calib_msg = 0xFF;

			int i;
			for(i = 0;i < reg_cnt;i++)
			{
				msg = dmtxDecodeMatrixRegion(dec, &reg[i]);
				if(msg != NULL)
				{
					if(calib_msg == 0xFF &&(msg->output[0] == '1' || msg->output[0] == '2' || msg->output[0] == '3' || msg->output[0] == '4'))
					{
						calib_msg = msg->output[0];
						break;
					}
					else
					{
 						if(msg->output[0] == calib_msg)
							break;
					}
				}
			}
			if (i != reg_cnt)
			{
				//right_cnt++;

				DmtxVector2 p00,p01,p11,p10;
				p00.X = p00.Y = p10.Y = p01.X = 0.0;
				p10.X = p01.Y = p11.X = p11.Y = 1.0;
				dmtxMatrix3VMultiplyBy(&p00,reg[i].fit2raw);
				dmtxMatrix3VMultiplyBy(&p10,reg[i].fit2raw);
				//dmtxMatrix3VMultiplyBy(&p11,reg->fit2raw);

				Point2d P0 = Point2d(p00.X, p00.Y);
				Point2d P2 = Point2d(p10.X, p10.Y);

				Point2d P0_t = Point2d(reg[i].res_loc.X+start_point_project.X, reg[i].res_loc.Y+start_point_project.Y);	//P0精确位置+偏差

#ifdef CIRCLE_LINE_CALIB
				//dmtxMatrix3VMultiplyBy(&p01,reg[i].fit2raw);
				//Point2d P1 = Point2d(p01.X, p01.Y);

				//二维码中心的投影图坐标
				g_calib_Point.X = reg[i].res_loc.X+start_point_project.X;
				g_calib_Point.Y = reg[i].res_loc.Y+start_point_project.Y;

				if(calib_flag != NON_CALIB)
				{
					if(last_calib_flag == calib_flag)
					{
						if(g_calib_cnt < 1999)
							g_calib_cnt++;
					}
					else
						g_calib_cnt = 0;

					g_calib_distort[g_calib_cnt] = g_calib_Point;
					//project2distortPoint(g_calib_Point, &g_calib_distort[g_calib_cnt]);

					for(int i = 0;i < g_calib_cnt+1;i++)
						dstImage.data[(int)g_calib_distort[i].Y*g_dm_proj_img_size+(int)g_calib_distort[i].X] = 255;

					//cout << "g_calib_cnt = "<< g_calib_cnt << endl;

					last_calib_flag = calib_flag;
				}
#endif
				double xx = -g_center_p.X+P0_t.x;
				double yy = g_center_p.Y-P0_t.y;

				Point2d v2(P2.x-P0.x,P2.y-P0.y);			//粗略下边沿方向

				//double b_angle = -reg->res_angle;			//加负号是因为y坐标翻转
				double b_angle = reg[i].res_angle;
				Point2d v2_t(cosdp_i(b_angle), sindp_i(b_angle));	//下边沿(正或负)方向
				double v_m = v2_t.x*v2.x+v2_t.y*v2.y;
				if(v_m < 0)
				{
					//使reg->res_angle的范围在-PI到PI之间，并指向正确方向
					if(reg[i].res_angle > 0)
						b_angle -= PI; 
					else
						b_angle += PI;

					v2_t.x = -v2_t.x;
					v2_t.y = -v2_t.y;
				}
				//reg->res_angle = -reg->res_angle;	

				double tmp_d_angle;
				tmp_d_angle = b_angle - p_dir_angle;
				if(tmp_d_angle > PI)
					tmp_d_angle -= 2*PI;
				else if(tmp_d_angle <= -PI)
					tmp_d_angle += 2*PI;
				result.delta_angle = tmp_d_angle*(180/PI);

				//Point v3(P1.x - P0.x, P1.y - P0.y);					//左边沿方向
				//下边沿与x轴负方向的夹角
				double cost1 = (g_v_x_.x*v2_t.x+g_v_x_.y*v2_t.y)*0.004;			//1/250	
				double sint1 = (g_v_x_.x*v2_t.y-v2_t.x*g_v_x_.y)*0.004;
				result.delta_x = (xx*cost1-yy*sint1)*g_proj_ratio;
				result.delta_y = (xx*sint1+yy*cost1)*g_proj_ratio;
				//以二维码下边沿为x轴，以二维码左边沿为y轴

				memcpy(result.quick_mark_name, msg->output, 80);

#ifdef SHOW_LOCATION
				char loc_str[100]; 
				sprintf(loc_str, "%.2lf, %.2lf, %.2lf, %s", result.delta_x, result.delta_y, result.delta_angle, (char*)msg->output);
#ifdef FULL_PIC_PROJECT
				putText(dstImage, loc_str, Point(20, 20), 1, 1, Scalar(255, 0, 255), 2);
#else
				putText(*cur_frame, loc_str, Point(20, 20), 1, 1, Scalar(255, 0, 255), 2);
#endif
#endif

#ifdef LOG_OUT
				log_out<<", output = "<<result.quick_mark_name;
				log_out<<", delta_angle = "<<result.delta_angle;
				log_out<<", delta_x = "<<result.delta_x;
				log_out<<", delta_y = "<<result.delta_y;
#endif
				g_last_bottom_angle = reg[i].bottomAngle;

				detect = true;
			}
#ifdef LOG_OUT
			else
			{
				log_out<<", output = "<<-1;
			}
#endif
		}
	}
#ifdef LOG_OUT
	else
	{
		log_out<<", output = "<<-1;
	}
#endif

#ifdef TIME_TEST
	result.detect_quickMark_used_time = DecodeMatrixRegion_time = toc("DecodeMatrixRegion");

#ifdef LOG_OUT
	log_out<<", DecodeMatrixRegion_time = "<<DecodeMatrixRegion_time;
#endif

#ifdef SHOW_LOCATION
	tic();
#endif

#endif

#ifdef DST_IMG_SAVE
	sprintf(g_img_name, DST_IMG_PATH, index);
	imwrite(g_img_name, dstImage);
#endif

#ifdef SHOW_LOCATION
	if(calib_flag == EX_PARAM_CALIB)
	{
		if(reg_cnt != 4)
		{
			putText(dstImage, "DataMatrix found not enough!", Point(20, 20), 1, 1, Scalar(128, 0, 128), 1);

			imshow("ShowLocation", dstImage);
			waitKey(2);
		}
		else
		{
			putText(dstImage, "ex_param calibration success!", Point(20, 20), 1, 1, Scalar(128, 0, 128), 1);

			putText(dstImage, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(128, 0, 128), 1);

			imshow("ShowLocation", dstImage);
			waitKey(0);
		}

		
	}
	else if(calib_flag == UP_CALIB)
	{
		
		if(detect)
		{
			putText(dstImage, "up_calib success!", Point(20, 20), 1, 1, Scalar(128, 0, 128), 1);
			putText(dstImage, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(128, 0, 128), 1);
			imshow("ShowLocation", dstImage);
			waitKey(0);
		}
		else
		{
			putText(dstImage, "up_calib failed!", Point(20, 20), 1, 1, Scalar(128, 0, 128), 1);
			imshow("ShowLocation", dstImage);
			waitKey(2);
		}
	}
	else if(calib_flag == NON_CALIB || calib_flag == CIRCLE_CALIB || calib_flag == LINE_CALIB)
	{
		line(dstImage, g_center_point, g_pp, Scalar(255,0,0),1);
		circle(dstImage,
				g_center_point,
				5,
				Scalar(255,0,0),
				2);

		imshow("ShowLocation", dstImage);
		waitKey(2);
	}

#ifdef TIME_TEST
	imshow_time = toc("imshow");
	usb_time += imshow_time;
#ifdef LOG_OUT
	log_out<<", imshow_time = "<<imshow_time;
#endif

#endif

#endif

	new_pic_cnt--;

#ifdef TIME_TEST
	result.feedback_time = usb_time+interval_time+roi_time+project_time+DecodeMatrixRegion_time;
#else
	result.feedback_time = toc("")+usb_time+interval_time;
#endif

#ifdef SHOW_RESULT
	printf("本次总耗时: %.3f\n\n", result.feedback_time);
#endif

#ifdef LOG_OUT
	log_out<<", total_time = "<<result.feedback_time<<endl;
#endif

	return detect;
}


#endif

//flag为0表示不保存，为1保存circle_point,为2保存line_point
void point_save(int flag)
{
#ifdef CIRCLE_LINE_CALIB
	if(flag == 1)
		circle_out << g_calib_Point.X << " "<< g_calib_Point.Y << endl;
	else if(flag == 2)
		line_out << g_calib_Point.X << " "<< g_calib_Point.Y << endl;
#endif
}

#ifdef SINGLE_TEST

int
main(int argc, char *argv[])
{
	//time_t t1, t2;

	/*calib_line();
	calib_ellipse();
	system("pause");
	return 0;*/ 

	while(1)
	{
	if(dmInitial(PICONLY) == 0)
	{
		system("pause");
		return 0; 
	}

	//cameraSelect(2, PICONLY);

#ifdef CIRCLE_LINE_CALIB
	while(1)
	{
		if(dmStart(CIRCLE_CALIB, PICONLY) == 0)
		{
			//system("pause");
			//return 0;
			continue;
		}
		else
		{
			//continue;
			//break;
			point_save(CIRCLE_CALIB);
		}
	}

	/*while(1)
	{
		if(dmStart(LINE_CALIB, PICONLY) == 0)
		{
			continue;
		}
		else
		{
			point_save(LINE_CALIB);
		}
	}*/

	/*while(1)
	{
		if(dmStart(CIRCLE_CALIB, PICONLY) == 0)
		{
			continue;
		}
		else
		{
			point_save(CIRCLE_CALIB);
		}
	}*/
#else
	int k = 0;
	while(1)
	{

		if(dmStart(NON_CALIB, PICONLY) == 0)
		{
			//system("pause");
			//return 0;
		}
		else
		{
			//point_save(LINE_CALIB);
		}

		k++;
		if(k > 50)
			break;
	}
#endif

	dmExit(PICONLY);

	Sleep(100);
	}

	/*WaitForSingleObject(h_USB_getPic, INFINITE);

	CloseHandle(h_USB_getPic);
	DeleteCriticalSection(&g_csPicIndex);*/

	system("pause");
}

#endif

