#ifndef _CYUSB_INTERFACE_H
#define _CYUSB_INTERFACE_H

#include<stdio.h>
#include "cyusb_driver.h"
#include "typedef.h"

#define DEBUG 0

enum
{
	CV_NOT_NEEDED = 0x00,	//不进行图像处理
	CV_NEEDED = 0x01,		//进行图像处理
	ERROR_PIC = 0x02		//进行图像处理并上传错图
};

typedef struct
{
	U16 column_start;   //列起始 (752-480)/2=136
	U16 window_height;  //image height 480
	U16 window_width;   //480
	U16 horizon_blank;  //366+182=548
	U16 chip_control;   //0x388
	U16 aec_agc;        //0x0
	U16 coarse_shutter_width; //Exposure time 280
	U16 analoge_gain;   //16 analog gain X1.0 Analog Gain:default(0X10)-gainX1  0X20-gainX2 

}INIT_CMOS_REG_STRUCT;


typedef struct
{
	U16 exposure_time;   //曝光时间 0~32765
	U16 analoge_gain;  //16 analog gain X1.0 Analog Gain:default(0X10)-gainX1  0X20-gainX2 
	U16 gain_x0y0;      //0x04f1~0x04FF
	/*U16 gain_x1y0;
	U16 gain_x2y0;
	U16 gain_x3y0;
	U16 gain_x4y0;
	U16 gain_x0y1;
	U16 gain_x1y1;
	U16 gain_x2y1;
	U16 gain_x3y1;
	U16 gain_x4y1;
	U16 gain_x0y2;
	U16 gain_x1y2;
	U16 gain_x2y2;
	U16 gain_x3y2;
	U16 gain_x4y2;
	U16 gain_x0y3;
	U16 gain_x1y3;
	U16 gain_x2y3;
	U16 gain_x3y3;
	U16 gain_x4y3;
	U16 gain_x0y4;
	U16 gain_x1y4;
	U16 gain_x2y4;
	U16 gain_x3y4;
	U16 gain_x4y4;*/
	
}SET_CMOS_REG_STRUCT;

/* 初始化整个USB设备/相机板卡
* cmos_data 		INIT_CMOS_REG_STRUCT结构体
* CV_mode		DSP图像处理模式，CV_NOT_NEEDED = 0x00, CV_NEEDED = 0x01, ERROR_PIC = 0x02
* 返回值			为0表示成功，小于0表示失败*/
int Init_Usb_Device(U8 CV_mode);

/* 选择cmos传感器id
* cmos_id			0x01表示选择相机1，0x02表示选择相机2
* 返回值			为0表示成功，小于0表示失败*/
int Select_Cmos_Id(U8 cmos_id);

/* 获取一帧图像
* pYBuf			保存480*480Byte的图像数据指针
* pTime			获取一帧图像时上位机等待时间
* 返回值		为0表示成功，小于0表示失败	*/
int GetOnePic( U8 *pYBuf, double *pTime);

/* 设置cmos传感器参数
* cmos_register		 SET_CMOS_REG_STRUCT结构体
* 返回值			为0表示成功，小于0表示失败*/
int Set_CMOS_Register(SET_CMOS_REG_STRUCT cmos_register);

/* 选择DSP图像处理工作模式
* CV_mode		DSP图像处理模式
* 返回值		为0表示成功，小于0表示失败*/
int Select_Mode(U8 CV_mode);

/* 获取图像处理反馈结果
* pYBuf			保存反馈结果结构体地址
* pTime			获取反馈结果时上位机等待时间
* 返回值			为0表示成功，小于0表示失败*/	
int GetAdjRes( U8 *pYBuf, double *pTime);

/* 获取图像处理反馈结果及该帧图像
* pYBuf			保存反馈结果结构体及该帧图像的地址
* pTime			获取反馈结果结构体及该帧图像时上位机等待时间
* 返回值		为0表示成功，小于0表示失败*/
int GetAdjRes_OnePic( U8 *pYBuf, double *pTime);

int Select_Camera(U8 cmos_id);

void closeUSB();

int restartUSB();

#endif