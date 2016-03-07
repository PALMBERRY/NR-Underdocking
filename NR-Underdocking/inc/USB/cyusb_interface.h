#ifndef _CYUSB_INTERFACE_H
#define _CYUSB_INTERFACE_H

#include<stdio.h>
#include "cyusb_driver.h"
#include "typedef.h"

#define DEBUG 0

enum
{
	CV_NOT_NEEDED = 0x00,	//������ͼ����
	CV_NEEDED = 0x01,		//����ͼ����
	ERROR_PIC = 0x02		//����ͼ�����ϴ���ͼ
};

typedef struct
{
	U16 column_start;   //����ʼ (752-480)/2=136
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
	U16 exposure_time;   //�ع�ʱ�� 0~32765
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

/* ��ʼ������USB�豸/����忨
* cmos_data 		INIT_CMOS_REG_STRUCT�ṹ��
* CV_mode		DSPͼ����ģʽ��CV_NOT_NEEDED = 0x00, CV_NEEDED = 0x01, ERROR_PIC = 0x02
* ����ֵ			Ϊ0��ʾ�ɹ���С��0��ʾʧ��*/
int Init_Usb_Device(U8 CV_mode);

/* ѡ��cmos������id
* cmos_id			0x01��ʾѡ�����1��0x02��ʾѡ�����2
* ����ֵ			Ϊ0��ʾ�ɹ���С��0��ʾʧ��*/
int Select_Cmos_Id(U8 cmos_id);

/* ��ȡһ֡ͼ��
* pYBuf			����480*480Byte��ͼ������ָ��
* pTime			��ȡһ֡ͼ��ʱ��λ���ȴ�ʱ��
* ����ֵ		Ϊ0��ʾ�ɹ���С��0��ʾʧ��	*/
int GetOnePic( U8 *pYBuf, double *pTime);

/* ����cmos����������
* cmos_register		 SET_CMOS_REG_STRUCT�ṹ��
* ����ֵ			Ϊ0��ʾ�ɹ���С��0��ʾʧ��*/
int Set_CMOS_Register(SET_CMOS_REG_STRUCT cmos_register);

/* ѡ��DSPͼ������ģʽ
* CV_mode		DSPͼ����ģʽ
* ����ֵ		Ϊ0��ʾ�ɹ���С��0��ʾʧ��*/
int Select_Mode(U8 CV_mode);

/* ��ȡͼ���������
* pYBuf			���淴������ṹ���ַ
* pTime			��ȡ�������ʱ��λ���ȴ�ʱ��
* ����ֵ			Ϊ0��ʾ�ɹ���С��0��ʾʧ��*/	
int GetAdjRes( U8 *pYBuf, double *pTime);

/* ��ȡͼ�������������֡ͼ��
* pYBuf			���淴������ṹ�弰��֡ͼ��ĵ�ַ
* pTime			��ȡ��������ṹ�弰��֡ͼ��ʱ��λ���ȴ�ʱ��
* ����ֵ		Ϊ0��ʾ�ɹ���С��0��ʾʧ��*/
int GetAdjRes_OnePic( U8 *pYBuf, double *pTime);

int Select_Camera(U8 cmos_id);

void closeUSB();

int restartUSB();

#endif