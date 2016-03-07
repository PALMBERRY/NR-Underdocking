#include <iostream>

#include "USB/typedef.h"
#include "USB/ErrorNum.h"

#include "USB/cyusb_interface.h"
#include <mmsystem.h>

#include "tic_toc.h"
#include "DataMatrix.h"
#include "project.h"

#pragma comment(lib, "winmm.lib")

/* define usb cmd from pc */ 
#define CMD_GET_ONE_FRAME		0x01
#define CMD_SET_CAM_REG			0x02
#define CMD_GET_CAM_REG			0x04
#define CMD_SLE_CCD				0x08   //0x03 0x01 代表选择CCD1 0x03 0x02 代表选择CCD2
#define CMD_GET_ADJ_RES			0X10
#define CMD_SLE_MODE			0x20
#define CMD_SET_CAMERA_PARAM	0x40

enum
{
	SELECT_CCD1=0x01,
	SELECT_CCD2=0x02
};

#define PIC_X_SIZE		480
#define PIC_Y_SIZE		480

U8	YBuf[PIC_Y_SIZE][PIC_X_SIZE];

CUsbDevice usbDev;

/*******************************************************************************
* Function Name   : SetCMOSReg
* Description	  : 设置cmos传感器寄存器值
*					
* Input		      : CUsbDevice *hUsb, U8 nRegAddr, U16 nValue
* Output		  : None
* Return		  : None.
*******************************************************************************/
static int SetCMOSReg(CUsbDevice *hUsb, U8 nRegAddr, U16 nValue )
{
	U8 WrBuf[4];
	int RetVal;

	long len;

	WrBuf[0] = CMD_SET_CAM_REG;	//set ccd reg cmd 
	WrBuf[1] = nRegAddr;
	WrBuf[2] = nValue & 0xff;
	WrBuf[3] = nValue >> 8;

	RetVal = hUsb->BulkEndPtOut(4 ,WrBuf);
	if (RetVal < 0)
	{
#if DEBUG
		printf("I2C写USB操作失败, 返回值:%d",RetVal);
#endif
		return RetVal;
	}

	len = 1;
	RetVal = hUsb->BulkEndPtIn(len, WrBuf);
	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if(WrBuf[0] != 0)
		{
			/* get picture timeout */
#if DEBUG
			printf("I2C操作失败, USB返回值:%d",(int)(WrBuf[0]));
#endif
			RetVal = -WrBuf[0];
		}
		else
		{
			RetVal = 0;
		}
	}
	else
	{
#if DEBUG
		printf("I2C读USB操作失败, 返回值:%d",RetVal);
#endif

	}

	return RetVal;
}

/*******************************************************************************
* Function Name   : GetOnePic
* Description	  : 获取一帧图像
*					
* Input		      : U8 *pYBuf, long *pTime time [ms]
* Output		  : None
* Return		  : None.
*******************************************************************************/
int GetOnePic( U8 *pYBuf, double *pTime)
{

	int RetVal;
	U8 SendBuf[4];
	long len;
	//DWORD StartTime, EndTime;
	LARGE_INTEGER start_time;

	if (pTime != NULL) *pTime = 0;

	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG 
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}

	//StartTime = timeGetTime();
	start_time = tic_1();

	/* send the get one frame cmd */
	SendBuf[0] = CMD_GET_ONE_FRAME;
	RetVal = usbDev.BulkEndPtOut(1 ,SendBuf);
	if (RetVal < 0)
	{
#if  DEBUG
		printf("USB发送命令错误");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal;
	}

	/* read back one frame */
	len = PIC_X_SIZE * PIC_Y_SIZE;
	RetVal=usbDev.BulkEndPtIn(len,pYBuf);

	//EndTime = timeGetTime();

	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if (pTime != NULL)
		{
			//*pTime = EndTime - StartTime;
			*pTime = toc_1(start_time);
		}

		if(len == 1)
		{
			/* get picture timeout */
#if DEBUG
			printf("读取图像时返回超时");
#endif
			RetVal = E_GET_PIC_TIME_OUT;
		}
	}
	else
	{
#if DEBUG
		printf("USB读操作失败");
#endif
		goto End;
	}

	RetVal = E_NO_ERR;

End:

	return RetVal;
}

int GetAdjRes_OnePic( U8 *pYBuf, double *pTime)
{

	int RetVal;
	U8 SendBuf[4];
	long len;
	//DWORD StartTime, EndTime;
	LARGE_INTEGER start_time;

	if (pTime != NULL) *pTime = 0;

	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG 
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}

	//StartTime = timeGetTime();
	start_time = tic_1();

	/* send the get one frame cmd */
	SendBuf[0] = CMD_GET_ONE_FRAME|CMD_GET_ADJ_RES;
	RetVal = usbDev.BulkEndPtOut(1 ,SendBuf);
	if (RetVal < 0)
	{
#if  DEBUG
		printf("USB发送命令错误");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal;
	}

	/* read back one frame */
	len = sizeof(AdjustInfo_vision)+PIC_X_SIZE * PIC_Y_SIZE;
	RetVal=usbDev.BulkEndPtIn(len, pYBuf);

	//EndTime = timeGetTime();

	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if (pTime != NULL)
		{
			//*pTime = EndTime - StartTime;
			*pTime = toc_1(start_time);
		}

		if(len == 1)
		{
			/* get picture timeout */
#if DEBUG
			printf("读取图像时返回超时");
#endif
			RetVal = E_GET_PIC_TIME_OUT;
		}
	}
	else
	{
#if DEBUG
		printf("USB读操作失败");
#endif
		goto End;
	}

	RetVal = E_NO_ERR;

End:

	return RetVal;
}

int GetAdjRes( U8 *pYBuf, double *pTime)
{	
	int RetVal;
	U8 SendBuf[4];
	long len;
	//DWORD StartTime, EndTime;
	LARGE_INTEGER start_time;

	if (pTime != NULL) *pTime = 0;

	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG 
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}

	//StartTime = timeGetTime();
	start_time = tic_1();

	/* send the get one frame cmd */
	SendBuf[0] = CMD_GET_ADJ_RES;
	RetVal = usbDev.BulkEndPtOut(1 ,SendBuf);
	if (RetVal < 0)
	{
#if  DEBUG
		printf("USB发送命令错误");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal;
	}

	/* read back one frame */
	len = sizeof(AdjustInfo_vision);
	RetVal=usbDev.BulkEndPtIn(len, pYBuf);

	//EndTime = timeGetTime();

	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if (pTime != NULL)
		{
			//*pTime = EndTime - StartTime;
			*pTime = toc_1(start_time);
		}

		if(len == 1)
		{
			/* get picture timeout */
#if DEBUG
			printf("读取图像时返回超时");
#endif
			RetVal = E_GET_PIC_TIME_OUT;
		}
	}
	else
	{
#if DEBUG
		printf("USB读操作失败");
#endif
		goto End;
	}

	RetVal = E_NO_ERR;

End:

	return RetVal;
}

static int SetCameraParam()
{
	int RetVal;
	U8 SendBuf[sizeof(Camera_Param)+1];
	long len;

	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}

	SendBuf[0] = CMD_SET_CAMERA_PARAM;
	memcpy(SendBuf+1, cur_camera, sizeof(Camera_Param));

	RetVal = usbDev.BulkEndPtOut(sizeof(Camera_Param)+1 ,SendBuf);
	if (RetVal < 0)
	{
#if DEBUG
		printf("USB发送命令错误");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal ;
	}

	/* read back one frame */
	len = 1;
	RetVal=usbDev.BulkEndPtIn(len,SendBuf);

	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if(SendBuf[0] != 0)
		{
			/* get picture timeout */
#if DEBUG
			printf("摄像头选择操作失败, USB返回值:%d",(int)(SendBuf[0]));
#endif
			RetVal = -SendBuf[0];
		}
		else
		{
			RetVal = 0;
#if DEBUG
			printf("选择摄像头1成功");
#endif
		}
	}
	else
	{	
#if DEBUG
		printf("读USB返回值操作失败");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal ;
	}
	RetVal =  E_NO_ERR;
	return RetVal ;
}

/*******************************************************************************
* Function Name   : Init_Usb_Device
* Description	  : 初始化整个usb设备/相机板卡
*					
* Input		      : U8 cmos_id
* Output		  : None
* Return		  : None.
*******************************************************************************/
int Init_Usb_Device(U8 CV_mode)
{
	//CUsbDevice usbDev;
	int RetVal;

	RetVal = usbDev.Open();
	if(RetVal < 0)
	{ 
#if DEBUG
		printf("USB设备未连接");
#endif
		return RetVal;
	}

#ifdef IAGV
	RetVal = Select_Camera(0x02);
#else
	RetVal = Select_Camera(0x01);
#endif
	if(RetVal < 0)
	{ 
#if DEBUG
		printf("选择相机模式失败");
#endif
		return RetVal;
	}

	RetVal = Select_Mode(CV_mode);
	if(RetVal < 0)
	{ 
#if DEBUG
		printf("选择DSP模式失败");
#endif
		return RetVal;
	}

	return RetVal;
}
/*******************************************************************************
* Function Name   : Select_Cmos_Id
* Description	  : 选择cmos传感器id
*					
* Input		      : U8 cmos_id
* Output		  : None
* Return		  : None.
*******************************************************************************/
int Select_Cmos_Id(U8 cmos_id)
{
	int RetVal;
	U8 SendBuf[4];
	long len;

	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}
	/* send the select ccd1 for board */
	if((cmos_id !=SELECT_CCD1)&&(cmos_id !=SELECT_CCD2))
	{
		RetVal = E_PARAMETER_ERROR;
		return RetVal;
	}
	SendBuf[0] = CMD_SLE_CCD;
	SendBuf[1] = cmos_id;

	RetVal = usbDev.BulkEndPtOut(2 ,SendBuf);
	if (RetVal < 0)
	{
#if DEBUG
		printf("USB发送命令错误");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal ;
	}

	/* read back one frame */
	len = 1;
	RetVal=usbDev.BulkEndPtIn(len,SendBuf);

	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if(SendBuf[0] != 0)
		{
			/* get picture timeout */
#if DEBUG
			printf("摄像头选择操作失败, USB返回值:%d",(int)(SendBuf[0]));
#endif
			RetVal = -SendBuf[0];
		}
		else
		{
			RetVal = 0;
#if DEBUG
			printf("选择摄像头1成功");
#endif
		}
	}
	else
	{	
#if DEBUG
		printf("读USB返回值操作失败");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal ;
	}
	RetVal =  E_NO_ERR;
	return RetVal ;
}

int Select_Mode(U8 CV_mode)
{
	int RetVal;
	U8 SendBuf[4];
	long len;

	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}
	/* send the select ccd1 for board */
	if((CV_mode != CV_NEEDED)&&(CV_mode != CV_NOT_NEEDED)&&(CV_mode != ERROR_PIC))
	{
		RetVal = E_PARAMETER_ERROR;
		return RetVal;
	}
	SendBuf[0] = CMD_SLE_MODE;
	SendBuf[1] = CV_mode;

	RetVal = usbDev.BulkEndPtOut(2 ,SendBuf);
	if (RetVal < 0)
	{
#if DEBUG
		printf("USB发送命令错误");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal ;
	}

	/* read back one frame */
	len = 1;
	RetVal=usbDev.BulkEndPtIn(len,SendBuf);

	if(RetVal == E_RECEIVE_SUCCES)
	{ 
		if(SendBuf[0] != 0)
		{
			/* get picture timeout */
#if DEBUG
			printf("图像处理模式选择操作失败, USB返回值:%d",(int)(SendBuf[0]));
#endif
			RetVal = -SendBuf[0];
		}
		else
		{
			//RetVal = 0;
#if DEBUG
			printf("选择摄像头1成功");
#endif
		}
	}
	else
	{	
#if DEBUG
		printf("读USB返回值操作失败");
#endif
		usbDev.Close();
		usbDev.usb_status = 0;
		return RetVal ;
	}
	RetVal =  E_NO_ERR;
	return RetVal ;
}

/*******************************************************************************
* Function Name   : Set_CMOS_Register
* Description	  : 设置cmos传感器参数
*					
* Input		      : SET_CMOS_REG_STRUCT cmos_register
* Output		  : None
* Return		  : None.
*******************************************************************************/
int Set_CMOS_Register(SET_CMOS_REG_STRUCT cmos_register)
{	
	SET_CMOS_REG_STRUCT cmos_reg;

	U16 RegTab[30][2];
	int i;

	int RetVal;

	cmos_reg=cmos_register;
	if((cmos_reg.exposure_time>32765)||(cmos_reg.exposure_time<1))
	{
#if DEBUG
		printf("曝光时间必须大于0行，小于32765行");
#endif
		RetVal = E_PARAMETER_ERROR;
		return RetVal ;
	}	

	RegTab[0][0] = 0x0b;
	RegTab[1][0] = 0x35;

	RegTab[0][1] = cmos_reg.exposure_time;
	RegTab[1][1] = cmos_reg.analoge_gain;

	for(i = 0; i < 25; i++)
	{
		RegTab[i+2][0] = 0x80 + i;
		RegTab[i+2][1] = cmos_reg.gain_x0y0;
	}
	if(usbDev.usb_status == 0)
	{
		RetVal = usbDev.Open();
		if(RetVal < 0)
		{
#if DEBUG
			printf("USB设备未连接");
#endif
			return RetVal;
		}
	}


	for(i = 0; i < 27; i++)
	{
		RetVal = SetCMOSReg(&usbDev, (U8)(RegTab[i][0] & 0xff), RegTab[i][1]);
		if(RetVal != E_NO_ERR) 
			break;
	}

	return RetVal;
}

int Select_Camera(U8 cmos_id)
{
	int RetVal;

	/*int coarse;
	std::cout<<"输入曝光度：";

	std::cin>>coarse;*/

	if(cmos_id == 0x01)
	{
		INIT_CMOS_REG_STRUCT cmos_data;
		cmos_data.column_start = 136;
		cmos_data.window_height = SRC_IMG_Y;
		cmos_data.window_width = SRC_IMG_X;
		cmos_data.horizon_blank = 548;
		cmos_data.chip_control = 0x388;
		cmos_data.aec_agc = 0x0;
		cmos_data.coarse_shutter_width = 10;//coarse;//280;//10;//280;
		cmos_data.analoge_gain = 16;

		U16 RegInitTab[8][2] = {
			{0x01, cmos_data.column_start}, 
			{0x03, cmos_data.window_height},
			{0x04, cmos_data.window_width}, 
			{0x05, cmos_data.horizon_blank}, 
			{0x07, cmos_data.chip_control}, 
			{0xaf, cmos_data.aec_agc},
			{0x0b, cmos_data.coarse_shutter_width},
			{0x35, cmos_data.analoge_gain}, 
		};

		RetVal = Select_Cmos_Id(0x01);
		if(RetVal < 0)
		{ 
#if DEBUG
			printf("选择CCD失败");
#endif
			return RetVal;
		}

		RetVal = SetCameraParam();
		if(RetVal < 0)
		{ 
#if DEBUG
			printf("设置相机参数失败");
#endif
			return RetVal;
		}

		for(int i = 0; i < 8; i++)
		{
			RetVal = SetCMOSReg(&usbDev, (U8)(RegInitTab[i][0] & 0xff), RegInitTab[i][1]);
			if(RetVal != E_NO_ERR) 
				break;
		}
	}
	else
	{
		INIT_CMOS_REG_STRUCT cmos_data;
		cmos_data.column_start = 136;
		cmos_data.window_height = SRC_IMG_Y;
		cmos_data.window_width = SRC_IMG_X;
		cmos_data.horizon_blank = 548;
		cmos_data.chip_control = 0x388;
		cmos_data.aec_agc = 0x0;
		cmos_data.coarse_shutter_width = 8;//coarse;//280;//8;//280;
		cmos_data.analoge_gain = 16;

		U16 RegInitTab[8][2] = {
			{0x01, cmos_data.column_start}, 
			{0x03, cmos_data.window_height},
			{0x04, cmos_data.window_width}, 
			{0x05, cmos_data.horizon_blank}, 
			{0x07, cmos_data.chip_control}, 
			{0xaf, cmos_data.aec_agc},
			{0x0b, cmos_data.coarse_shutter_width},
			{0x35, cmos_data.analoge_gain}, 
		};

		RetVal = Select_Cmos_Id(0x02);
		if(RetVal < 0)
		{ 
#if DEBUG
			printf("选择CCD失败");
#endif
			return RetVal;
		}

		RetVal = SetCameraParam();
		if(RetVal < 0)
		{ 
#if DEBUG
			printf("设置相机参数失败");
#endif
			return RetVal;
		}

		for(int i = 0; i < 8; i++)
		{
			RetVal = SetCMOSReg(&usbDev, (U8)(RegInitTab[i][0] & 0xff), RegInitTab[i][1]);
			if(RetVal != E_NO_ERR) 
				break;
		}
	}

	return RetVal;
}


void closeUSB()
{
	usbDev.Close();
	usbDev.usb_status = 0;
}

int restartUSB()
{
	int RetVal;

	usbDev.Close();
	usbDev.usb_status = 0;

	Sleep(125);

	RetVal = usbDev.Open();
	if(RetVal < 0)
	{
#if DEBUG
		printf("USB设备未连接");
#endif
		return RetVal;
	}
}