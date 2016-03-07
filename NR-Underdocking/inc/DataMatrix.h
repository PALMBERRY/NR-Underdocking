#ifndef _DATAMATRIX_H
#define _DATAMATRIX_H

#include "config.h"
#include "project.h"

typedef struct AdjustInfo_vision_struct {
	double delta_x; // m
	double delta_y; // m
	double delta_angle; // °
	char quick_mark_name[80];
	double feedback_time;
	double detect_quickMark_used_time;
	double timeStamp;

	int err_code;
}AdjustInfo_vision;

enum
{
	PICONLY = 0x00,
	PICONLY_PICSAVE = 0x01,
	ADJRES = 0x02,
	ADJRES_PICSAVE = 0x03,
	ADJRES_ERRORPIC = 0x04
};

enum
{
	NON_CALIB = 0x00,
	CIRCLE_CALIB = 0x01,
	LINE_CALIB = 0x02,
	EX_PARAM_CALIB = 0x03,
	UP_CALIB = 0x04
};

#ifdef WIN32

#include "USB/cyusb_interface.h"

//struct AdjustInfo {
//	double delta_x; // m
//	double delta_y; // m
//	double delta_angle; // °
//	std::string quick_mark_name;
//	double timestamp;
//	double detect_quickMark_used_time;
//
//	AdjustInfo()
//	:delta_x(0), 
//	delta_y(0), 
//	delta_angle(0),
//	timestamp(0),
//	detect_quickMark_used_time(0)
//	{}
//};

extern AdjustInfo_vision result;

DWORD WINAPI USB_getPic(LPVOID lpParameter);

int dmInitial(uint8_t CV_mode);

int cameraSelect(U8 cmos_id, uint8_t CV_mode);

int dmStart(int calib_flag, uint8_t CV_mode);

void dmExit(uint8_t CV_mode);

void point_save(int flag);

//objecject_size为实际视野范围，单位为mm
//dm_obj_size为实际二维码尺寸，单位为mm
//label_obj_size为实际标签尺寸，单位为mm
//best_param为最佳分辨率，单位为像素/mm
void calc_param(double object_size, double dm_obj_size, double label_obj_size, double best_param);

#endif

#endif

#ifdef c6748

#ifndef SINGLE_TEST
#include "typedef.h"
#endif

extern AdjustInfo_vision g_adj_res;
extern int g_adj_flag;
extern unsigned long g_video_buf_addr;
extern volatile uint8_t g_error_mode;
extern volatile uint8_t g_CV_needed;

int dmInit();

int dmStart(uint8_t* src, AdjustInfo_vision* result);


#endif
