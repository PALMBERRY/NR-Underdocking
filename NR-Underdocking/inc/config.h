#ifndef _CONFIG_H
#define _CONFIG_H

//#ifdef __cplusplus
//extern "C" {
//#endif

//#ifndef STDINT_H_
//#include <stdint.h>
//#endif

#include "tic_toc.h"

#ifndef INTSTD
typedef signed char        int8_t;
typedef short              int16_t;
typedef int                int32_t;
typedef long long          int64_t;
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;

typedef signed char        int_least8_t;
typedef short              int_least16_t;
typedef int                int_least32_t;
typedef long long          int_least64_t;
typedef unsigned char      uint_least8_t;
typedef unsigned short     uint_least16_t;
typedef unsigned int       uint_least32_t;
typedef unsigned long long uint_least64_t;

typedef signed char        int_fast8_t;
typedef int                int_fast16_t;
typedef int                int_fast32_t;
typedef long long          int_fast64_t;
typedef unsigned char      uint_fast8_t;
typedef unsigned int       uint_fast16_t;
typedef unsigned int       uint_fast32_t;
typedef unsigned long long uint_fast64_t;

#endif

#ifdef WIN32

#define _rcpsp(x)	(1.0/(x))			//float倒数
#define _rcpdp(x)	(1.0/(x))			//double倒数
#define _rsqrsp(x)	(1.0/sqrt(x))		//float开方的倒数
#define _spint(x)	((int)(x+0.5))		//float转int,round mode
#define _dpint(x)	((int)(x+0.5))		//double转int,round mode
#define _fabsf		fabsf				//float绝对值
#define _fabs		fabs				//double绝对值
#define sqrtsp_i	sqrtf				//float的开方运算
#define sqrtdp_i	sqrt				//double的开方运算
#define expsp_i		exp					//float的指数运算
#define sinsp_i		sinf				//float的sin运算
#define sindp_i		sin					//double的sin运算
#define cossp_i		cosf				//float的cos运算
#define cosdp_i		cos					//double的cos运算
#define atansp_i	atanf				//float的atan运算
#define atandp_i	atan				//double的atan运算
#define atan2sp_i	atan2f				//float的atan2运算
#define atan2dp_i	atan2				//double的atan2运算

#define restrict
#define _nassert(x)

#endif

#ifdef c6748
#include <ti/mathlib/src/sqrtdp/sqrtdp.h>
#include <ti/mathlib/src/sqrtsp/sqrtsp.h>
#include <ti/mathlib/src/expsp/expsp.h>
#include <ti/mathlib/src/sinsp/sinsp.h>
#include <ti/mathlib/src/sindp/sindp.h>
#include <ti/mathlib/src/cossp/cossp.h>
#include <ti/mathlib/src/cosdp/cosdp.h>
#include <ti/mathlib/src/atansp/atansp.h>
#include <ti/mathlib/src/atandp/atandp.h>
#include <ti/mathlib/src/atan2sp/atan2sp.h>
#include <ti/mathlib/src/atan2dp/atan2dp.h>
#endif

// 定义最大最小值的宏
#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)<(y)?(x):(y))

#define PI 3.1415926

//图像和参数路径
#define	TABLE_PATH				"C:\\NJ_workspace\\dat\\table\\%s"
#define TEST_IMG_PATH			"C:\\NJ_workspace\\testimg\\filename.txt"
#define RUNNING_IMG_PATH		"C:\\NJ_workspace\\testimg\\%d.bmp"
#define TMP_IMG_PATH			"C:\\NJ_workspace\\testimg\\a.bmp"
#define SRC_IMG_PATH			"C:\\NJ_workspace\\srcimg\\%d.bmp"
#define DST_IMG_PATH			"C:\\NJ_workspace\\dstimg\\%d.bmp"
#define CAMERA1_PARAM_PATH		"C:\\NJ_workspace\\calibration\\camera1\\%s"
#define CAMERA2_PARAM_PATH		"C:\\NJ_workspace\\calibration\\camera2\\%s"
#define TMP_PARAM_PATH			"C:\\NJ_workspace\\NJ_calibration\\calibration\\%s"
#define POINT_CIRCLE_PATH		"C:\\NJ_workspace\\NJ_calibration\\point_circle.txt"
#define POINT_CIRCLE_SAVE_PATH	"C:\\NJ_workspace\\NJ_calibration\\calibration\\center_point.txt"
#define POINT_LINE_PATH			"C:\\NJ_workspace\\NJ_calibration\\point_line.txt"
#define POINT_LINE_SAVE_PATH	"C:\\NJ_workspace\\NJ_calibration\\calibration\\positive_direction.txt"
#define LOG_PATH				"C:\\NJ_workspace\\log.txt"
#define TMP_LOG_PATH			"C:\\NJ_workspace\\tmp_log.txt"

//二维码区域边长为20.5*2.29=46.95（像素）
//标签总边长为35*2.29=80.15（像素）

//二维码粗定位
#define DM_ONE_SIZE				6								//二维码一格的像素长度（考虑模糊边界，在真实尺寸4.7基础上适当放大）
#define GROUND_THRES			100								//背景与标签白底的绝对阈值
//#define WHITE_LENGTH			17//25								//(int)(2.29*(35-2.05)/2)二维码区域与标签白底边界的距离
#define WHITE_LENGTH_2			8								//(int)(2.29*(35-20.5)/2/2),向下取整
#define DM_THRES				90								//二维码与标签白底的绝对阈值
#define SEARCH_SIZE				112//120								//(int)(标签总边长*sqrt(2)),向下取整
#define MAX_THROUGH_POINT		200								//畸变图中最大二维码边界点数
#define LEAST_POINT				16								//二维码边界点的最小值
#define MAX_THROUGH_DIFF		34//38								//(int)(二维码区域边长*sqrt(2)/2)，向上取整，二维码边界点的最大离散距离

#define GRID_SIZE				30				//网格尺寸
#define GRID_CNT				16				//网格数目
#define LEAST_POINT_GAI			3
#define MAX_DIFF				60
#define MIN_MIN_K				4//3//4//4
#define MIN_MAX_K				20//9//14//20			第一代为14，第二代为20	
#define MAX_MAX_K				24//12//22//24			第一代为22，第二代为24

//#define CALC_DM_DISTORT_IMG_SIZE
#define	DM_DISTORT_IMG_SIZE		116//120								//粗定位畸变图大小(由粗定位投影图大小决定）:116对应88，
#define DM_DISTORT_IMG_SIZE_2	(DM_DISTORT_IMG_SIZE>>1)


//原始图像尺寸（图像分辨率）
#define SRC_IMG_X	480
#define SRC_IMG_Y	480

//投影图像尺寸（图像分辨率）
#define DOWN_PROJ_IMG_SIZE_FULL	376

#define UP_PROJ_IMG_SIZE_FULL	376//288

//投影图像对应实际空间尺寸(单位mm)
#define DOWN_OBJECT_SIZE	165

#define UP_OBJECT_SIZE		165//125

//投影图与世界坐标比例尺，即一个像素对应空间尺寸(mm)
//#define PROJ_RATIO_X (((double)OBJECT_X)/PROJ_IMG_X)
//#define PROJ_RATIO_Y (((double)OBJECT_Y)/PROJ_IMG_Y)

//#define MIN_DIFF 1E-4
//#define ITERATE_TIME 6


//搜索种子点时的梯度阈值
#define	EDGE_THRESHOLD	77
//线上点最大保留个数
#define LINE_POINT_NUM	150 
//轮廓最大点
#define OUTLINE_POINT_NUM	300		//g_maxDiagonal*4

#define ANGLE_SEARCH_RANGE			15		//实线边的霍夫变换的角度搜索范围
#define VERTICAL_ANGLE_SEARCH_RANGE	2		//虚线边的霍夫变换的角度搜索范围

#ifdef c6748
#define USE_DMA
#endif

#define IAGV

#ifdef IAGV
#define UP_EDGE_MIN		20
#define DOWN_EDGE_MIN	42
#define EDGE_MAX		200	//关注g_maxDiagonal的修改
#else
#define UP_EDGE_MIN		32
#define DOWN_EDGE_MIN	42
#define EDGE_MAX		52	//关注g_maxDiagonal的修改
#endif

//#define FULL_PIC_PROJECT
//#define CIRCLE_LINE_CALIB

#ifdef CIRCLE_LINE_CALIB
#ifndef FULL_PIC_PROJECT
#define FULL_PIC_PROJECT
#endif
#endif

#define UP_PROJ_IMG_SIZE		376//288								//粗定位投影图大小
#define UP_PROJ_IMG_SIZE_2		(UP_PROJ_IMG_SIZE>>1)

#ifdef FULL_PIC_PROJECT
#define DOWN_PROJ_IMG_SIZE			376								//粗定位投影图大小
#define DOWN_PROJ_IMG_SIZE_2		(DOWN_PROJ_IMG_SIZE>>1)
#else
#define DOWN_PROJ_IMG_SIZE			104//88								//粗定位投影图大小
#define DOWN_PROJ_IMG_SIZE_2		(DOWN_PROJ_IMG_SIZE>>1)
#endif

#define CARMERA_2

//#define PRINT_LOC
//#define WATCH_MODE

#ifdef WATCH_MODE
#define DM_PROJ_IMG_SIZE	376//104//88
#endif

//#define TIME_TEST

//#define CHECK_MODE
//#define STATIC_PIC_TEST
//#define STATIC_PIC_TEST_RUNNING
//#define SINGLE_TEST

#define SHOW_LOCATION
//#define SHOW_RESULT

//#define DST_IMG_SAVE
#define LOG_OUT

//#define TAKE_A_PICTURE
//#define PRINT_LEN

//#ifdef __cplusplus
//}
//#endif

#endif
