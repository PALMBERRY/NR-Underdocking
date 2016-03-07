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

#define _rcpsp(x)	(1.0/(x))			//float����
#define _rcpdp(x)	(1.0/(x))			//double����
#define _rsqrsp(x)	(1.0/sqrt(x))		//float�����ĵ���
#define _spint(x)	((int)(x+0.5))		//floatתint,round mode
#define _dpint(x)	((int)(x+0.5))		//doubleתint,round mode
#define _fabsf		fabsf				//float����ֵ
#define _fabs		fabs				//double����ֵ
#define sqrtsp_i	sqrtf				//float�Ŀ�������
#define sqrtdp_i	sqrt				//double�Ŀ�������
#define expsp_i		exp					//float��ָ������
#define sinsp_i		sinf				//float��sin����
#define sindp_i		sin					//double��sin����
#define cossp_i		cosf				//float��cos����
#define cosdp_i		cos					//double��cos����
#define atansp_i	atanf				//float��atan����
#define atandp_i	atan				//double��atan����
#define atan2sp_i	atan2f				//float��atan2����
#define atan2dp_i	atan2				//double��atan2����

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

// ���������Сֵ�ĺ�
#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)<(y)?(x):(y))

#define PI 3.1415926

//ͼ��Ͳ���·��
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

//��ά������߳�Ϊ20.5*2.29=46.95�����أ�
//��ǩ�ܱ߳�Ϊ35*2.29=80.15�����أ�

//��ά��ֶ�λ
#define DM_ONE_SIZE				6								//��ά��һ������س��ȣ�����ģ���߽磬����ʵ�ߴ�4.7�������ʵ��Ŵ�
#define GROUND_THRES			100								//�������ǩ�׵׵ľ�����ֵ
//#define WHITE_LENGTH			17//25								//(int)(2.29*(35-2.05)/2)��ά���������ǩ�׵ױ߽�ľ���
#define WHITE_LENGTH_2			8								//(int)(2.29*(35-20.5)/2/2),����ȡ��
#define DM_THRES				90								//��ά�����ǩ�׵׵ľ�����ֵ
#define SEARCH_SIZE				112//120								//(int)(��ǩ�ܱ߳�*sqrt(2)),����ȡ��
#define MAX_THROUGH_POINT		200								//����ͼ������ά��߽����
#define LEAST_POINT				16								//��ά��߽�����Сֵ
#define MAX_THROUGH_DIFF		34//38								//(int)(��ά������߳�*sqrt(2)/2)������ȡ������ά��߽��������ɢ����

#define GRID_SIZE				30				//����ߴ�
#define GRID_CNT				16				//������Ŀ
#define LEAST_POINT_GAI			3
#define MAX_DIFF				60
#define MIN_MIN_K				4//3//4//4
#define MIN_MAX_K				20//9//14//20			��һ��Ϊ14���ڶ���Ϊ20	
#define MAX_MAX_K				24//12//22//24			��һ��Ϊ22���ڶ���Ϊ24

//#define CALC_DM_DISTORT_IMG_SIZE
#define	DM_DISTORT_IMG_SIZE		116//120								//�ֶ�λ����ͼ��С(�ɴֶ�λͶӰͼ��С������:116��Ӧ88��
#define DM_DISTORT_IMG_SIZE_2	(DM_DISTORT_IMG_SIZE>>1)


//ԭʼͼ��ߴ磨ͼ��ֱ��ʣ�
#define SRC_IMG_X	480
#define SRC_IMG_Y	480

//ͶӰͼ��ߴ磨ͼ��ֱ��ʣ�
#define DOWN_PROJ_IMG_SIZE_FULL	376

#define UP_PROJ_IMG_SIZE_FULL	376//288

//ͶӰͼ���Ӧʵ�ʿռ�ߴ�(��λmm)
#define DOWN_OBJECT_SIZE	165

#define UP_OBJECT_SIZE		165//125

//ͶӰͼ��������������ߣ���һ�����ض�Ӧ�ռ�ߴ�(mm)
//#define PROJ_RATIO_X (((double)OBJECT_X)/PROJ_IMG_X)
//#define PROJ_RATIO_Y (((double)OBJECT_Y)/PROJ_IMG_Y)

//#define MIN_DIFF 1E-4
//#define ITERATE_TIME 6


//�������ӵ�ʱ���ݶ���ֵ
#define	EDGE_THRESHOLD	77
//���ϵ����������
#define LINE_POINT_NUM	150 
//��������
#define OUTLINE_POINT_NUM	300		//g_maxDiagonal*4

#define ANGLE_SEARCH_RANGE			15		//ʵ�߱ߵĻ���任�ĽǶ�������Χ
#define VERTICAL_ANGLE_SEARCH_RANGE	2		//���߱ߵĻ���任�ĽǶ�������Χ

#ifdef c6748
#define USE_DMA
#endif

#define IAGV

#ifdef IAGV
#define UP_EDGE_MIN		20
#define DOWN_EDGE_MIN	42
#define EDGE_MAX		200	//��עg_maxDiagonal���޸�
#else
#define UP_EDGE_MIN		32
#define DOWN_EDGE_MIN	42
#define EDGE_MAX		52	//��עg_maxDiagonal���޸�
#endif

//#define FULL_PIC_PROJECT
//#define CIRCLE_LINE_CALIB

#ifdef CIRCLE_LINE_CALIB
#ifndef FULL_PIC_PROJECT
#define FULL_PIC_PROJECT
#endif
#endif

#define UP_PROJ_IMG_SIZE		376//288								//�ֶ�λͶӰͼ��С
#define UP_PROJ_IMG_SIZE_2		(UP_PROJ_IMG_SIZE>>1)

#ifdef FULL_PIC_PROJECT
#define DOWN_PROJ_IMG_SIZE			376								//�ֶ�λͶӰͼ��С
#define DOWN_PROJ_IMG_SIZE_2		(DOWN_PROJ_IMG_SIZE>>1)
#else
#define DOWN_PROJ_IMG_SIZE			104//88								//�ֶ�λͶӰͼ��С
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
