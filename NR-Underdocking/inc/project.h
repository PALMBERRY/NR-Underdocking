#ifndef _PROJECT_H_
#define _PROJECT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
#include "libdmtx/dmtx.h"

typedef struct _Inter_Linear_Table
{
	uint16_t x;
	uint16_t y;
	uint16_t a;
	uint16_t b;
}Inter_Linear_Table;

typedef struct _Camera_Param
{
	int camera_id;
	float inParam[9];			//相机内参
	float exParamR[9];			//相机外参旋转矩阵
	float exParamR_[9];			//相机外参旋转矩阵的逆
	float exParamt[3];			//相机外参平移向量
	float distortion[4];		//相机畸变参数
	DmtxVector2 center_point;	//中心点
	double dir_angle;			//正方向
	double coeffs_x;			//空间坐标偏移量
	double coeffs_y;			
}Camera_Param;

extern DmtxVector2 g_center_p;

//正方向斜率及其角度
extern double p_dir_angle;

extern int g_dm_proj_img_size;
extern int g_dm_proj_img_size_2;
extern Inter_Linear_Table* g_proj_table;
extern double g_object_size;
extern double g_proj_ratio;
extern double g_proj_ratio_;

extern Camera_Param* cur_camera;

//预计算空间偏移量
void calcObjCoeffs();

//预计算生成投影图表格
void GeneratePrjTable();

//设置尺寸参数
//@param 四组参数
//void SetSizeParam(int i,...);

#ifdef WIN32
//将投影图表格保存成离线格式
//@param 文件名
void SaveTable();
#endif


//从文件读取相机参数
//@param 参数路径名
int LoadCamParam(char* pathname);

#ifdef WIN32
int LoadCamParam_calib(char* pathname);
#endif

int SetCamParam();

//对一副图像进行投影变换
//@param 需变换的图像

void Project2_one_step(const uint8_t* srcImg, uint8_t* restrict dstImg);

void Project3_one_step(const uint8_t* srcImg, uint8_t* restrict dstImg, DmtxVector2 ori_loc, DmtxPixelLoc* start_point_project);

/* 去畸变与投影变换，得到二维码区域投影图。
* srcImg			输入图像
* dstImg			输出图像
* ori_loc			DM码中心点粗定位结果
* start_point_project	二维码区域投影图在完整投影图中的相对位置		
* 返回值			DmtxPass表示成功，DmtxFail表示失败*/
void Project3_one_step_DSP(const uint8_t* srcImg, uint8_t* restrict dstImg, DmtxVector2 ori_loc, DmtxPixelLoc* start_point_project);

//畸变点转去畸变点
void distort2undistortPoint( const DmtxVector2 distort_point, DmtxVector2* undistort_point);

//去畸变点转投影点
void undistort2projectPoint(const DmtxVector2 undistort_point, DmtxVector2* project_point);

//投影点转去畸变点
void project2undistortPoint(const DmtxVector2 project_point, DmtxVector2* undistort_point);

//去畸变点转畸变点
void undistort2distortPoint(const DmtxVector2 undistort_point, DmtxVector2* distort_point);

//畸变点转投影点
void distort2projectPoint(const DmtxVector2 distort_point, DmtxVector2* project_point);

//投影点转畸变点
void project2distortPoint(const DmtxVector2 project_point, DmtxVector2* distort_point);

//畸变点转世界坐标(z = 0)
void distort2worldPoint(const DmtxVector2 distort_point, DmtxVector3* world_point);

//从去畸变点计算对应的世界坐标
void GetXYGivenZ(double u,double v,double Z,double* X,double* Y);

#ifdef __cplusplus
}
#endif

#endif 
