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
	float inParam[9];			//����ڲ�
	float exParamR[9];			//��������ת����
	float exParamR_[9];			//��������ת�������
	float exParamt[3];			//������ƽ������
	float distortion[4];		//����������
	DmtxVector2 center_point;	//���ĵ�
	double dir_angle;			//������
	double coeffs_x;			//�ռ�����ƫ����
	double coeffs_y;			
}Camera_Param;

extern DmtxVector2 g_center_p;

//������б�ʼ���Ƕ�
extern double p_dir_angle;

extern int g_dm_proj_img_size;
extern int g_dm_proj_img_size_2;
extern Inter_Linear_Table* g_proj_table;
extern double g_object_size;
extern double g_proj_ratio;
extern double g_proj_ratio_;

extern Camera_Param* cur_camera;

//Ԥ����ռ�ƫ����
void calcObjCoeffs();

//Ԥ��������ͶӰͼ���
void GeneratePrjTable();

//���óߴ����
//@param �������
//void SetSizeParam(int i,...);

#ifdef WIN32
//��ͶӰͼ��񱣴�����߸�ʽ
//@param �ļ���
void SaveTable();
#endif


//���ļ���ȡ�������
//@param ����·����
int LoadCamParam(char* pathname);

#ifdef WIN32
int LoadCamParam_calib(char* pathname);
#endif

int SetCamParam();

//��һ��ͼ�����ͶӰ�任
//@param ��任��ͼ��

void Project2_one_step(const uint8_t* srcImg, uint8_t* restrict dstImg);

void Project3_one_step(const uint8_t* srcImg, uint8_t* restrict dstImg, DmtxVector2 ori_loc, DmtxPixelLoc* start_point_project);

/* ȥ������ͶӰ�任���õ���ά������ͶӰͼ��
* srcImg			����ͼ��
* dstImg			���ͼ��
* ori_loc			DM�����ĵ�ֶ�λ���
* start_point_project	��ά������ͶӰͼ������ͶӰͼ�е����λ��		
* ����ֵ			DmtxPass��ʾ�ɹ���DmtxFail��ʾʧ��*/
void Project3_one_step_DSP(const uint8_t* srcImg, uint8_t* restrict dstImg, DmtxVector2 ori_loc, DmtxPixelLoc* start_point_project);

//�����תȥ�����
void distort2undistortPoint( const DmtxVector2 distort_point, DmtxVector2* undistort_point);

//ȥ�����תͶӰ��
void undistort2projectPoint(const DmtxVector2 undistort_point, DmtxVector2* project_point);

//ͶӰ��תȥ�����
void project2undistortPoint(const DmtxVector2 project_point, DmtxVector2* undistort_point);

//ȥ�����ת�����
void undistort2distortPoint(const DmtxVector2 undistort_point, DmtxVector2* distort_point);

//�����תͶӰ��
void distort2projectPoint(const DmtxVector2 distort_point, DmtxVector2* project_point);

//ͶӰ��ת�����
void project2distortPoint(const DmtxVector2 project_point, DmtxVector2* distort_point);

//�����ת��������(z = 0)
void distort2worldPoint(const DmtxVector2 distort_point, DmtxVector3* world_point);

//��ȥ���������Ӧ����������
void GetXYGivenZ(double u,double v,double Z,double* X,double* Y);

#ifdef __cplusplus
}
#endif

#endif 
