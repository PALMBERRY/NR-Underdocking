#include "project.h"
#include "DMA_configuration.h"

#include <stdio.h>

#pragma DATA_SECTION(up_proj_table, ".table");
#pragma DATA_ALIGN(up_proj_table, 8)
Inter_Linear_Table up_proj_table[UP_PROJ_IMG_SIZE_FULL*UP_PROJ_IMG_SIZE_FULL];			//投影表

#pragma DATA_SECTION(down_proj_table, ".table");
#pragma DATA_ALIGN(down_proj_table, 8)
Inter_Linear_Table down_proj_table[DOWN_PROJ_IMG_SIZE_FULL*DOWN_PROJ_IMG_SIZE_FULL];			//投影表

#pragma DATA_ALIGN(dm_proj_table, 8)
Inter_Linear_Table dm_proj_table[DOWN_PROJ_IMG_SIZE*DOWN_PROJ_IMG_SIZE];

#pragma DATA_ALIGN(dm_distort_img, 8)
uint8_t dm_distort_img[DM_DISTORT_IMG_SIZE*DM_DISTORT_IMG_SIZE];

float g_fx;
float g_fx_;
float g_fy;
float g_fy_;
float g_cx;
float g_cy;

float g_k1;
float g_k2;

//中心点
DmtxVector2 g_center_p;

//正方向斜率及其角度
double p_dir_angle;

//空间坐标偏移量
double g_coeffs_x;
double g_coeffs_y;

Camera_Param* cur_camera;

int LoadCamParam(char* pathname)
{
	FILE* in;
	char filename[100];
	int j;

	//intrinsic 3*3 内参矩阵
	sprintf(filename, pathname, "intrinsic.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		for (j=0; j < 9; ++j){
			fscanf(in, "%f", &cur_camera->inParam[j]);
		}
	}
	else  {
		printf("Invalid input intrinsic.txt file\n");
		return 0;
	}
	fclose(in);
	
	//4*1 畸变向量
	sprintf(filename, pathname, "distortion.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		for (j=0; j < 4; j++)
		{
			fscanf(in, "%f", &cur_camera->distortion[j]);
		}
	}
	else  {
		printf("Invalid input distortion.txt file\n");
		return 0;
	}
	fclose(in);

	//3*1 外参旋转向量
	sprintf(filename, pathname, "extrinsicR.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		for (j=0; j < 9; j++){
			fscanf(in, "%f", &cur_camera->exParamR[j]);
		}
		for (j=0; j < 9; j++){
			fscanf(in, "%f", &cur_camera->exParamR_[j]);
		}
	}
	else  {
		printf("Invalid input extrinsicR.txt file\n");
		return 0;
	}
	fclose(in);

	//3*1 外参平移向量
	sprintf(filename, pathname, "extrinsicT.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		for (j=0; j < 3; j++){
			fscanf(in, "%f", &cur_camera->exParamt[j]);
		}
	}
	else  {
		printf("Invalid input extrinsict.txt file\n");
		return 0;
	}
	fclose(in);

	//中心点
	sprintf(filename, pathname, "center_point.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		fscanf(in, "%lf", &cur_camera->center_point.X);
		fscanf(in, "%lf", &cur_camera->center_point.Y);
	}
	else  {
		printf("Invalid input center_point.txt file\n");
		return 0;
	}
	fclose(in);

	//正方向
	sprintf(filename, pathname, "positive_direction.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		fscanf(in, "%lf", &cur_camera->dir_angle);
	}
	else  {
		printf("Invalid input positive_direction.txt file\n");
		return 0;
	}
	fclose(in);

	return 1;
}

#ifdef WIN32
int LoadCamParam_calib(char* pathname)
{
	FILE* in;
	char filename[100];
	int j;

	//3*1 外参旋转向量
	sprintf(filename, pathname, "extrinsicR.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		for (j=0; j < 9; j++){
			fscanf(in, "%f", &cur_camera->exParamR[j]);
		}
		for (j=0; j < 9; j++){
			fscanf(in, "%f", &cur_camera->exParamR_[j]);
		}
	}
	else  {
		printf("Invalid input extrinsicR.txt file\n");
		return 0;
	}
	fclose(in);

	//3*1 外参平移向量
	sprintf(filename, pathname, "extrinsicT.txt");
	in = fopen(filename, "r");
	if(in != NULL)  {
		for (j=0; j < 3; j++){
			fscanf(in, "%f", &cur_camera->exParamt[j]);
		}
	}
	else  {
		printf("Invalid input extrinsict.txt file\n");
		return 0;
	}
	fclose(in);

	return 1;
}
#endif

int g_dm_proj_img_size;
int g_dm_proj_img_size_2;
Inter_Linear_Table* g_proj_table;
double g_object_size;
double g_proj_ratio;
double g_proj_ratio_;

int SetCamParam()
{
	g_fx = cur_camera->inParam[0]; g_fy = cur_camera->inParam[4];
	g_fx_ = 1.0/g_fx;  g_fy_ = 1.0/g_fy;
	g_cx = cur_camera->inParam[2]; g_cy = cur_camera->inParam[5];

	//4*1 畸变向量

	g_k1 = cur_camera->distortion[0], g_k2 = cur_camera->distortion[1];

	//中心点
	g_center_p = cur_camera->center_point;

	//正方向
	p_dir_angle = cur_camera->dir_angle;//-1.55891;

	g_coeffs_x = cur_camera->coeffs_x;
	g_coeffs_y = cur_camera->coeffs_y;

	if(cur_camera->camera_id == 0)
	{
		g_object_size = DOWN_OBJECT_SIZE;
		g_proj_ratio = g_object_size/DOWN_PROJ_IMG_SIZE_FULL;

		g_dm_proj_img_size = DOWN_PROJ_IMG_SIZE;
		g_dm_proj_img_size_2 = g_dm_proj_img_size>>1;
	
		g_proj_table = down_proj_table;
	}
	else
	{
		g_object_size = UP_OBJECT_SIZE;
		g_proj_ratio = g_object_size/UP_PROJ_IMG_SIZE_FULL;

		g_dm_proj_img_size = UP_PROJ_IMG_SIZE;
		g_dm_proj_img_size_2 = g_dm_proj_img_size>>1;

		g_proj_table = up_proj_table;
	}

	g_proj_ratio_ = 1.0/g_proj_ratio;

	return 1;
}

void calcObjCoeffs()
{
	DmtxVector3 obj_lt, obj_lb, obj_rt, obj_rb;
	DmtxVector2 src_point, obj_center;

	//将四个顶点从畸变图像坐标系转换到为世界坐标（mm）
	src_point.X = 0;
	src_point.Y = 0;
	distort2worldPoint(src_point, &obj_lt);
	src_point.Y = SRC_IMG_Y-1;
	distort2worldPoint(src_point, &obj_lb);
	src_point.X = SRC_IMG_X-1;
	distort2worldPoint(src_point, &obj_rb);
	src_point.Y = 0;
	distort2worldPoint(src_point, &obj_rt);

	//计算空间坐标的中心点
	obj_center.X = (MIN(obj_lt.X, MIN(obj_lb.X, MIN(obj_rt.X, obj_rb.X)))+MAX(obj_lt.X, MAX(obj_lb.X, MAX(obj_rt.X, obj_rb.X))))*0.5;
	obj_center.Y = (MIN(obj_lt.Y, MIN(obj_lb.Y, MIN(obj_rt.Y, obj_rb.Y)))+MAX(obj_lt.Y, MAX(obj_lb.Y, MAX(obj_rt.Y, obj_rb.Y))))*0.5;

	//中心点的偏移量(世界坐标平移系数）
	if(cur_camera->camera_id == 0)
	{
		cur_camera->coeffs_x = g_coeffs_x = obj_center.X-DOWN_OBJECT_SIZE*0.5;	
		cur_camera->coeffs_y = g_coeffs_y =  obj_center.Y-DOWN_OBJECT_SIZE*0.5;
	}
	else
	{
		cur_camera->coeffs_x = g_coeffs_x = obj_center.X-UP_OBJECT_SIZE*0.5;	
		cur_camera->coeffs_y = g_coeffs_y =  obj_center.Y-UP_OBJECT_SIZE*0.5;
	}
}

void GeneratePrjTable()
{
	DmtxVector2 proj_point, distort_point;
	int i, j, proj_size;
	uint16_t x_, y_;
	double x0, y0, a, b;
	int camera_id;
	Inter_Linear_Table* proj_table;

	camera_id = cur_camera->camera_id;

#ifdef CALC_DM_DISTORT_IMG_SIZE
	int k, l;
	uint16_t x_min, x_max, y_min, y_max, tmp;
	int16_t tmp1, tmp2, x_range = 0, y_range = 0;
#endif

	if(camera_id == 0)
	{
		proj_size = DOWN_PROJ_IMG_SIZE_FULL;
		proj_table = down_proj_table;
	}
	else
	{
		proj_size = UP_PROJ_IMG_SIZE_FULL;
		proj_table = up_proj_table;
	}

	//生成投影表格,i是列,j是行
	for (i = 0;i < proj_size; i++)
	{
		for (j = 0;j < proj_size; j++)
		{
			proj_point.X = i;
			proj_point.Y = j;
			project2distortPoint(proj_point, &distort_point);

			x0 = MIN(MAX(distort_point.X, 0.0), SRC_IMG_X-1.01);
			y0 = MIN(MAX(distort_point.Y, 0.0), SRC_IMG_Y-1.01);

			x_ = (uint16_t)x0;
			y_ = (uint16_t)y0;

			b = x0-x_;			//b	
			a = y0-y_;

			proj_table[j*proj_size+i].x = x_;
			proj_table[j*proj_size+i].y = y_;
			proj_table[j*proj_size+i].a = (uint16_t)(a*65536);
			proj_table[j*proj_size+i].b = (uint16_t)(b*65536);
		}
	}


#ifdef CALC_DM_DISTORT_IMG_SIZE
	//根据当前投影表的尺寸，计算畸变图所需的大小
	for (i = 0;i < PROJ_IMG_Y-DM_PROJ_IMG_SIZE+1; i++)
	{
		for (j = 0;j < PROJ_IMG_X-DM_PROJ_IMG_SIZE+1; j++)
		{
			x_min = 9999, x_max = 0;
			y_min = 9999, y_max = 0;
			for(k = 0;k < DM_PROJ_IMG_SIZE;k++)
			{
				for(l = 0;l < DM_PROJ_IMG_SIZE;l++)
				{
					tmp = proj_table[(i+k)*PROJ_IMG_X+j+l].x;
					if(tmp > x_max)
						x_max = tmp;
					if(tmp < x_min)
						x_min = tmp;

					tmp = proj_table[(i+k)*PROJ_IMG_X+j+l].y;
					if(tmp > y_max)
						y_max = tmp;
					if(tmp < y_min)
						y_min = tmp;
				}
			}
			tmp1 = abs(x_min-proj_table[(i+DM_PROJ_IMG_SIZE/2)*PROJ_IMG_X+j+DM_PROJ_IMG_SIZE/2].x);
			tmp2 = abs(x_max-proj_table[(i+DM_PROJ_IMG_SIZE/2)*PROJ_IMG_X+j+DM_PROJ_IMG_SIZE/2].x);
			tmp = max(tmp1, tmp2);
			if(tmp > x_range)
				x_range = tmp;

			tmp1 = abs(y_min-proj_table[(i+DM_PROJ_IMG_SIZE/2)*PROJ_IMG_X+j+DM_PROJ_IMG_SIZE/2].y);
			tmp2 = abs(y_max-proj_table[(i+DM_PROJ_IMG_SIZE/2)*PROJ_IMG_X+j+DM_PROJ_IMG_SIZE/2].y);
			tmp = max(tmp1, tmp2);
			if(tmp > y_range)
			{
				y_range = tmp;
				//cout<<i<<'\t'<<j<<'\t'<<y_range<<endl;
			}
		}
	}
	x_range++;
	y_range++;

	printf("%d\t%d\n", (x_range+1)*2, (y_range+1)*2);
#endif
}

#ifdef WIN32
void SaveTable()
{
	FILE* out;
	//stringstream ss;
	char one_line[20], filename[100];
	uint8_t hex_mem[4];
	uint8_t* table;
	int i;

	/*sprintf(filename, TABLE_PATH, "src2undistort_table.dat");
	out = fopen(filename, "w");
	table = (byte*)src2undistort_table;
	fputs("1651 1 C0070800 0 70800\n", out);
	for (i = 0;i < SRC_IMG_X*SRC_IMG_Y*2;i++)
	{
		hex_mem[0] = *table;
		table++;
		hex_mem[1] = *table;
		table++;
		hex_mem[2] = *table;
		table++;
		hex_mem[3] = *table;
		table++;
		sprintf(one_line, "0x%.2x%.2x%.2x%.2x\n", hex_mem[3], hex_mem[2], hex_mem[1], hex_mem[0]); 
		fputs(one_line, out);
	}
	fclose(out);*/

	sprintf(filename, TABLE_PATH, "proj_table.dat");
	out = fopen(filename, "w");
	table = (uint8_t*)g_proj_table;
	fputs("1651 1 C0000000 0 3C800\n", out);
	for (i = 0;i < g_dm_proj_img_size*g_dm_proj_img_size*2;i++)
	{
		hex_mem[0] = *table;
		table++;
		hex_mem[1] = *table;
		table++;
		hex_mem[2] = *table;
		table++;
		hex_mem[3] = *table;
		table++;
		sprintf(one_line, "0x%.2x%.2x%.2x%.2x\n", hex_mem[3], hex_mem[2], hex_mem[1], hex_mem[0]); 
		fputs(one_line, out);
	}
	fclose(out);
}
#endif

void Project2_one_step(const uint8_t* srcImg, uint8_t* restrict dstImg)
{
	int i, src_point;
	uint16_t x, y;
	uint16_t a, b, a_, b_, factor1, factor2;
	uint8_t g00, g01, g10, g11;
	Inter_Linear_Table* current_table_point = g_proj_table;

	_nassert((int)(dstImg)%8 == 0);
#pragma MUST_ITERATE(, , 8)
	for (i = 0;i < g_dm_proj_img_size*g_dm_proj_img_size;i++)
	{
		x = current_table_point->x;
		y = current_table_point->y;
		a = current_table_point->a;
		b = current_table_point->b;
		current_table_point++;

		//g_point[i] = y*SRC_IMG_Y+x;
		src_point = y*SRC_IMG_X+x;
		g00 = srcImg[src_point++];
		g01 = srcImg[src_point];
		src_point += SRC_IMG_X;
		g11 = srcImg[src_point--];
		g10 = srcImg[src_point];

		a_ = 0xFFFF-a;
		b_ = 0xFFFF-b;

		factor1 = (a*g11+a_*g01)>>8;
		factor2 = (a*g10+a_*g00)>>8;

		dstImg[i] = (b*factor1+b_*factor2)>>24;

		//result_img[dst_point++] = (uint8_t)(b*(a*g11+(1-a)*g01)+(1-b)*(a*g10+(1-a)*g00));
	}
}

//仅适用于DOWN
void Project3_one_step(const uint8_t* srcImg, uint8_t* restrict dstImg, DmtxVector2 ori_loc, DmtxPixelLoc* start_point_project)
{
	int i, j, src_point;
	int16_t x, y;
	uint16_t a, b, a_, b_, factor1, factor2;
	uint8_t g00, g01, g10, g11;
	Inter_Linear_Table* current_table_point;
	DmtxVector2 point_project;

	uint8_t* current_point;

	double tmp_x, tmp_y;
	
	//根据畸变点计算去畸变点
	//distort2undistortPoint(ori_loc, &point_undistort);

	//根据去畸变点计算投影点
	//undistort2projectPoint(point_undistort, &point_project);	

	distort2projectPoint(ori_loc, &point_project);

	tmp_x = point_project.X;
	tmp_y = point_project.Y;

	if(tmp_x > DOWN_PROJ_IMG_SIZE_2)
	{
		if(tmp_x > DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2)
		{
			point_project.X = DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2;
		}
	}
	else
	{
		point_project.X = DOWN_PROJ_IMG_SIZE_2;
	}

	if(tmp_y > DOWN_PROJ_IMG_SIZE_2)
	{
		if(tmp_y > DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2)
		{
			point_project.Y = DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2;
		}
	}
	else
	{
		point_project.Y = DOWN_PROJ_IMG_SIZE_2;
	}

	start_point_project->X = _dpint(point_project.X)-DOWN_PROJ_IMG_SIZE_2;
	start_point_project->Y = _dpint(point_project.Y)-DOWN_PROJ_IMG_SIZE_2;

	//投影变换
	current_table_point = &down_proj_table[start_point_project->Y*DOWN_PROJ_IMG_SIZE_FULL+start_point_project->X];
	current_point = dstImg;
//#pragma UNROLL(8)
	for (i = 0;i < DOWN_PROJ_IMG_SIZE;i++)
	{
		for(j = 0;j < DOWN_PROJ_IMG_SIZE;j++)
		{
			/*tmp = _amem8(src2undistort_table+i);
			x = tmp&0x00000000FFFF0000 >> 48;
			y = tmp&0x00000000FFFF0000 >> 32;
			a = tmp&0x00000000FFFF0000 >> 16;
			b = tmp&0x000000000000FFFF;*/

			/*x = src2undistort_x[i];
			y = src2undistort_y[i];
			a = src2undistort_a[i];
			b = src2undistort_b[i];*/

			x = current_table_point->x;
			y = current_table_point->y;
			a = current_table_point->a;
			b = current_table_point->b;
			current_table_point++;

			assert(x >= 0 && x < SRC_IMG_X-1);
			assert(y >= 0 && y < SRC_IMG_Y-1);

			//g_point[i] = y*SRC_IMG_Y+x;
			src_point = y*SRC_IMG_X+x;
			g00 = srcImg[src_point++];
			g01 = srcImg[src_point];
			src_point += SRC_IMG_X;
			g11 = srcImg[src_point--];
			g10 = srcImg[src_point];

			a_ = 0xFFFF-a;
			b_ = 0xFFFF-b;

			factor1 = (a*g11+a_*g01)>>8;
			factor2 = (a*g10+a_*g00)>>8;
			*current_point = (b*factor1+b_*factor2)>>24;
			current_point++;

			//dstImg[i] = (uint8_t)(b*(a*g11+(1-a)*g01)+(1-b)*(a*g10+(1-a)*g00));
		}
		current_table_point += DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE;
	}

	//imshow("result_small_", result_small_);
	//cvWaitKey(2);
}

//仅适用于DOWN
void Project3_one_step_DSP(const uint8_t* srcImg, uint8_t* restrict dstImg, DmtxVector2 ori_loc, DmtxPixelLoc* start_point_project)
{
	int i, src_point;
	int16_t x, y;
	uint16_t a, b, a_, b_, factor1, factor2;
	uint8_t g00, g01, g10, g11;
	Inter_Linear_Table *current_table_point;
	DmtxVector2 project_point, distort_point;
	DmtxPixelLoc start_point_distort;

	//uint8_t* current_point;
	const uint8_t *cur_src_point;

#ifndef USE_DMA
	Inter_Linear_Table* dm_table_point;
	uint8_t* dm_point;
	int j;
#endif

	double tmp_x, tmp_y;
	
	//根据畸变点计算去畸变点
	//distort2undistortPoint(ori_loc, &point_undistort);

	//根据去畸变点计算投影点
	//undistort2projectPoint(point_undistort, &point_project);	

	distort_point = ori_loc;
	distort2projectPoint(distort_point, &project_point);

	tmp_x = project_point.X;
	tmp_y = project_point.Y;

	if(tmp_x > DOWN_PROJ_IMG_SIZE_2)
	{
		if(tmp_x > DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2)
			project_point.X = DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2;
	}
	else
		project_point.X = DOWN_PROJ_IMG_SIZE_2;

	if(tmp_y > DOWN_PROJ_IMG_SIZE_2)
	{
		if(tmp_y > DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2)
			project_point.Y = DOWN_PROJ_IMG_SIZE_FULL-DOWN_PROJ_IMG_SIZE_2;
	}
	else
		project_point.Y = DOWN_PROJ_IMG_SIZE_2;

	if(tmp_x != project_point.X || tmp_y != project_point.Y)
	{
		project2distortPoint(project_point, &distort_point);
	}

	start_point_project->X = _dpint(project_point.X)-DOWN_PROJ_IMG_SIZE_2;
	start_point_project->Y = _dpint(project_point.Y)-DOWN_PROJ_IMG_SIZE_2;

	//tic();

	//拷贝所需的投影表
	current_table_point = &down_proj_table[start_point_project->Y*DOWN_PROJ_IMG_SIZE_FULL+start_point_project->X];

#ifdef USE_DMA
	configureEDMA3CC(g_PaRAM3, 3, (uint8_t*)current_table_point, (uint8_t*)dm_proj_table, DOWN_PROJ_IMG_SIZE*8, DOWN_PROJ_IMG_SIZE, DOWN_PROJ_IMG_SIZE_FULL*8, DOWN_PROJ_IMG_SIZE*8, 0, 0, 1, 0xFFFF);
	startEDMA3(3);
#else
	dm_table_point = dm_proj_table;
	for(i = 0;i < DOWN_PROJ_IMG_SIZE;i++)
	{
		for(j = 0;j < DOWN_PROJ_IMG_SIZE;j++)
		{
			dm_table_point[j] = current_table_point[j];
		}
		current_table_point += DOWN_PROJ_IMG_SIZE_FULL;
		dm_table_point += DOWN_PROJ_IMG_SIZE;
	}
#endif
	//toc("");

	tmp_x = distort_point.X;
	tmp_y = distort_point.Y;

	if(tmp_x > DM_DISTORT_IMG_SIZE_2)
	{
		if(tmp_x > SRC_IMG_X-DM_DISTORT_IMG_SIZE_2)
			distort_point.X = SRC_IMG_X-DM_DISTORT_IMG_SIZE_2;
	}
	else
		distort_point.X = DM_DISTORT_IMG_SIZE_2;

	if(tmp_y > DM_DISTORT_IMG_SIZE_2)
	{
		if(tmp_y > SRC_IMG_Y-DM_DISTORT_IMG_SIZE_2)
			distort_point.Y = SRC_IMG_Y-DM_DISTORT_IMG_SIZE_2;
	}
	else
		distort_point.Y = DM_DISTORT_IMG_SIZE_2;

	start_point_distort.X = _dpint(distort_point.X)-DM_DISTORT_IMG_SIZE_2;
	start_point_distort.Y = _dpint(distort_point.Y)-DM_DISTORT_IMG_SIZE_2;

	//tic();
	//拷贝所需的畸变图
	cur_src_point = &srcImg[start_point_distort.Y*SRC_IMG_X+start_point_distort.X];

#ifdef USE_DMA
	waitForEDMA3(3);

	configureEDMA3CC(g_PaRAM3, 3, cur_src_point, dm_distort_img, DM_DISTORT_IMG_SIZE, DM_DISTORT_IMG_SIZE, SRC_IMG_X, DM_DISTORT_IMG_SIZE, 0, 0, 1, 0xFFFF);
	startEDMA3(3);

	waitForEDMA3(3);
#else
	dm_point = dm_distort_img;
	for(i = 0;i < DM_DISTORT_IMG_SIZE;i++)
	{
		for(j = 0;j < DM_DISTORT_IMG_SIZE;j++)
		{
			dm_point[j] = cur_src_point[j];
		}
		cur_src_point += SRC_IMG_X;
		dm_point += DM_DISTORT_IMG_SIZE;
	}
#endif
	//toc("");

	//投影变换
	current_table_point = dm_proj_table;
	//current_point = dstImg;

	_nassert((int)(dstImg)%8 == 0);
	for (i = 0;i < DOWN_PROJ_IMG_SIZE*DOWN_PROJ_IMG_SIZE;i++)
	{
		/*tmp = _amem8(src2undistort_table+i);
		x = tmp&0x00000000FFFF0000 >> 48;
		y = tmp&0x00000000FFFF0000 >> 32;
		a = tmp&0x00000000FFFF0000 >> 16;
		b = tmp&0x000000000000FFFF;*/

		/*x = src2undistort_x[i];
		y = src2undistort_y[i];
		a = src2undistort_a[i];
		b = src2undistort_b[i];*/

		x = current_table_point->x - start_point_distort.X;
		y = current_table_point->y - start_point_distort.Y;
		a = current_table_point->a;
		b = current_table_point->b;
		current_table_point++;

		assert(x >= 0 && x < DM_DISTORT_IMG_SIZE-1);
		assert(y >= 0 && y < DM_DISTORT_IMG_SIZE-1);

		//g_point[i] = y*SRC_IMG_Y+x;
		src_point = y*DM_DISTORT_IMG_SIZE+x;
		g00 = dm_distort_img[src_point++];
		g01 = dm_distort_img[src_point];
		src_point += DM_DISTORT_IMG_SIZE;
		g11 = dm_distort_img[src_point--];
		g10 = dm_distort_img[src_point];

		a_ = 0xFFFF-a;
		b_ = 0xFFFF-b;

		factor1 = (a*g11+a_*g01)>>8;
		factor2 = (a*g10+a_*g00)>>8;
		dstImg[i] = (b*factor1+b_*factor2)>>24;
		//current_point++;

		//dstImg[i] = (uint8_t)(b*(a*g11+(1-a)*g01)+(1-b)*(a*g10+(1-a)*g00));
	}

	//imshow("result_small_", result_small_);
	//cvWaitKey(2);
}

//畸变点转去畸变点
void distort2undistortPoint(const DmtxVector2 distort_point, DmtxVector2* undistort_point)
{
	int i;

	double u = distort_point.X;
	double v = distort_point.Y;

	double y0 = (v - g_cy)*g_fy_; 
	double x0 = (u - g_cx)*g_fx_;

	double x = x0;
	double y = y0;

	double r2, icdist;

	for( i = 0; i < 5; i++ )
    {
        r2 = x*x + y*y;
        icdist = 1*_rcpdp(1 + (g_k2*r2 + g_k1)*r2);
        x = x0*icdist;
        y = y0*icdist;
    }

	undistort_point->X = x*g_fx+g_cx;
	undistort_point->Y = y*g_fy+g_cy;

	//计算有误差，用原始数据
	/*float dx = image_point_dst.x - image_point_src.x;
	float dy = image_point_dst.y - image_point_src.y;
	if (dx*dx + dy*dy >60)
	{
		image_point_dst = image_point_src;
	}*/
}

//去畸变点转投影点
void undistort2projectPoint(const DmtxVector2 undistort_point, DmtxVector2* project_point)
{
	double u = undistort_point.X;
	double v = undistort_point.Y;
	double X = 0;
	double Y = 0;

	GetXYGivenZ(u, v, 0, &X, &Y);

	project_point->X = (X-g_coeffs_x)*g_proj_ratio_;
	project_point->Y = (Y-g_coeffs_y)*g_proj_ratio_;
}

//投影点转去畸变点
void project2undistortPoint(const DmtxVector2 project_point, DmtxVector2* undistort_point)
{
	DmtxVector3 obj_point;
	double X, Y, Z, x, y, z;

	//投影点转去畸变点
	obj_point.X = g_proj_ratio*project_point.X+g_coeffs_x;		
	obj_point.Y = g_proj_ratio*project_point.Y+g_coeffs_y;
	obj_point.Z = 0;

	X = obj_point.X, Y = obj_point.Y, Z = obj_point.Z;
	x = cur_camera->exParamR[0]*X + cur_camera->exParamR[1]*Y + cur_camera->exParamR[2]*Z + cur_camera->exParamt[0];
	y = cur_camera->exParamR[3]*X + cur_camera->exParamR[4]*Y + cur_camera->exParamR[5]*Z + cur_camera->exParamt[1];
	z = cur_camera->exParamR[6]*X + cur_camera->exParamR[7]*Y + cur_camera->exParamR[8]*Z + cur_camera->exParamt[2];

	z = z ? _rcpdp(z) : 1.;
	x *= z; y *= z;

	undistort_point->X = x*g_fx+g_cx;
	undistort_point->Y = y*g_fy+g_cy;
}

//去畸变点转畸变点
void undistort2distortPoint(const DmtxVector2 undistort_point, DmtxVector2* distort_point)
{
	double x, y, r2, r4, xd, yd, cdist;

	//去畸变点转畸变点
	x = (undistort_point.X-g_cx)*g_fx_;
	y = (undistort_point.Y-g_cy)*g_fy_;

	//去畸变点转畸变点
	r2 = x*x+y*y;
	r4 = r2*r2;

	cdist = 1 + g_k1*r2 + g_k2*r4;

	xd = x*cdist;
	yd = y*cdist;

	distort_point->X = xd*g_fx+g_cx;
	distort_point->Y = yd*g_fy+g_cy;
}

//畸变点转投影点
void distort2projectPoint(const DmtxVector2 distort_point, DmtxVector2* project_point)
{
	int i;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3, s;

	double u = distort_point.X;
	double v = distort_point.Y;

	double y0 = (v - g_cy)*g_fy_; 
	double x0 = (u - g_cx)*g_fx_;

	double x = x0;
	double y = y0;

	double r2, icdist;

	for( i = 0; i < 5; i++ )
    {
        r2 = x*x + y*y;
        icdist = 1*_rcpdp(1 + (g_k2*r2 + g_k1)*r2);
        x = x0*icdist;
        y = y0*icdist;
    }
	//x,y为去畸变点的摄像机坐标

	//计算该点的世界坐标
	r11 = cur_camera->exParamR_[0];
	r12 = cur_camera->exParamR_[1];
	r13 = cur_camera->exParamR_[2];
	r21 = cur_camera->exParamR_[3];
	r22 = cur_camera->exParamR_[4];
	r23 = cur_camera->exParamR_[5];
	r31 = cur_camera->exParamR_[6];
	r32 = cur_camera->exParamR_[7];
	r33 = cur_camera->exParamR_[8];

	t1 = cur_camera->exParamt[0];
	t2 = cur_camera->exParamt[1];
	t3 = cur_camera->exParamt[2];

	s = (r31*t1+r32*t2+r33*t3)*_rcpdp(r31*x+r32*y+r33);

	t1 = s*x-t1;
	t2 = s*y-t2;
	t3 = s-t3;

	x = r11*t1+r12*t2+r13*t3;
	y = r21*t1+r22*t2+r23*t3;
	//x,y为该点的世界坐标

	project_point->X = (x-g_coeffs_x)*g_proj_ratio_;
	project_point->Y = (y-g_coeffs_y)*g_proj_ratio_;
}

//投影点转畸变点
void project2distortPoint(const DmtxVector2 project_point, DmtxVector2* distort_point)
{
	DmtxVector3 obj_point;
	double X, Y, Z, x, y, z;
	double r2, r4, xd, yd, cdist;

	//投影点转去畸变点
	obj_point.X = g_proj_ratio*project_point.X+g_coeffs_x;		
	obj_point.Y = g_proj_ratio*project_point.Y+g_coeffs_y;
	obj_point.Z = 0;

	X = obj_point.X, Y = obj_point.Y, Z = obj_point.Z;
	x = cur_camera->exParamR[0]*X + cur_camera->exParamR[1]*Y + cur_camera->exParamR[2]*Z + cur_camera->exParamt[0];
	y = cur_camera->exParamR[3]*X + cur_camera->exParamR[4]*Y + cur_camera->exParamR[5]*Z + cur_camera->exParamt[1];
	z = cur_camera->exParamR[6]*X + cur_camera->exParamR[7]*Y + cur_camera->exParamR[8]*Z + cur_camera->exParamt[2];

	z = z ? _rcpdp(z) : 1.;
	x *= z; y *= z;
	//x,y为去畸变点的摄像机坐标

	//去畸变点转畸变点
	r2 = x*x+y*y;
	r4 = r2*r2;

	cdist = 1 + g_k1*r2 + g_k2*r4;

	xd = x*cdist;
	yd = y*cdist;

	distort_point->X = xd*g_fx+g_cx;
	distort_point->Y = yd*g_fy+g_cy;
}

//畸变点转世界坐标(z = 0)
void distort2worldPoint(const DmtxVector2 distort_point, DmtxVector3* world_point)
{
	int i;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3, s;

	double u = distort_point.X;
	double v = distort_point.Y;

	double y0 = (v - g_cy)*g_fy_; 
	double x0 = (u - g_cx)*g_fx_;

	double x = x0;
	double y = y0;

	double r2, icdist;

	for( i = 0; i < 5; i++ )
    {
        r2 = x*x + y*y;
        icdist = _rcpdp(1 + (g_k2*r2 + g_k1)*r2);
        x = x0*icdist;
        y = y0*icdist;
    }
	//x,y为去畸变点的摄像机坐标

	//计算该点的世界坐标
	r11 = cur_camera->exParamR_[0];
	r12 = cur_camera->exParamR_[1];
	r13 = cur_camera->exParamR_[2];
	r21 = cur_camera->exParamR_[3];
	r22 = cur_camera->exParamR_[4];
	r23 = cur_camera->exParamR_[5];
	r31 = cur_camera->exParamR_[6];
	r32 = cur_camera->exParamR_[7];
	r33 = cur_camera->exParamR_[8];

	t1 = cur_camera->exParamt[0];
	t2 = cur_camera->exParamt[1];
	t3 = cur_camera->exParamt[2];

	s = (r31*t1+r32*t2+r33*t3)*_rcpdp(r31*x+r32*y+r33);

	t1 = s*x-t1;
	t2 = s*y-t2;
	t3 = s-t3;

	world_point->X = r11*t1+r12*t2+r13*t3;
	world_point->Y = r21*t1+r22*t2+r23*t3;
}

//从去畸变点计算对应的世界坐标
void GetXYGivenZ(double u, double v, double Z, double* X, double* Y)
{
	double x0 = (u-g_cx)*g_fx_;
	double y0 = (v-g_cy)*g_fy_;

	//Mat A=(Mat_<float>(3,1)<<x0,y0,1);

	double r11 = cur_camera->exParamR_[0];
	double r12 = cur_camera->exParamR_[1];
	double r13 = cur_camera->exParamR_[2];
	double r21 = cur_camera->exParamR_[3];
	double r22 = cur_camera->exParamR_[4];
	double r23 = cur_camera->exParamR_[5];
	double r31 = cur_camera->exParamR_[6];
	double r32 = cur_camera->exParamR_[7];
	double r33 = cur_camera->exParamR_[8];

	double t1 = cur_camera->exParamt[0];
	double t2 = cur_camera->exParamt[1];
	double t3 = cur_camera->exParamt[2];

	double s = (Z+r31*t1+r32*t2+r33*t3)*_rcpdp(r31*x0+r32*y0+r33);

	//Mat obj_3D(3,1,CV_32FC1);
	//obj_3D=invR*(s*A-translation_vector);

	t1 = s*x0-t1;
	t2 = s*y0-t2;
	t3 = s-t3;

	*X = r11*t1+r12*t2+r13*t3;
	*Y = r21*t1+r22*t2+r23*t3;

	//X=obj_3D.at<float>(0,0);
	//Y=obj_3D.at<float>(1,0);
}

