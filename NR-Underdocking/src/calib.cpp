#include "calib.h"
#include "config.h"

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

using namespace std;

//仅下方
void calib_line()
{
	Mat line;
	ifstream p_line;
	float x, y;
	float vx, vy, x0, y0, A, B, C, thres_dis, cur_dis, max_dis;
	int max_i;

	Mat img = Mat::zeros(DOWN_PROJ_IMG_SIZE_FULL, DOWN_PROJ_IMG_SIZE_FULL, CV_8UC1);	

	p_line.open(POINT_LINE_PATH);

	//把点集中的点插入到向量中  
	std::vector<Point2f> points;  
	
	while(1)
	{
		p_line>>x;
		p_line>>y;
		if(p_line.eof())
			break;
		points.push_back(Point2f(x,y)); 
	}
	if(points.size() < 10)
	{
		//cout<<"参考点太少了！"<<endl;

		putText(img, "Too little points!", Point(20, 20), 1, 1, Scalar(255, 0, 255), 1);
		putText(img, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(255, 0, 255), 1);

		imshow("line", img);
		waitKey(0);

		return;
	}

	for(int i = 0;i < points.size();i++)
	{
		img.data[(int)(points.at(i).y)*DOWN_PROJ_IMG_SIZE_FULL+(int)(points.at(i).x)] = 128;
	}

	while(points.size() >= 10)
	{
		fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);
		vx = line.at<float>(0, 0);
		vy = line.at<float>(1, 0);
		x0 = line.at<float>(2, 0);
		y0 = line.at<float>(3, 0);

		A = vy;
		B = -vx;
		C = vx*y0-vy*x0;

		thres_dis = sqrt(A*A+B*B);
		max_i = 0;
		max_dis = 0;
		for(int i = 0;i < points.size();i++)
		{
			cur_dis = abs(A*points.at(i).x+B*points.at(i).y+C);
			if(cur_dis > max_dis)
			{
				max_dis = cur_dis;
				max_i = i;
			}

			//img.data[(int)(points.at(i).y)*PROJ_IMG_X+(int)(points.at(i).x)] = 128;
		}
		if(max_dis > thres_dis)
			points.erase(points.begin()+max_i);
		else
			break;
	}

	if(points.size() < 10)
	{
		//cout<<"参考点太少了！"<<endl;

		putText(img, "Too much bad points!", Point(20, 20), 1, 1, Scalar(255, 0, 255), 1);
		putText(img, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(255, 0, 255), 1);

		imshow("line", img);
		waitKey(0);

		return;
	}

	float theta = atan2(vy, vx);
	float theta_t = atan2(points[0].y-points.back().y, points[0].x-points.back().x);
	if(abs(theta_t-theta) > PI/2)
	{
		if(theta > 0)
			theta -= PI;
		else
			theta += PI;
	}

	ofstream out;
	out.open(POINT_LINE_SAVE_PATH);
	out<<theta<<endl;
	out.close();

	Point p1, p2;

	p1.x = x0-y0*vx/vy;
	p1.y = 0;

	p2.x = x0+(DOWN_PROJ_IMG_SIZE_FULL-1-y0)*vx/vy;
	p2.y = DOWN_PROJ_IMG_SIZE_FULL - 1;

	cv::line(img, p1, p2, Scalar(255,0,0),1);

	putText(img, "Positive direction calibration success!", Point(20, 20), 1, 1, Scalar(255, 0, 255), 1);
	putText(img, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(255, 0, 255), 1);

	imshow("line", img);
	waitKey(0);

	//cout<<line.at<float>(0, 0)<<'\t'<<line.at<float>(1, 0)<<endl;
}

//仅下方
void calib_ellipse()
{
	Mat line;
	ifstream p_line;
	float x, y, x0, y0, max_dis, cur_dis;
	int max_i;

	Mat img = Mat::zeros(DOWN_PROJ_IMG_SIZE_FULL, DOWN_PROJ_IMG_SIZE_FULL, CV_8UC1);

	p_line.open(POINT_CIRCLE_PATH);

	//把点集中的点插入到向量中  
	std::vector<Point2f> points;  
	
	while(1)
	{
		p_line>>x;
		p_line>>y;
		if(p_line.eof())
			break;
		points.push_back(Point2f(x,y)); 
	}

	if(points.size() < 40)
	{
		putText(img, "Too little points!", Point(20, 20), 1, 1, Scalar(255, 0, 255), 1);
		putText(img, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(255, 0, 255), 1);

		imshow("ellipse", img);
		waitKey(0);
		return;
	}

	for(int i = 0;i < points.size();i++)
	{
		img.data[(int)(points.at(i).y)*DOWN_PROJ_IMG_SIZE_FULL+(int)(points.at(i).x)] = 128;
	}

	RotatedRect rec;
	float theta, a, b;
	float t, cur_x, cur_y, tmp_x, tmp_y;
	while(points.size() >= 40)
	{
		rec = fitEllipse(points);

		//ellipse(img, rec, Scalar(255,0,0),1);

		x0 = rec.center.x;
		y0 = rec.center.y;
		if(rec.angle > 90)
			rec.angle -= 180;
		theta = rec.angle/180*PI;
		b = rec.size.width/2;
		a = rec.size.height/2;

		max_i = 0;
		max_dis = 0;
		for(int i = 0;i < points.size();i++)
		{
			tmp_x = points.at(i).x;
			tmp_y = points.at(i).y;

			t = atan2(tmp_y-y0, tmp_x-x0);
			t = t-theta;

			cur_x = a*cos(t)*cos(theta)-b*sin(t)*sin(theta)+x0;
			cur_y = a*cos(t)*sin(theta)+b*sin(t)*cos(theta)+y0;

			cur_dis = (cur_x-tmp_x)*(cur_x-tmp_x)+(cur_y-tmp_y)*(cur_y-tmp_y);
			if(cur_dis > max_dis)
			{
				max_dis = cur_dis;
				max_i = i;
			}

			//img.data[(int)(points.at(i).y)*PROJ_IMG_X+(int)(points.at(i).x)] = 128;
		}
		if(max_dis > 4.0)
			points.erase(points.begin()+max_i);
		else
			break;
	}

	if(points.size() < 40)
	{
		putText(img, "Too much bad points!", Point(20, 20), 1, 1, Scalar(255, 0, 255), 1);
		putText(img, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(255, 0, 255), 1);

		imshow("ellipse", img);
		waitKey(0);
		return;
	}

	ofstream out;
	out.open(POINT_CIRCLE_SAVE_PATH);
	out<<rec.center.x<<'\t'<<rec.center.y<<endl;
	out.close();
	
	ellipse(img, rec, Scalar(255,0,0),1);

	char loc_str[100];
	float sum = rec.size.width/rec.size.height;
	sprintf(loc_str, "width:height = %.2f", sum);
	putText(img, loc_str, Point(20, 20), 1, 1, Scalar(255, 0, 255), 1);

	putText(img, "Press anykey to continue!", Point(20, 50), 1, 1, Scalar(255, 0, 255), 1);

	imshow("ellipse", img);
	waitKey(0);

	//cout<<line.at<float>(0, 0)<<'\t'<<line.at<float>(1, 0)<<endl;
}