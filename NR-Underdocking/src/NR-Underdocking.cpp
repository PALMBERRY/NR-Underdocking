#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include "common/utils.h"
#include "common/numeric.h"
#include "messages/MessageInteraction.h"
#include "messages/AGVMessage.h"
#include "chassis/chassis.h"
#include "io/param_reader.h"
#include "map/map_convert.h"
#include "map/traval_map.h"
#include "cvtools/cv_plot.h"
#include <common/logger.h>		// log4cpp
//#include <highgui.h>
#include <time.h>
#include "math.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "laser2line.h"
#define INTSTD
#include "DataMatrix.h"
using namespace NJRobot;

// init log
NJRobot::Logger* Docking_logger = new NJRobot::Logger("DOCK");

//some define
#define Center_Sick			0.34
bool PLAN = false;		// true: plan_laser false: plan_camera

// line detection
boost::mutex g_mutex_laser;
Laser2Line roller;
LaserScan Laserdata;
edge EdgeX, EdgeY;
edge SafeX, SafeY;
bool stop_flag = false;
std::vector<segments>	roller_line;

// docker flags
bool Docking_Enable = false;
int	 direction_flag = 0;
bool pause_flag = false;
bool enter_flag = false;
bool leave_flag = false;
bool back_flag  = false;
bool nfind_flag = false;
bool filter_flag = false;
double side_flag;
int filter_num = 0;
int reach_flag = 0;
bool notre_flag;

// elevator flags
bool Elevator_Enable = false;

// docking para
double elevator_lenth;
double elevator_in_dist;
double elevator_out_dist;
double docking_in_dist;
double docking_out_dist;
double SafeDist;

// odometry
OrientedPoint curAccu_pose, accuPose, curAccu_odom, preAccu_odom, deltaDockPose;
OrientedPoint curLeave_pose, curLeave_odom, preLeave_odom, deltaPose;
boost::mutex g_mtx_odom;

// chose the roller line
segments key_line;
segments key_line2;
std::vector<MedFilter<double> > filter(3,MedFilter<double>(3));	



// some dec
bool get_the_line(void);


// set para
double get_para(double a, double b)
{
	if(a>b)	return sqrt(a-b);
	else return 0-sqrt(b-a);
}

//// 进出电梯导航
//void elevator_navigation()
//{
//	Mesg_RobotSpeed pub_speed;
//	Mesg_ControlTask feedbackInfo;
//	if(enter_flag)	// enter station
//	{
//		if(get_the_line())
//		{
//			nfind_flag = false;
//		//	hellocv.line(key_line.lbegin_x,key_line.lbegin_y,key_line.lend_x,key_line.lend_y,3,1);
//		//	printf("THE ROLLER LINE:Y=%lfX+%lf lenth:%lf ROBOT2LINE:%lf\n",key_line.line_k,key_line.line_b,key_line.line_length,
//		//		roller.point2line_dist(Point(0,0),Point(key_line.lbegin_x,key_line.lbegin_y),Point(key_line.lend_x,key_line.lend_y)));
//		}
//		else
//			nfind_flag = true;
//
//
//		Point robot_roller;
//		if(reach_flag == 0)
//		{
//			Point Target_point;
//			double HX_theta = atan(key_line.line_k);
//			double TA_dist	= roller.point2line_dist(Point(0,0),Point(key_line.lbegin_x,key_line.lbegin_y),Point(key_line.lend_x,key_line.lend_y));
//			if(key_line.lend_x>key_line.lbegin_x)	{Target_point.x = key_line.lend_x;	Target_point.y = key_line.lend_y;}
//			else									{Target_point.x = key_line.lbegin_x;Target_point.y = key_line.lbegin_y;}
//			
//			robot_roller.x = Target_point.x*cos(HX_theta)+Target_point.y*sin(HX_theta);
//			robot_roller.y = Target_point.y*cos(HX_theta)-Target_point.x*sin(HX_theta);
//
//			if(!filter_flag)
//			{
//				filter_num ++;
//				filter[0].input(robot_roller.x);
//				filter[1].input(TA_dist);
//				filter[2].input(HX_theta);
//			
//				if(filter_num == 3)
//				{
//					filter_num = 0;
//					filter_flag = true;
//					robot_roller.x = filter[0].output();
//					TA_dist		   = filter[1].output();
//					HX_theta	   = filter[2].output();
//				}
//			}
//			if(filter_flag)
//			{
//				filter_flag = false;
//				if(robot_roller.x<DockOffsetXside)
//				{
//					pub_speed.set_vx(0.07);	pub_speed.set_vy(0); pub_speed.set_w(0);
//
//					curAccu_pose.x = 0.0; curAccu_pose.y =0.0; curAccu_pose.theta=0.0;
//					preAccu_odom = curAccu_odom;
//					reach_flag = 1;
//					notre_flag = true;
//				}
//				else if(robot_roller.x<BackDist)
//				{
//				//	pub_speed.set_vx(0.07);	pub_speed.set_vy(0); pub_speed.set_w(side_flag*(DockOffsetY-TA_dist)*0.7+HX_theta*1.3);
//					pub_speed.set_vx(0.07);	pub_speed.set_vy(0); pub_speed.set_w(HX_theta*0.7);
//				}
//				else
//				{
//					pub_speed.set_vx(0.1);	pub_speed.set_vy(0); pub_speed.set_w(side_flag*get_para(DockOffsetY,TA_dist)*Rollerwpara+HX_theta*0.7);
//				}
//				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(pub_speed);
//
//				deltaPose = absoluteDifference(curAccu_odom, preAccu_odom);
//				curAccu_pose = absoluteSum(curAccu_pose,deltaPose);
//				preAccu_odom = curAccu_odom;
//				double dist_edge = curAccu_pose.x*curAccu_pose.x + curAccu_pose.y*curAccu_pose.y;
//				EdgeX.edge_large = 2.0 - sqrt(dist_edge);
//
//				printf("HX_theta:%lf Dist:%lf Pose:%lf,%lf Speed:%lf,%lf XEdge:%lf\n",HX_theta*180/M_PI,TA_dist,robot_roller.x,robot_roller.y,pub_speed.vx(),pub_speed.w(),EdgeX.edge_large);
//			}
//		}
//
//		if(reach_flag == 1)
//		{
//			deltaPose = absoluteDifference(curAccu_odom, preAccu_odom);
//			curAccu_pose = absoluteSum(curAccu_pose,deltaPose);
//			preAccu_odom = curAccu_odom;
//			double dist_safe = curAccu_pose.x*curAccu_pose.x + curAccu_pose.y*curAccu_pose.y;
//			if(dist_safe>SafeDist*SafeDist && notre_flag)
//			{
//				notre_flag = false;
//				feedbackInfo.set_flag(feedbackInfo.ECTF_ROLLER_ENTER_STATION);
//				feedbackInfo.set_result(2);
//				SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
//				printf("REACH!REACH!REACH!\n");
//			}
//			pub_speed.set_vx(0.07);	pub_speed.set_vy(0); pub_speed.set_w(0);
//			//pub_speed.set_vx(0);	pub_speed.set_vy(0); pub_speed.set_w(0);
//			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(pub_speed);
//		}
//
//		if(reach_flag == 2)
//		{
//			deltaPose = absoluteDifference(curAccu_odom, preAccu_odom);
//			curAccu_pose = absoluteSum(curAccu_pose,deltaPose);
//			preAccu_odom = curAccu_odom;
//			double dist_more = curAccu_pose.x*curAccu_pose.x + curAccu_pose.y*curAccu_pose.y;
//			if(dist_more>AdjDist*AdjDist)
//			{
//				pub_speed.set_vx(0);	pub_speed.set_vy(0); pub_speed.set_w(0);
//				feedbackInfo.set_flag(feedbackInfo.ECTF_ROLLER_ENTER_STATION);
//				feedbackInfo.set_result(0);
//				SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
//				enter_flag = false;
//				Docking_Enable = false;
//				printf("Enter Station Successfully!\n");
//			}
//			else
//			{
//				pub_speed.set_vx(0.03);	pub_speed.set_vy(0); pub_speed.set_w(0);
//			}
//			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(pub_speed);
//		}
//	}
//
//	if(leave_flag)
//	{
//		deltaPose = absoluteDifference(curLeave_odom, preLeave_odom);
//		curLeave_pose = absoluteSum(curLeave_pose,deltaPose);
//		preLeave_odom = curLeave_odom;
//		double leave_dist = curLeave_pose.x*curLeave_pose.x + curLeave_pose.y*curLeave_pose.y;
//		if(leave_dist>LeaveDist*LeaveDist)
//		{
//			pub_speed.set_vx(0);	pub_speed.set_vy(0); pub_speed.set_w(0);
//			feedbackInfo.set_flag(feedbackInfo.ECTF_ROLLER_LEAVE_STATION);
//			feedbackInfo.set_result(0);
//			SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
//			printf("Leave Station Successfully!\n");
//			leave_flag = false;
//			Docking_Enable = false;
//		}
//		else
//		{
//			pub_speed.set_vx(0.17);	pub_speed.set_vy(0); pub_speed.set_w(0);
//		}
//		SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(pub_speed);
//	}
//}




// 挑选roller线段
bool get_the_line(void)
{
	bool real_line_flag = false;
	for(int i=0; i<roller_line.size(); ++i)
	{
		if( roller_line[i].line_length>elevator_lenth-0.30 && roller_line[i].line_length<elevator_lenth+0.30 && 
			roller_line[i].line_k<0.5 && roller_line[i].line_k>-0.5 )
		{
			key_line = roller_line[i];
			real_line_flag = true;
		}
	}
	return real_line_flag;
}

// update laser
void updateLaser(Mesg_Laser ldata)
{ 
	if(g_mutex_laser.try_lock()) 	
	{
		if(Elevator_Enable || Docking_Enable)
		{
		CvPlot hellocv;
		hellocv.setResolution(0.003);
		Laserdata = transLaser(ldata);
		Laserdata = transformFrame(Laserdata,RobotPose(Center_Sick,0,0));
		
		stop_flag = false;
		for(int i=0; i<Laserdata.size(); ++i)
		{
			if( Laserdata[i].x>SafeX.edge_small && Laserdata[i].x<SafeX.edge_large &&
				Laserdata[i].y>SafeY.edge_small && Laserdata[i].y<SafeY.edge_large	)
				stop_flag = true;
		}

		for(int i=0; i<Laserdata.size(); ++i)
		{
			if( Laserdata[i].x<EdgeX.edge_small || Laserdata[i].x>EdgeX.edge_large ||
				Laserdata[i].y<EdgeY.edge_small || Laserdata[i].y>EdgeY.edge_large	)
			{	Laserdata.erase(Laserdata.begin()+i);	i --;	}
		}
		double laserp[1111] = {0,};		
		for(int i=0; i<Laserdata.size(); i++)
		{
			laserp[2*i]		= Laserdata[i].x;
			laserp[2*i+1]	= Laserdata[i].y;
		}	
	//	printf("laser data size:%d\n",Laserdata.size());
		hellocv.plot(laserp,Laserdata.size(),0,0);

		roller.setLaserData(Laserdata);
		roller.split_and_merge();
		roller_line = roller.get_line();
	//	printf("there are %d lines\n",roller_line.size());
		double linesp[3111] = {0,};
		for(int i=0; i<roller_line.size(); i++)
		{
			linesp[4*i]		= roller_line[i].lbegin_x;
			linesp[4*i+1]	= roller_line[i].lbegin_y;
			linesp[4*i+2]	= roller_line[i].lend_x;
			linesp[4*i+3]	= roller_line[i].lend_y;
		//	printf("the line %d:%d-%d lenth:%lf\n",i,roller_line[i].start,roller_line[i].end,roller_line[i].line_length);
			hellocv.line(linesp[4*i],linesp[4*i+1],linesp[4*i+2],linesp[4*i+3],2,2);
		}

		if(stop_flag)
		{
			Mesg_RobotSpeed stop_speed;
			stop_speed.set_vx(0);	stop_speed.set_vy(0);	stop_speed.set_w(0);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(stop_speed);
			printf("NI ZOU KAI!\n");
		}
		//else
		//	elevator_navigation();

		//if(get_the_line())
		//{
			//hellocv.line(key_line.lbegin_x,key_line.lbegin_y,key_line.lend_x,key_line.lend_y,3,1);
			//hellocv.line(key_line2.lbegin_x,key_line2.lbegin_y,key_line2.lend_x,key_line2.lend_y,3,1);
			//printf("THE ROLLER LINE:Y=%lfX+%lf lenth:%lf ROBOT2LINE:%lf HX_theta:%lf\n",key_line.line_k,key_line.line_b,key_line.line_length,
			//	roller.point2line_dist(Point(0,0),Point(key_line.lbegin_x,key_line.lbegin_y),Point(key_line.lend_x,key_line.lend_y)),
			//	atan(key_line.line_k));
		//}
		//else
		//	printf(":(\n");

		//hellocv.show(700,"HA");
		//cv::destroyWindow("HA");
		//hellocv.clear();
		}
		g_mutex_laser.unlock();
	}
}

// 里程计数据
void updateOdom(Mesg_RobotState state_odata)
{
	if(g_mtx_odom.try_lock())
	{
		curLeave_odom.x = state_odata.x();
		curLeave_odom.y = state_odata.y();
		curLeave_odom.theta = state_odata.theta();

		curAccu_odom.x = state_odata.x();
		curAccu_odom.y = state_odata.y();
		curAccu_odom.theta = state_odata.theta();
		g_mtx_odom.unlock();
	}
}

void updateMotionFlags(Mesg_NavigationTask Docking_motion)
{
	printf("Get Navi_task it's %d\n",Docking_motion.flag());
	switch(Docking_motion.flag())
	{
		case Docking_motion.UNDER_DOCK_INLEFT:	direction_flag = 1;leave_flag = false;	enter_flag = true;	Docking_Enable = true; Elevator_Enable = false; break;
		case Docking_motion.UNDER_DOCK_INRIGHT:	direction_flag =-1;leave_flag = false;	enter_flag = true;	Docking_Enable = true; Elevator_Enable = false; break;
		case Docking_motion.UNDER_DOCK_INFRONT:	direction_flag = 2;leave_flag = false;	enter_flag = true;	Docking_Enable = true; Elevator_Enable = false; break;
		case Docking_motion.UNDER_DOCK_INBACK:	direction_flag =-2;leave_flag = false;	enter_flag = true;	Docking_Enable = true; Elevator_Enable = false; break;
		case Docking_motion.UNDER_DOCK_OUT:		direction_flag = 0;leave_flag = true;	enter_flag = false;	Docking_Enable = true; Elevator_Enable = false; break;
	//	case Docking_motion.ELEVATOR_TASK:	leave_flag = false;	enter_flag = false;	Docking_Enable = false; Elevator_Enable = true; break;
		default: enter_flag = leave_flag = Docking_Enable = Elevator_Enable = false;	break;
	}

	Mesg_RobotSpeed test_speed;
	Mesg_ControlTask feedbackInfo;

	Mesg_CommonData Under_Docking_commondata;
	Mesg_DataUnit* unit_data = Under_Docking_commondata.add_datas();

	if(enter_flag && Docking_Enable)
	{
		// init odm
		curLeave_pose.x = 0.0; curLeave_pose.y = 0.0; curLeave_pose.theta = 0.0;
		preLeave_odom = curLeave_odom;

		SafeX.edge_small =	1.4;	SafeX.edge_large = 1.5;
		SafeY.edge_small = -0.5;	SafeY.edge_large = 0.5;
		reach_flag = 0;
		LOG_COUT_INFO(Docking_logger,"Get Docking_GO_IN Task Direction Number: "<<direction_flag);
		//if(Docking_motion.safedist() == 0)
		//{
			//reach_flag = 0;
			//printf("Get Docking_GO_IN task\n");
			//if(Docking_motion.isfinalpoint() == 0)
			//{
			//	side_flag = -1.0;	printf("UP! Track LEFT Side\n");
			//	EdgeX.edge_small = -0.5;	EdgeX.edge_large = 2.0;
			//	EdgeY.edge_small = -0.5;	EdgeY.edge_large = 1.0;
			//	SafeX.edge_small =	0.4;	SafeX.edge_large = 0.5;
			//	SafeY.edge_small = -0.4;	SafeY.edge_large = 0.1;
			//	DockOffsetY = DockOffsetYU;
			//}
			//if(Docking_motion.isfinalpoint() == 1)
			//{
			//	side_flag = 1.0;	printf("UP! Track RIGHT Side\n");
			//	EdgeX.edge_small = -0.5;	EdgeX.edge_large = 2.0;
			//	EdgeY.edge_small = -1.0;	EdgeY.edge_large = 0.5;
			//	SafeX.edge_small =	0.4;	SafeX.edge_large = 0.5;
			//	SafeY.edge_small = -0.1;	SafeY.edge_large = 0.4;
			//	DockOffsetY = DockOffsetYU;
			//}
			//if(Docking_motion.isfinalpoint() == 2)
			//{
			//	side_flag = -1.0;	printf("DOWN! Track LEFT Side\n");
			//	EdgeX.edge_small = -0.5;	EdgeX.edge_large = 2.0;
			//	EdgeY.edge_small = -0.5;	EdgeY.edge_large = 1.0;
			//	SafeX.edge_small =	0.4;	SafeX.edge_large = 0.5;
			//	SafeY.edge_small = -0.4;	SafeY.edge_large = 0.1;
			//	DockOffsetY = DockOffsetYD;
			//}
			//if(Docking_motion.isfinalpoint() == 3)
			//{
			//	side_flag = 1.0;	printf("DOWN! Track RIGHT Side\n");
			//	EdgeX.edge_small = -0.5;	EdgeX.edge_large = 2.0;
			//	EdgeY.edge_small = -1.0;	EdgeY.edge_large = 0.5;
			//	SafeX.edge_small =	0.4;	SafeX.edge_large = 0.5;
			//	SafeY.edge_small = -0.1;	SafeY.edge_large = 0.4;
			//	DockOffsetY = DockOffsetYD;
			//}

		//	curAccu_pose.x = 0.0; curAccu_pose.y =0.0; curAccu_pose.theta=0.0;
		//	preAccu_odom = curAccu_odom;
		//}
		//else
		//{
		//	reach_flag = 2;
		//	printf("get REACH task!\n");
		//	curAccu_pose.x = 0.0; curAccu_pose.y =0.0; curAccu_pose.theta=0.0;
		//	preAccu_odom = curAccu_odom;
		//}
		curAccu_pose.x = 0.0; curAccu_pose.y =0.0; curAccu_pose.theta=0.0;
		preAccu_odom = curAccu_odom;

		// docking

		std::cout<<"go!"<<endl;
		bool g_has_qr = false;
		bool tuning_flag = true;
		bool front_tuning = true;
		bool back_tuning = true;
		bool finish_flag = false;
		bool info_qr;
		double target_angle,init_angle;
		dmInitial(PICONLY);
		while(enter_flag)
		{
			info_qr = dmStart(NON_CALIB, PICONLY);
			if(info_qr && !g_has_qr)
			{
				if(fabs(fabs(result.delta_angle)-90)>20 && !g_has_qr && !pause_flag)		// 货架摆放偏差大于±20°
				{
					LOG_COUT_INFO(Docking_logger,"Over 20 degr! And paused!");
					unit_data->set_flag(Mesg_DataUnit::EDU_SHELF_ERRORS);
					//unit_data->add_values_int(1);
					SubPubManager::Instance()->m_commoninfo.GetPublisher()->publish(Under_Docking_commondata);

					test_speed.set_vx(0);	test_speed.set_vy(0); test_speed.set_w(0);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);	
					pause_flag = true;
				}
				
				if(fabs(fabs(result.delta_angle)-90)<=20)				// 货架摆放偏差不大于±20°
				{
					if(pause_flag)
					{
						pause_flag = false;
						unit_data->set_flag(Mesg_DataUnit::EDU_SHELF_RESTORE);
						//unit_data->add_values_int(0);
						SubPubManager::Instance()->m_commoninfo.GetPublisher()->publish(Under_Docking_commondata);
						LOG_COUT_INFO(Docking_logger,"Pause Restore!");

					}
					g_has_qr = true;
					if(fabs(result.delta_angle+90)<30)
					{
						init_angle = -90;
						if(direction_flag == 1)				// left
							target_angle = 180;
						else								// other
							target_angle = 0;
					}
					else
					{
						init_angle = 90;
						if(direction_flag == 1)				// left
							target_angle =  0;
						else								// other
							target_angle = 180;
					}
				}
			}
			if(!pause_flag)
			{
			if(!g_has_qr)		// 没有二维码就直走
			{
				deltaPose = absoluteDifference(curLeave_odom, preLeave_odom);
				curLeave_pose = absoluteSum(curLeave_pose,deltaPose);
				preLeave_odom = curLeave_odom;
				double leave_dist = curLeave_pose.x*curLeave_pose.x + curLeave_pose.y*curLeave_pose.y;
				if(leave_dist>docking_in_dist*docking_in_dist)
				{
					test_speed.set_vx(0);	test_speed.set_vy(0); test_speed.set_w(0);
					LOG_COUT_INFO(Docking_logger,"Not Find QR!");
					unit_data->set_flag(Mesg_DataUnit::EDU_SHELF_ERRORS);
					unit_data->add_values_int(1);
					SubPubManager::Instance()->m_commoninfo.GetPublisher()->publish(Under_Docking_commondata);
					pause_flag = true;

					//feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
					//feedbackInfo.set_result(1);
					//SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
					//enter_flag = false;
					//Docking_Enable = false;
				}
				else
				{
					test_speed.set_vx(0.07);	test_speed.set_vy(0); test_speed.set_w(0);
				}
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
			}
			else
			{
				if(info_qr)
				{
					std::cout<<"X:"<<result.delta_x<<" Y:"<<result.delta_y<<" Angle:"<<result.delta_angle<<endl;
					if(fabs(result.delta_y)<3)			// 判断到点
					{
						test_speed.set_vx(0);	test_speed.set_vy(0);	test_speed.set_w(0);
						SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
						printf("Arrive! Final Tuning...\n");
						while(tuning_flag)		// 旋转到目标角度
						{
							if(dmStart(NON_CALIB, PICONLY))
							{
								std::cout<<"X:"<<result.delta_x<<" Y:"<<result.delta_y<<" Angle:"<<result.delta_angle<<endl;
								test_speed.set_vx(0);	test_speed.set_vy(0);	
								if(target_angle == 0)
									test_speed.set_w((result.delta_angle)*0.007);
								else
								{
									double dgreee;
									if(result.delta_angle > 0)
										dgreee = result.delta_angle - 180;
									else
										dgreee = 180 + result.delta_angle;
									test_speed.set_w(dgreee*0.007);
								}
								SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
								if(fabs(fabs(result.delta_angle)-target_angle)<1.0)
								{
									tuning_flag = false;
									test_speed.set_vx(0);	test_speed.set_vy(0);	test_speed.set_w(0);
									SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
								}
							}						
						}
						while(!finish_flag)
						{
							if(dmStart(NON_CALIB, PICONLY))
							{
								if(fabs(result.delta_x)<3)
								{
									test_speed.set_vx(0);	test_speed.set_vy(0);	test_speed.set_w(0);
									SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
									if(direction_flag == 1 || direction_flag == -1)		// left or right
									{
										if(fabs(result.delta_x)<10 && fabs(result.delta_y)<10)
										{
											feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
											feedbackInfo.set_result(0);
											SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
											LOG_COUT_INFO(Docking_logger,"Under Docking Finish!");
										}
										else
										{
											feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
											feedbackInfo.set_result(1);
											SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
											LOG_COUT_INFO(Docking_logger,"Under Docking Field!");
										}
										enter_flag = false;
										finish_flag = true;
										Docking_Enable = false;
									}
									if((direction_flag == 2 && init_angle == 90) || (direction_flag == -2 && init_angle == -90))
									{
										while(front_tuning)		// 旋转到目标角度
										{
											if(dmStart(NON_CALIB, PICONLY))
											{
												std::cout<<"X:"<<result.delta_x<<" Y:"<<result.delta_y<<" Angle:"<<result.delta_angle<<endl;
												test_speed.set_vx(0);	test_speed.set_vy(0);													
												double dgreee;
												if(init_angle == 90)
												{
													if(result.delta_angle > 0)
														dgreee = result.delta_angle - 90;
													else
														dgreee = 270 + result.delta_angle;
												}
												if(init_angle == -90)
												{
													if(result.delta_angle > 0)
														dgreee = result.delta_angle - 90;
													else
														dgreee = result.delta_angle - 90;
												}
												test_speed.set_w(dgreee*0.007);												
												SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
												if(fabs(fabs(result.delta_angle)-90)<1.0)
												{
													front_tuning = false;
													test_speed.set_vx(0);	test_speed.set_vy(0);	test_speed.set_w(0);
													SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);

													if(fabs(result.delta_x)<10 && fabs(result.delta_y)<10)
													{
														feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
														feedbackInfo.set_result(0);
														SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
														LOG_COUT_INFO(Docking_logger,"Under Docking Finish!");
													}
													else
													{
														feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
														feedbackInfo.set_result(1);
														SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
														LOG_COUT_INFO(Docking_logger,"Under Docking Field!");
													}
													enter_flag = false;
													finish_flag = true;
													Docking_Enable = false;
												}
											}
										}
									}
									if((direction_flag == 2 && init_angle == -90) || (direction_flag == -2 && init_angle == 90))
									{
										while(front_tuning)		// 旋转到目标角度
										{
											if(dmStart(NON_CALIB, PICONLY))
											{
												std::cout<<"X:"<<result.delta_x<<" Y:"<<result.delta_y<<" Angle:"<<result.delta_angle<<endl;
												test_speed.set_vx(0);	test_speed.set_vy(0);	
												double dgreee;
												if(init_angle == 90)
												{
													if(result.delta_angle > 0)
														dgreee = result.delta_angle - 270;
													else
														dgreee = result.delta_angle + 90;
												}
												if(init_angle == -90)
												{	
													if(result.delta_angle > 0)
														dgreee = 90 + result.delta_angle;
													else
														dgreee = 90 + result.delta_angle;
												}
												test_speed.set_w(dgreee*0.007);
												SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
												if(fabs(fabs(result.delta_angle)-90)<1.0)
												{
													front_tuning = false;
													test_speed.set_vx(0);	test_speed.set_vy(0);	test_speed.set_w(0);
													SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);

													if(fabs(result.delta_x)<10 && fabs(result.delta_y)<10)
													{
														feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
														feedbackInfo.set_result(0);
														SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
														LOG_COUT_INFO(Docking_logger,"Under Docking Finish!");
													}
													else
													{
														feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_IN);
														feedbackInfo.set_result(1);
														SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
														LOG_COUT_INFO(Docking_logger,"Under Docking Field!");
													}
													enter_flag = false;
													finish_flag = true;
													Docking_Enable = false;
												}
											}						
										}
									}									
								}
								else
								{
									//test_speed.set_vx(fabs(result.delta_x)/result.delta_x*0.01);	test_speed.set_vy(0);	test_speed.set_w(0);
									if(target_angle == 0)
										test_speed.set_vx(-0.001*result.delta_x);
									if(target_angle == 180)
										test_speed.set_vx(0.001*result.delta_x);
									test_speed.set_vy(0);	test_speed.set_w(0);
									SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
								}
							}
						}
					}
					else
					{
						if(init_angle == 90)
							test_speed.set_vx(-0.001*result.delta_y);	
						if(init_angle == -90)
							test_speed.set_vx(0.001*result.delta_y);
						test_speed.set_vy(0);	test_speed.set_w(0);
						SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
					}
				}
			}
			}
		}
		printf("OVER\n");
		dmExit(PICONLY);
		//while(1)
		//{
		//	info_qr = dmStart(NON_CALIB, PICONLY);
		//}

//		pre_theta = 0;
//		no_find_times = 0;
//		start_odm_flag = false;
//		back_time_flag = 0;
//		back_dist = 0;
//		filter_flag = false;	filter_num = 0;	sum_x = 0;	sum_y = 0;	sum_theta = 0;	sum_HX_theta = 0;
	}
	if(leave_flag && Docking_Enable)
	{
		LOG_COUT_INFO(Docking_logger,"Get Docking_GO_OUT task");

		SafeX.edge_small =	1.4;	SafeX.edge_large = 1.5;
		SafeY.edge_small = -0.5;	SafeY.edge_large = 0.5;

		curLeave_pose.x = 0.0; curLeave_pose.y = 0.0; curLeave_pose.theta = 0.0;
		preLeave_odom = curLeave_odom;

		while(leave_flag)
		{
			deltaPose = absoluteDifference(curLeave_odom, preLeave_odom);
			curLeave_pose = absoluteSum(curLeave_pose,deltaPose);
			preLeave_odom = curLeave_odom;
			double leave_dist = curLeave_pose.x*curLeave_pose.x + curLeave_pose.y*curLeave_pose.y;
			if(leave_dist>docking_out_dist*docking_out_dist)
			{
				test_speed.set_vx(0);	test_speed.set_vy(0); test_speed.set_w(0);
				feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_OUT);
				feedbackInfo.set_result(0);
				SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
				printf("Dock Out Successfully!\n");
				leave_flag = false;
				Docking_Enable = false;
			}
			else
			{
				test_speed.set_vx(sin(sqrt(leave_dist)*M_PI/docking_out_dist)*0.2+0.03);	test_speed.set_vy(0); test_speed.set_w(0);
			}
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
		}
	}
	// handle elevator task
	if(Elevator_Enable)
	{
		printf("Get Elevator Task\n");
	}
	//// leave elevator
	//if(leave_flag && Elevator_Enable)
	//{
	//	printf("Get Elevator_GO_OUT task\n");
	//	curLeave_pose.x = 0.0; curLeave_pose.y = 0.0; curLeave_pose.theta = 0.0;
	//	preLeave_odom = curLeave_odom;

	//	while(leave_flag)
	//	{
	//		deltaPose = absoluteDifference(curLeave_odom, preLeave_odom);
	//		curLeave_pose = absoluteSum(curLeave_pose,deltaPose);
	//		preLeave_odom = curLeave_odom;
	//		double leave_dist = curLeave_pose.x*curLeave_pose.x + curLeave_pose.y*curLeave_pose.y;
	//		if(leave_dist>0.1*0.1)	// to be modified
	//		{
	//			test_speed.set_vx(0);	test_speed.set_vy(0); test_speed.set_w(0);
	//			feedbackInfo.set_flag(feedbackInfo.ECTF_UNDER_DOCK_OUT);
	//			feedbackInfo.set_result(0);
	//			SubPubManager::Instance()->m_controltaskresult.GetPublisher()->publish(feedbackInfo);
	//			printf("Leave Elevator Successfully!\n");
	//			leave_flag = false;
	//			Elevator_Enable = false;
	//		}
	//		else
	//		{
	//			test_speed.set_vx(0.03);	test_speed.set_vy(0); test_speed.set_w(0);
	//		}
	//		SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(test_speed);
	//	}
	//}
}

void Set_Dock_Para()
{
	double ElevatorLenth;
	double ElevatorInDist;
	double ElevatorOutDist;
	double DockingInDist;
	double DockingOutDist;
	double DockingSafeDist;

	NRF_ParamReader::Instance()->readParams(PATH_PARAM.c_str()); // "./params/NR_AGV_param.xml"
	{
		DECLARE_PARAM_READER_BEGIN(Docking)
		READ_PARAM(ElevatorLenth)
		READ_PARAM(ElevatorInDist)
		READ_PARAM(ElevatorOutDist)
		READ_PARAM(DockingInDist)
		READ_PARAM(DockingOutDist)
		READ_PARAM(DockingSafeDist)
		DECLARE_PARAM_READER_END
	}

	elevator_lenth		= ElevatorLenth;			// 电梯横板底部的长度
	elevator_in_dist	= ElevatorInDist;			// 进入电梯到点距离电梯横板的距离
	elevator_out_dist	= ElevatorOutDist;			// 出电梯到点距离电梯横板的距离
	docking_in_dist		= DockingInDist;			// 精确对接最大的前进的距离
	docking_out_dist	= DockingOutDist;			// 对接出站的盲走距离
	SafeDist			= DockingSafeDist;			// 模块运行中激光停障距离

	printf("Docking Init Completed\n");
}

int main(int argc,char* argv)
{
	cv::namedWindow("HELLO");
	readLog4cppConfigure(WORKING_DIR+"params/log4cpp.conf");	// log init
	using namespace std;
	//============ROS/SURO publisher=======================//
	std::cout<<"done\nInitializing SURO...";
	NODE.init("NR_Module_DockingNav");
	LOG_INFO(Docking_logger,"***********************************************");
	LOG_INFO(Docking_logger," Under_Docking Module Vision 2.0.0.20160307_α ");
	LOG_INFO(Docking_logger,"***********************************************");
	Set_Dock_Para();
	//// single test
	//Docking_Enable = true;
	//enter_flag = true;
	//reach_flag = 0;
	//EdgeX.edge_small = -0.5;	EdgeX.edge_large = 2.0;
	//EdgeY.edge_small = -1.0;	EdgeY.edge_large = 1.0;

	// handle message
	SubPubManager::Instance()->m_navtask.Initialize(Topic::Topic_NavTask,updateMotionFlags);			//接收任务指令
//	SubPubManager::Instance()->m_urg_laser.Initialize(Topic::Topic_Laser, updateLaser);					//接收激光数据
	SubPubManager::Instance()->m_odometer.Initialize(Topic::Topic_Odometer,updateOdom);					//接收里程计数据
	SubPubManager::Instance()->m_robotspeed.Initialize(Topic::Topic_Speed, NULL);						//发布机器人速度
	SubPubManager::Instance()->m_controltaskresult.Initialize(Topic::Topic_CtrlTaskResult, NULL);		//发布docking反馈信息
	SubPubManager::Instance()->m_commoninfo.Initialize(Topic::Topic_CommonInfo,NULL);					//发布机器人相关信息

	Sleep(500);
	cv::destroyWindow("HELLO");
	//进入spin函数，程序循环等待，当激光数据到来后，调用'回调函数updateLaser()，在这里面进行处理并将机器人控制速度发布出去
	//NODE.spin();
}