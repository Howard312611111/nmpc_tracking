#pragma once
// BPNG_3D
#ifndef _UAVSTATE_H_20230801
#define	_UAVSTATE_H_20230801

#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/MountControl.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/AttitudeTarget.h>

///////////////////////////////////////////////////////////////////////////////
// UAV state
///////////////////////////////////////////////////////////////////////////////
class UAVState
{
private:
	double x, y, z;
	double vx, vy, vz;
	double q_x, q_y, q_z, q_w;
	double phi_ang, theta_ang, psi_ang;
	double xm_thm=0, ym_thm=0, zm_thm=0;
	double vxm_thm=0, vym_thm=0, vzm_thm=0;
	ros::NodeHandle nhdl;
	ros::Rate rate = 20.0;

	// publisher and subscriber
	ros::Subscriber car_odom_sub;
	ros::Subscriber uav_pose_sub;
	ros::Subscriber uav_velocity_body_sub;
	ros::Publisher  uav_cmd_waypint_pub;
	ros::Publisher  uav_cmd_vel_pub;
	ros::Publisher  uav_cmd_unstamped_pub;
	ros::Publisher  uav_cmd_acc_pub;
	ros::Publisher  uav_cmd_att_thr_pub;

	Eigen::Vector3f carVel;
	geometry_msgs::Pose carPos;
	Eigen::Vector3f uavLinearVel;
	Eigen::Vector3f uavAngularVel;
	geometry_msgs::Pose uavPos;
	geometry_msgs::TwistStamped uavLinearVelCmd;
	geometry_msgs::Twist uavLinearVelCmdUstamped;
	geometry_msgs::PoseStamped uavWaypointCmd;
	geometry_msgs::Vector3Stamped uavLinearAccCmd;
	mavros_msgs::AttitudeTarget uavAttitudeThrustCmd;


public:

	UAVState();  // constructor
	~UAVState() {}; // destructor

	void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);
	void getUavPos(const geometry_msgs::PoseStamped::ConstPtr& pos);
	void getUavVel(const geometry_msgs::TwistStamped::ConstPtr& vel);

	void showPosVel();

	double get_dis_waypoint(double x_wp, double y_wp, double z_wp);

	void waypoint_cmd(double x_wp, double y_wp, double z_wp);
	void vel_cmd(double vx, double vy, double vz);

	void test_raw_attitude(double vxm, double vym, double vzm);
	void acc_cmd_test(double acc_x, double acc_y, double acc_z);
	void bpng_azi_acc_cmd(double vari);
	void bpng_acc_cmd(double varia, double varie);
};



#endif _UAVSTATE_H_20230801
