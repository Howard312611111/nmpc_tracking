#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/MountControl.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "util.cpp"
#include <casadi/casadi.hpp>
ros::Subscriber car_odom_sub;
ros::Subscriber fw_pose_sub;
Eigen::Quaternionf quat_planeEarth_flu;
geometry_msgs::Pose carPos;
geometry_msgs::Pose fwPos;
Eigen::Vector3f Carpos;
Eigen::Vector3f carVel;
Eigen::Vector3f fwVel;
Eigen::Vector3f fweuler; 
Eigen::Matrix3f R_frd;
void getFwPose(const nav_msgs::Odometry::ConstPtr& odom);
void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);

int main(int argc, char **argv){
    ros::init(argc, argv, "nmpc_draw");
    ros::NodeHandle nh;
    ros::Rate rate = 10;
    car_odom_sub = nh.subscribe("/wamv/base_pose_ground_truth", 10, getAgentOdom);
    fw_pose_sub = nh.subscribe("/uav0/base_pose_ground_truth", 10, getFwPose);
    R_frd << 1,0,0,0,-1,0,0,0,-1;
    while(ros::ok()){
        float cal_car_vel, cal_fw_vel, cal_xy_dis;
        cal_car_vel = sqrt(carVel[0]*carVel[0]+carVel[1]*carVel[1]+carVel[2]*carVel[2]);
        Eigen::Matrix3f rot_yaw = rotationMatrix('Z',-fweuler[2]);
        Eigen::Vector3f rel_pose;
        rel_pose << carPos.position.x-fwPos.position.x, carPos.position.y-fwPos.position.y, carPos.position.z-fwPos.position.z;
        cal_xy_dis = sqrt(rel_pose[0]*rel_pose[0]+rel_pose[1]*rel_pose[1]);
        cal_fw_vel = sqrt(fwVel[0]*fwVel[0]+fwVel[1]*fwVel[1]+fwVel[2]*fwVel[2]);
        Eigen::Vector3f yaw_rel = rot_yaw*R_frd*rel_pose;
        float limit_angle = atan2(yaw_rel[2],yaw_rel[1]);
        std::cout << "Yaw frame ref: " << yaw_rel << std::endl ;
        std::cout << "The xy plane distance: "<<cal_xy_dis<<std::endl;
        // std::cout << "The limit angle: "<<limit_angle<<" roll angle: "<<fweuler[0] <<std::endl;
        std::cout << "The UAV velovity: "<< cal_fw_vel << std::endl;
        ros::spinOnce();
        rate.sleep();

    }
}

void getFwPose(const nav_msgs::Odometry::ConstPtr& odom)
{
    fwPos.position.x = odom->pose.pose.position.x; 
    fwPos.position.y = odom->pose.pose.position.y;
    fwPos.position.z = odom->pose.pose.position.z; 
    //std::cout<<fwPos<<std::endl;

    fwVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    
    quat_planeEarth_flu = Eigen::Quaternionf( odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z );
    fweuler = Quaternion2Euler(quat_planeEarth_flu);
    // R_planeEarth_frd =  R_frd * Quat2RotaMatrix(quat_planeEarth_flu);
    
}

void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    carVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    carPos.position.x = odom->pose.pose.position.x;
    carPos.position.y = odom->pose.pose.position.y;
    carPos.position.z = odom->pose.pose.position.z;
    Carpos = {carPos.position.x,carPos.position.y,carPos.position.z};
    //std::cout<<carPos<<std::endl;
}