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
#include <fw_control_plan/EstimateOutput.h>

#define N_1 6
#define N_2 2

// #define desire_velocity 12.0
#define theta_t (30 * M_PI / 180.0)
#define phi_t (0 * M_PI / 180.0)

geometry_msgs::Pose carPos_truth;
geometry_msgs::Pose carPos_ukf;
geometry_msgs::Pose fwPos;
Eigen::Quaternionf quat_planeEarth_flu;
Eigen::Matrix3f R_planeEarth_frd;
Eigen::Matrix3f R_frd;
Eigen::Matrix3f R_enu;
Eigen::Matrix3f R_panPlane;
Eigen::Matrix3f R_camPan;
Eigen::Vector3f Carpos_truth;
Eigen::Vector3f Carpos_ukf;  
Eigen::Vector3f carVel_truth;
Eigen::Vector3f carVel_ukf;
Eigen::Vector3f fwpose;
Eigen::Vector3f fwVel;
Eigen::Vector3f fweuler;  
std::vector<float> gimbalAng{0.0, 0.0, 0.0};
std::vector<float> gimbalAngVel{0.0, 0.0, 0.0}; 
ros::Subscriber car_odom_sub;
ros::Subscriber ukf_sub;
ros::Subscriber fw_pose_sub;
ros::Subscriber gimbal_sub;
ros::Publisher att_pub;
mavros_msgs::AttitudeTarget cmd_att;
float dT;
float gravity=9.81;
float kp = 0.002;
void getFwPose(const nav_msgs::Odometry::ConstPtr& odom);
void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);
void getGimbalState(const sensor_msgs::JointState::ConstPtr& state);
void getUKFResults(const fw_control_plan::EstimateOutput::ConstPtr& data);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternoion_bpng");
    ros::NodeHandle nh;
    ros::Rate rate = 30;
    car_odom_sub = nh.subscribe("/wamv/base_pose_ground_truth", 10, getAgentOdom);          //for boat simulation
    ukf_sub = nh.subscribe<fw_control_plan::EstimateOutput>("/uav0/estimation/ukf/output_data", 10, getUKFResults);
    // car_odom_sub = nh.subscribe("/prius/pose_ground_truth", 10, getAgentOdom);             //for car simulation  
    fw_pose_sub = nh.subscribe("/uav0/base_pose_ground_truth", 10, getFwPose);
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/attitude", 10);
    R_enu<<0,1,0,1,0,0,0,0,-1;

    while(ros::ok()){
        Eigen::Vector3f r_NED = R_enu*(Carpos_truth - fwpose);
        Eigen::Vector3f v_NED = R_enu*(carVel_truth - fwVel);
        
        //-------------------------------calculate BPNG----------------------------------
        Eigen::Vector3f u_f_ , v_m_;
        u_f_ = Eigen::Vector3f(cos(theta_t) * cos(phi_t), cos(theta_t) * sin(phi_t), -sin(theta_t));
        v_m_ = fwVel;
        double r_n = r_NED.norm();
        Eigen::Vector3f w = (r_NED.cross(v_NED)) / (r_n * r_n);
        Eigen::Vector3f a_pn = N_1 * (w.cross(v_NED));

        double t_go = r_n / v_NED.norm();
        // t_go = std::max(t_go, 12.0);
        double theta = acos(v_m_.normalized().dot(u_f_.normalized())); 
        Eigen::Vector3f a_bpng = (N_2 / t_go) * v_m_.cross( v_m_.cross(u_f_) / v_m_.cross(u_f_).norm()) * theta ;
        
        Eigen::Vector3f a_total = R_enu*(- a_pn + a_bpng);
        
        //-----------------------------command transform---------------------------------
        Eigen::Vector3f a_body;
        a_body = R_planeEarth_frd*a_total;
        float abs_velocity = fwVel.norm();
        float cal_yawrate = a_body[1]/abs_velocity;
        float cal_pitchrate = a_body[2]/abs_velocity;
        std::cout<<"=========================================================\n"
        <<"Calculate pitch rate: "<<cal_pitchrate<<" rad/s\n"
        <<"Calculate yaw rate: "<<cal_yawrate<< " rad/s\n"
        <<"Body frame acc: "<<a_body<<std::endl
        <<"Theta rotate: "<<theta<<std::endl;
        float current_thrust = 0.3;
        cmd_att.thrust = current_thrust + kp*(20- abs_velocity);///nmpc_cmd[0];
        if(cmd_att.thrust<0){
            cmd_att.thrust=0;
        }
        if(cmd_att.thrust>1){
            cmd_att.thrust=1;
        }
        current_thrust = cmd_att.thrust;
        cmd_att.body_rate.x = (std::atan2(abs_velocity*cal_yawrate, 9.81)-fweuler[0])*2;
        cmd_att.body_rate.y = cal_pitchrate;
        cmd_att.body_rate.z = cal_yawrate;
        cmd_att.type_mask = 132;
        att_pub.publish(cmd_att);
        ros::spinOnce();
        rate.sleep();
        // std::cout<<fwVel<<std::endl;
        // std::cout<<abs_velocity<<std::endl;
    }
}

void getFwPose(const nav_msgs::Odometry::ConstPtr& odom)
{
    fwPos.position.x = odom->pose.pose.position.x; 
    fwPos.position.y = odom->pose.pose.position.y;
    fwPos.position.z = odom->pose.pose.position.z; 
    //std::cout<<fwPos<<std::endl;
    fwpose << fwPos.position.x, fwPos.position.y, fwPos.position.z;

    fwVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    
    quat_planeEarth_flu = Eigen::Quaternionf( odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z );
    fweuler = Quaternion2Euler(quat_planeEarth_flu);
    R_frd<<1,0,0,0,-1,0,0,0,-1;
    R_planeEarth_frd =  R_frd * Quat2RotaMatrix(quat_planeEarth_flu);
    
}

void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    carVel_truth << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    carPos_truth.position.x = odom->pose.pose.position.x;
    carPos_truth.position.y = odom->pose.pose.position.y;
    carPos_truth.position.z = odom->pose.pose.position.z;
    Carpos_truth = {carPos_truth.position.x,carPos_truth.position.y,carPos_truth.position.z};
    //std::cout<<carPos<<std::endl;
}

void getUKFResults(const fw_control_plan::EstimateOutput::ConstPtr& data)
{
    carPos_ukf.position.x = data->target_pose.x;
    carPos_ukf.position.y = data->target_pose.y;
    carPos_ukf.position.z = data->target_pose.z;
    Carpos_ukf = {carPos_ukf.position.x,carPos_ukf.position.y,carPos_ukf.position.z};
    carVel_ukf << data->target_vel.x, data->target_vel.y, data->target_vel.z;
    // ukf_x1 = data->feature_1.data;
    // ukf_x2 = data->feature_2.data;
}

void getGimbalState(const sensor_msgs::JointState::ConstPtr& state)
{
    gimbalAng[0] = state->position[0];
    gimbalAng[1] = state->position[1];
    gimbalAng[2] = state->position[2];
    gimbalAngVel[0] = state->velocity[0];
    gimbalAngVel[1] = state->velocity[1];
    gimbalAngVel[2] = state->velocity[2];
    //std::cout<<gimbalAng<<std::endl;

}