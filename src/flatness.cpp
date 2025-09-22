#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
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
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/MountControl.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include "util.cpp"

float pi = 3.14;
bool ini_pose = 0;
Eigen::Vector3f acc_enu;
Eigen::Vector3f fwpose;
Eigen::Vector3f fwVel;
Eigen::Vector3f fweuler;
Eigen::Vector3f offset_position;
Eigen::Quaternionf quat_planeEarth_flu;
Eigen::Matrix3f R_planeEarth_frd;
Eigen::Matrix3f R_frd;

ros::Subscriber imu_acc;
ros::Subscriber fw_pose_sub;
ros::Publisher att_pub;
ros::Publisher pos_pub;
ros::Publisher acc_pub;
ros::Publisher phi_pub;
ros::Publisher bx_pub;
ros::Publisher by_pub;
ros::Publisher desired_acc;
ros::ServiceClient mode_client;
mavros_msgs::AttitudeTarget att_msg;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    Eigen::Vector3f acc_body(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z
    );

    Eigen::Quaternionf q(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z
    );

    Eigen::Matrix3f R = q.toRotationMatrix();
    acc_enu = R * acc_body;
}

void getFwPose(const nav_msgs::Odometry::ConstPtr& odom)
{

    fwpose << odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z;

    fwVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    
    quat_planeEarth_flu = Eigen::Quaternionf( odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z );
    fweuler = Quaternion2Euler(quat_planeEarth_flu);
    R_planeEarth_frd =  R_frd * Quat2RotaMatrix(quat_planeEarth_flu);

    if(odom && !ini_pose){
        offset_position = fwpose;
        ini_pose = 1;
    }

    
}

geometry_msgs::Quaternion eulerToQuat(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf rollAngle(roll,   Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw,     Eigen::Vector3f::UnitZ());

    // 注意旋轉順序：ZYX (yaw → pitch → roll)
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;

    geometry_msgs::Quaternion q_msg;
    q_msg.w = static_cast<double>(q.w());
    q_msg.x = static_cast<double>(q.x());
    q_msg.y = static_cast<double>(q.y());
    q_msg.z = static_cast<double>(q.z());

    return q_msg;
}

int main(int argc, char** argv){
    R_frd<<1,0,0,0,-1,0,0,0,-1;
    ros::init(argc, argv, "flatness");
    ros::NodeHandle nh;
    ros::Rate rate=30;
    float C_sla,C_sl0,C_sda,C_sd0,C_s;
    C_sla = 11.54928;
    C_sl0 = -0.67307;
    C_sda = 1.55936;
    C_sd0 = -0.09088;
    C_s = C_sd0 - C_sda*C_sl0/C_sla;
    Eigen::Vector3f e_iz = {0,0,1};
    float M = 2.07;
    float rho = 1.2041;
    // float C_t = 104.72;
    float C_t = 52.36;
    Eigen::Matrix3f K_v;
    K_v << 0.05,0,0,0,0.05,0,0,0,0.02;
    Eigen::Matrix3f K_p;
    K_p << 0.005,0,0,0,0.005,0,0,0,0.001;
    Eigen::Matrix3f I;
    I << 1,0,0,0,1,0,0,0,1;

    mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    acc_pub = nh.advertise<std_msgs::Float32>("desired_velocity", 10);
    phi_pub = nh.advertise<std_msgs::Float32>("truth_velocity", 10);
    bx_pub = nh.advertise<geometry_msgs::Vector3>("bx_vector", 10);
    by_pub = nh.advertise<geometry_msgs::Vector3>("by_vector", 10);
    desired_acc = nh.advertise<geometry_msgs::Vector3>("acc_vector", 10);

    geometry_msgs::PoseStamped dummy_pose;
    dummy_pose.pose.position.z = 2;
    for (int i = 0; i < 10; ++i) {
        pos_pub.publish(dummy_pose);
        ros::Duration(0.01).sleep();
    }
    mavros_msgs::SetMode mode_cmd;
    mode_cmd.request.custom_mode = "OFFBOARD";
    if (mode_client.call(mode_cmd) && mode_cmd.response.mode_sent) {
        ROS_INFO("Set OFFBOARD success");
    } else {
        ROS_WARN("Set OFFBOARD failed");
    }

    imu_acc = nh.subscribe("/uav0/mavros/imu/data", 10, imuCallback);
    fw_pose_sub = nh.subscribe("/uav0/base_pose_ground_truth", 10, getFwPose);
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/attitude", 10);
    Eigen::Vector3f a_g = {0,0,9.81};

    float start_time = ros::Time::now().toSec();

    // Eigen::Vector3f offset_position = fwpose;

    while(ros::ok){
        float offset_time = ros::Time::now().toSec() - start_time;
        // Eigen::Vector3f traj = {8*offset_time,100*sinf(0.2*offset_time),0};
        // Eigen::Vector3f p_dv = {8,20*cosf(0.2*offset_time),0};
        // Eigen::Vector3f p_da = {0,-4*sinf(0.2*offset_time),0};
        Eigen::Vector3f traj = {80*sinf(pi*offset_time/20),80*cosf(pi*offset_time/20)-80,0};
        Eigen::Vector3f p_dv = {4*pi*cosf(pi*offset_time/20),-4*pi*sinf(pi*offset_time/20),0};
        Eigen::Vector3f p_da = {-0.2*pi*pi*sinf(pi*offset_time/20),-0.2*pi*pi*cosf(pi*offset_time/20),0};
        Eigen::Vector3f p_d = traj + offset_position;
        Eigen::Vector3f p_ca = p_da + K_v*(p_dv-fwVel)+K_p*(p_d-fwpose);
        // Eigen::Vector3f p_ca = p_da + K_v*(p_dv-fwVel);
        Eigen::Vector3f e_vx = p_dv/p_dv.norm();
        float a_vx = e_vx.dot(p_ca - a_g);
        Eigen::Vector3f a_vz = (p_ca - a_g - a_vx*e_vx);
        Eigen::Vector3f a_v = {a_vx, 0, a_vz.norm()};
        Eigen::Vector3f e_vz = a_vz/a_vz.norm();
        Eigen::Vector3f e_vy = e_vz.cross(e_vx);
        Eigen::Matrix3f R_v;
        R_v << e_vx, e_vy, e_vz;
        R_v.transposeInPlace();
        float V = fwVel.norm();
        float alpha = 2*M*e_iz.dot(a_v)/(rho*V*V*C_sla)-C_sl0/C_sla;
        Eigen::Matrix3f R_a;
        R_a << cosf(alpha),0,-sinf(alpha),0,1,0,sinf(alpha),0,cosf(alpha);
        Eigen::Matrix3f R_b = R_v*R_a;
        Eigen::Matrix3f R_h;
        R_h << 0,1,0,-1,0,0,0,0,0;
        // R_h << 1,0,0,0,1,0,0,0,0;
        Eigen::Vector3f e_bx = R_b.col(0);
        Eigen::Vector3f e_by = R_b.col(1);
        std::cout << e_by.dot(R_h*e_bx)<< std::endl;
        float theta = -asinf(e_iz.dot(e_bx));
        float phi = std::copysign(1.0f, e_iz.dot(e_by))*acosf(e_by.dot(R_h*e_bx)/(R_h*e_bx).norm());
        float f1 = M*p_dv.dot(p_ca-a_g)/(C_t*p_dv.norm());
        float f2 = M*C_sda*((p_dv.squaredNorm()*I-p_dv*p_dv.transpose())*(p_ca-a_g)).norm()/(C_t*C_sla*p_dv.squaredNorm());
        float f3 = rho*C_s*p_dv.squaredNorm()/(2*C_t);
        float thrust = f1+f2+f3;

        att_msg.type_mask =
            mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
            mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
            mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
        att_msg.orientation = eulerToQuat(-phi, -theta, 0);
        att_msg.thrust = thrust;
        att_pub.publish(att_msg);
        //-----------------------------for plot-------------------------------------------
        std_msgs::Float32 d_acc;
        d_acc.data = p_dv.norm();
        acc_pub.publish(d_acc);
        std_msgs::Float32 r_phi;
        r_phi.data = V;
        phi_pub.publish(r_phi);

        geometry_msgs::Vector3 bx_msg;
        bx_msg.x = e_bx[0];
        bx_msg.y = e_bx[1];
        bx_msg.z = e_bx[2];
        bx_pub.publish(bx_msg);

        geometry_msgs::Vector3 by_msg;
        by_msg.x = e_by[0];
        by_msg.y = e_by[1];
        bx_msg.z = e_by[2];
        by_pub.publish(by_msg);

        if(ini_pose){
            geometry_msgs::Vector3 acc_msg;
            acc_msg.x = p_d[0];
            acc_msg.y = p_d[1];
            acc_msg.z = p_d[2];
            desired_acc.publish(acc_msg);
        }
        //---------------------------------------------------------------------------------
        std::cout << "phi: " << phi <<std::endl<<"theta: "<<theta<<std::endl<<"thrust: "<< thrust<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
}