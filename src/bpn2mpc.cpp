#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include "util.cpp"
#include <casadi/casadi.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

using namespace casadi;
geometry_msgs::Pose carPos;
geometry_msgs::Pose fwPos;
Eigen::Vector3f fwVel;
Eigen::Vector3f carVel;
Eigen::Quaternionf quat_planeEarth_flu;
Eigen::Vector3f fweuler;
Eigen::Matrix3f R_planeEarth_frd;
Eigen::Matrix3f R_enu;
Eigen::Vector3f Carpos;
std_msgs::Float32MultiArray ans_cmd;
ros::Subscriber state_sub;
ros::Subscriber car_odom_sub;
ros::Subscriber fw_pose_sub;
ros::Subscriber gimbal_sub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
// ros::Publisher nmpc_ans_pub;
ros::Publisher att_pub;
ros::Publisher vel_pub;
mavros_msgs::AttitudeTarget cmd_att;
ros::Time last_request;
geometry_msgs::TwistStamped cmd_vel;
mavros_msgs::State current_state;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool cmd_arm;

void getCurrentState(const mavros_msgs::State::ConstPtr& state)
{
    current_state = *state;
}

void SwitchFlightMode(std::string flightMode)
{
    offb_set_mode.request.custom_mode = flightMode;
    cmd_arm.request.value = true;
    ros::Rate rate2 = 5;

    last_request = ros::Time::now();
    while(ros::ok())
    {
        std::cout<<current_state.mode<<std::endl;
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(cmd_arm) &&
                cmd_arm.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if ( current_state.mode != flightMode &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent ){
                ROS_INFO("%s enabled", flightMode.c_str());
            }
            last_request = ros::Time::now();
            }
        }   
        if(current_state.mode == flightMode){
            break;
        }

        cmd_vel.twist.linear.x = 5;
        cmd_vel.twist.linear.y = 0;
        cmd_vel.twist.linear.z = 3;

        vel_pub.publish(cmd_vel);

        ros::spinOnce();
        rate2.sleep();
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
    R_planeEarth_frd =  R_enu * Quat2RotaMatrix(quat_planeEarth_flu);
    
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

int main(int argc, char **argv){
	R_enu <<1,0,0,
			0,-1,0,
			0,0,-1;
	float itergrate_theta = 0;
    ros::init(argc, argv, "bpn2mpc");
    ros::NodeHandle nh;
    ros::Rate rate = 10;
	state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state",10,getCurrentState);
    car_odom_sub = nh.subscribe("/wamv/base_pose_ground_truth", 10, getAgentOdom);
    fw_pose_sub = nh.subscribe("/uav0/base_pose_ground_truth", 10, getFwPose);
	att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/attitude", 10);
	vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
	std::cout<<"HI"<<std::endl;
	SwitchFlightMode("OFFBOARD");
    std::cout << "Switch to Offboard mode. Ready to fly!" << std::endl;
	int N=6;
	int etat=3;
	double k = 0.05;
	double kp = 0.002;
	double ki = 0.0005;
	while(ros::ok()){
		Eigen::Vector3f pm,pt,vm,vt,pr,vr;
		pm << fwPos.position.x,fwPos.position.y,fwPos.position.z;
		pt << Carpos[0],Carpos[1],Carpos[2];
		vm = fwVel;
		vt = carVel;
		pr = pt - pm;
		vr = vt - vm;
		Eigen::Vector3f u_r,u_v,u_t,u_b,u_v_prime;
		u_r = pr/pr.norm();
		u_v = vm/vm.norm();
		u_t = vt/vt.norm();
		Eigen::Vector3f v_b = u_t.cross(u_r);
		u_b = v_b/v_b.norm();
		Eigen::Vector3f v_temp = u_v.cross(u_b);
		v_temp = v_temp/v_temp.norm();
		u_v_prime = u_b.cross(v_temp);
		
		//plane pursuit part
		Eigen::Vector3f a_pp;
		float e = u_b.dot(u_v);
		v_temp = u_b.cross(u_v);
		a_pp = k*e*v_temp.cross(vm);

		//BPN guidance part
		Eigen::Vector3f omega;
		omega = pr.cross(vr)/(pr.norm()*pr.norm());
		v_temp = u_r.cross(u_v_prime);
		float dot_temp = v_temp.dot(u_b);
		float dot_temp2 = u_r.dot(u_v_prime);
		float gamma_prime = std::atan2(dot_temp,dot_temp2);

		v_temp = u_r.cross(u_t);
		dot_temp = v_temp.dot(u_b);
		dot_temp2 = u_r.dot(u_t);
		float gamma_f_prime = atan2(dot_temp,dot_temp2);
		float theta_prime = -gamma_prime-(N-1)*gamma_f_prime;
		if(!std::isnan(theta_prime)){
			itergrate_theta += theta_prime;
		}
		float b = kp*theta_prime+ki*itergrate_theta;
		v_temp = N*omega+b*u_b;

		Eigen::Vector3f a_bpn = v_temp.cross(vm)/5;
		Eigen::Vector3f am = a_bpn + a_pp;

		//command transform
		Eigen::Vector3f body_rate_cmd,vm_b,am_b;
		vm_b = R_planeEarth_frd*vm;
		am_b = R_planeEarth_frd*am;
		float abs_velocity = sqrt(vm_b[0]*vm_b[0]+vm_b[1]*vm_b[1]+vm_b[2]*vm_b[2]);
		float cal_yawrate = am_b[1]/abs_velocity;
		float cal_pitchrate = am_b[2]/abs_velocity;
		std::cout<<"=========================================================\n"
		<<"Calculate pitch rate: "<<cal_pitchrate<<" rad/s\n"
		<<"Calculate yaw rate: "<<cal_yawrate<< " rad/s\n"
		<<"Body frame acc: "<<am_b<<std::endl;
		float current_thrust = 0.3;
        cmd_att.thrust = current_thrust + kp*(20- fwVel.norm());///nmpc_cmd[0];
        if(cmd_att.thrust<0.2){
            cmd_att.thrust=0.2;
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
	}

}

