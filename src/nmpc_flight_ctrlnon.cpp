#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/TwistStamped.h>

ros::Subscriber ans_sub;
ros::Subscriber state_sub;
ros::Subscriber fw_pose_sub;
ros::Publisher vel_pub;
ros::Publisher att_pub;
ros::Publisher yawVelCommand_pub;
ros::Publisher pitchVelCommand_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

ros::Time last_request;
geometry_msgs::TwistStamped cmd_vel;
mavros_msgs::State current_state;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool cmd_arm;
mavros_msgs::AttitudeTarget cmd_att;
std_msgs::Float32 yawRate;
std_msgs::Float32 pitchRate;
std::vector<float> nmpc_cmd{0.0,0.0,0.0};
Eigen::Vector3f fwVel;
Eigen::Vector3f fwAng;
Eigen::Vector3f next_ang;

void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg);
void getCurrentState(const mavros_msgs::State::ConstPtr& state);
void SwitchFlightMode(std::string flightMode);
void getFwPose(const nav_msgs::Odometry::ConstPtr& pose);
Eigen::Quaternionf Euler2Quaternion(Eigen::Vector3f euler);

int main(int argc, char **argv){
    ros::init(argc, argv, "nmpc_flight_ctrlnon");
    ros::NodeHandle nh;
    ros::Rate rate = 10;
    state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state",10,getCurrentState);
    ans_sub = nh.subscribe("/nmpc_ans",10,getAns);
    fw_pose_sub = nh.subscribe("/uav0/base_pose_ground_truth", 10, getFwPose);
    vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav0/mavros/setpoint_velocity/cmd_vel", 10);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/uav0/mavros/setpoint_raw/attitude", 10);
    std::cout<<"HI"<<std::endl;
    SwitchFlightMode("OFFBOARD");
    std::cout << "Switch to Offboard mode. Ready to fly!" << std::endl;
    float kp = 0.005;
    float current_thrust = 0.3;
    while(ros::ok()){
        cmd_att.thrust = current_thrust + kp*(nmpc_cmd[0]- fwVel.norm());///nmpc_cmd[0];
        if(cmd_att.thrust<0.2){
            cmd_att.thrust=0.2;
        }
        if(cmd_att.thrust>1){
            cmd_att.thrust=1;
        }
        current_thrust = cmd_att.thrust;
        float yawcal = -9.81*tan(fwAng(0))*cos(fwAng(1))/nmpc_cmd[0];
        //trying
        // next_ang << nmpc_cmd[1],nmpc_cmd[2],yawcal;
        // next_ang = fwAng + next_ang*0.1;
        // Eigen::Quaternionf next_qua = Euler2Quaternion(next_ang);
        // cmd_att.orientation.x = next_qua.x();
        // cmd_att.orientation.y = next_qua.y();
        // cmd_att.orientation.z = next_qua.z();
        // cmd_att.orientation.w = next_qua.w();

        cmd_att.body_rate.x = nmpc_cmd[1];
        cmd_att.body_rate.y = nmpc_cmd[2];
        cmd_att.body_rate.z = yawcal;
        cmd_att.type_mask = 132;
        att_pub.publish(cmd_att);
        ros::spinOnce();
        rate.sleep();
    }


}

void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0;i<3;i++){
        nmpc_cmd[i] = msg->data[i];
    }
}

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
        //std::cout<<"HIHI"<<std::endl;
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

void getFwPose(const nav_msgs::Odometry::ConstPtr& pose)
{
    fwAng = Eigen::Quaternionf(pose->pose.pose.orientation.w,pose->pose.pose.orientation.x,pose->pose.pose.orientation.y,pose->pose.pose.orientation.z).toRotationMatrix().eulerAngles(0,1,2);
    fwVel << pose->twist.twist.linear.x, pose->twist.twist.linear.y, pose->twist.twist.linear.z;

}

Eigen::Quaternionf Euler2Quaternion(Eigen::Vector3f euler)
{
    Eigen::Quaternionf q;
    double roll = euler[0], pitch = euler[1], yaw = euler[2];

    /*** order: Z-Y-X ***/
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;

    return q;
}