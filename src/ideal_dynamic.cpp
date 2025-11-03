#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <limits>

std::vector<float> nmpc_cmd{0.0,0.0,0.0};

void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0;i<3;i++){
        nmpc_cmd[i] = msg->data[i];
    }
}

Eigen::Quaternionf eulerToQuat(float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q;
}

inline bool isValid(float v) {
    return std::isfinite(v);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ideal_dynamic");
    ros::NodeHandle nh;

    ros::Publisher wamv_pub = nh.advertise<nav_msgs::Odometry>("/wamv/base_pose_ground_truth", 10);
    ros::Publisher uav_pub  = nh.advertise<nav_msgs::Odometry>("/uav0/base_pose_ground_truth", 10);
    ros::Subscriber ans_sub = nh.subscribe("/nmpc_ans",10,getAns);

    ros::Rate rate(10); // 10 Hz 更新率

    // === WAMV initial state ===
    double wamv_x = 1000.0;
    double wamv_y = 0.0;
    double wamv_z = 0.0;
    double wamv_vx = 10.0; // constant velocity in x

    // === UAV initial state ===
    float uav_x = 0.0f;
    float uav_y = 0.0f;
    float uav_z = 1000.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();

        // ---------------- WAMV ----------------
        nav_msgs::Odometry wamv_odom;
        wamv_odom.header.stamp = now;
        wamv_odom.header.frame_id = "world";
        wamv_odom.child_frame_id = "wamv_base";

        // Update WAMV pose (x increases at 10 m/s)
        wamv_x += wamv_vx * (1.0 / 10.0); // since rate = 10 Hz
        wamv_odom.pose.pose.position.x = wamv_x;
        wamv_odom.pose.pose.position.y = wamv_y;
        wamv_odom.pose.pose.position.z = wamv_z;

        // Orientation: keep identity (no rotation)
        wamv_odom.pose.pose.orientation.w = 1.0;

        // Velocity
        wamv_odom.twist.twist.linear.x = wamv_vx;

        wamv_pub.publish(wamv_odom);

        // ---------------- UAV ----------------
        float dt = 1.0f / 10.0f;

        //----------UAV 6 order dynamic---------
        float x_dot = nmpc_cmd[0]*cos(yaw)*cos(pitch);
        float y_dot = nmpc_cmd[0]*sin(yaw)*cos(pitch);
        float z_dot = -nmpc_cmd[0]*sin(pitch);
        float roll_dot = nmpc_cmd[1]+sin(roll)*tan(pitch)*nmpc_cmd[2];
        float pitch_dot = nmpc_cmd[2]*cos(roll);
        float yaw_dot = -(9.81f/nmpc_cmd[0])*tan(roll)*cos(pitch);

        // === 檢查數值是否有效 ===
        if ( isValid(x_dot) && isValid(y_dot) && isValid(z_dot) &&
             isValid(roll_dot) && isValid(pitch_dot) && isValid(yaw_dot) )
        {
            // === 狀態積分更新 ===
            uav_x += x_dot * dt;
            uav_y += y_dot * dt;
            uav_z += z_dot * dt;
            roll  += roll_dot * dt;
            pitch += pitch_dot * dt;
            yaw   += yaw_dot * dt;
        }


        // === 封裝 Odometry ===
        nav_msgs::Odometry uav_odom;
        uav_odom.header.stamp = now;
        uav_odom.header.frame_id = "world";
        uav_odom.child_frame_id = "uav0_base";

        // 位置
        uav_odom.pose.pose.position.x = uav_x;
        uav_odom.pose.pose.position.y = uav_y;
        uav_odom.pose.pose.position.z = uav_z;

        // 姿態 (轉 quaternion)
        Eigen::Quaternionf q = eulerToQuat(roll, pitch, yaw);
        uav_odom.pose.pose.orientation.w = q.w();
        uav_odom.pose.pose.orientation.x = q.x();
        uav_odom.pose.pose.orientation.y = q.y();
        uav_odom.pose.pose.orientation.z = q.z();

        // twist (linear, angular)
        uav_odom.twist.twist.linear.x = x_dot;
        uav_odom.twist.twist.linear.y = y_dot;
        uav_odom.twist.twist.linear.z = z_dot;

        uav_odom.twist.twist.angular.x = roll_dot;
        uav_odom.twist.twist.angular.y = pitch_dot;
        uav_odom.twist.twist.angular.z = yaw_dot;

        uav_pub.publish(uav_odom);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
