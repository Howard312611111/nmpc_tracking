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

using namespace casadi;
SX rotationMatrix_SX(char axis, SX theta);
geometry_msgs::Pose carPos;
geometry_msgs::Pose fwPos;
std_msgs::Float32MultiArray ans_cmd;
std_msgs::Float32 yawRate;
std_msgs::Float32 pitchRate;
Eigen::Quaternionf quat_planeEarth_flu;
Eigen::Matrix3f R_planeEarth_frd;
Eigen::Matrix3f R_frd;
Eigen::Matrix3f R_panPlane;
Eigen::Matrix3f R_camPan;
Eigen::Vector3f Carpos;  
Eigen::Vector3f Rel_carpos;
Eigen::Vector3f carVel;
Eigen::Vector3f fweuler;  
Eigen::Vector3f fwVel;
Eigen::Vector3f fwAng;
std::vector<float> gimbalAng{0.0, 0.0, 0.0};
std::vector<float> gimbalAngVel{0.0, 0.0, 0.0}; 
std::vector<float> nmpc_cmd{0.0,0.0,0.0,0.0,0.0};
ros::Subscriber car_odom_sub;
ros::Subscriber fw_pose_sub;
ros::Subscriber gimbal_sub;
ros::Subscriber ans_sub;
ros::Publisher nmpc_ans_pub;
ros::Publisher yawVelCommand_pub;
ros::Publisher pitchVelCommand_pub;

int N;                                                               //prediction horizon
float dT;
float gravity=9.81;
void getFwPose(const nav_msgs::Odometry::ConstPtr& odom);
void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);
void getGimbalState(const sensor_msgs::JointState::ConstPtr& state);
void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_control_gimbal");
    ros::NodeHandle nh;
    ros::Rate rate = 50;
    ans_sub = nh.subscribe("/nmpc_ans",10,getAns);
    car_odom_sub = nh.subscribe("/prius/pose_ground_truth", 10, getAgentOdom);
    fw_pose_sub = nh.subscribe("/miniyy_gimbal_camera0/base_pose_ground_truth", 10, getFwPose);
    gimbal_sub = nh.subscribe("/miniyy_gimbal_camera0/gimbal/joint_states", 10, getGimbalState);
    pitchVelCommand_pub = nh.advertise<std_msgs::Float32>("/miniyy_gimbal_camera0/gimbal/pitch/cmd_joint_velocity", 10);
    yawVelCommand_pub = nh.advertise<std_msgs::Float32>("/miniyy_gimbal_camera0/gimbal/yaw/cmd_joint_velocity", 10);
    nmpc_ans_pub = nh.advertise<std_msgs::Float32MultiArray>("/nmpc_ans",10);
    std::vector<float> x0;
    // x0  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    x0  = {0,0};
    // x0 = {0,0,0,0};

    while(ros::ok()){
        R_frd << 1, 0, 0, 0, -1, 0, 0, 0, -1;

        //std::cout<<carPos<<std::endl;

        R_panPlane = rotationMatrix('Z', gimbalAng[2]);
        R_camPan = rotationMatrix('X', 90*M_PI/180) * rotationMatrix('Z', 90*M_PI/180) * rotationMatrix('Y', gimbalAng[1]);
        Rel_carpos = {Carpos[0]-fwPos.position.x,Carpos[1]-fwPos.position.y,Carpos[2]-fwPos.position.z};
        Eigen::Vector3f carpose_c = R_camPan*R_panPlane*R_planeEarth_frd*Rel_carpos;
        
        // start building nmpc //
        N = 5;
        dT = 0.01;
        SX W = SX::zeros(11,11);
        W(8,8)=10;
        W(9,9)=10;
        std::vector<float> X0 = {fwPos.position.x,fwPos.position.y,fwPos.position.z,fweuler[0],fweuler[1],fweuler[2],gimbalAng[2],gimbalAng[1],carpose_c[0]/carpose_c[2],carpose_c[1]/carpose_c[2],1/carpose_c[2]};
        //std::vector<SX> X_temp = {fwPos.position.x,fwPos.position.y,fwPos.position.z,fweuler[0],fweuler[1],fweuler[2],gimbalAng[2],gimbalAng[1],carpose_c[0]/carpose_c[2],carpose_c[1]/carpose_c[2],1/carpose_c[2]};
        
        DM trans = DM(X0);
        SX X_temp = SX(trans);
        //Eigen::Vector3f carpos_temp = Carpos;
        std::vector<float> cp_temp = {Carpos[0],Carpos[1],Carpos[2]};
        DM trans1 = DM(cp_temp);
        SX carpos_temp = SX(trans1);
        //Eigen::Vector3f carvel_temp = carVel;
        std::vector<float> cv_temp = {carVel[0],carVel[1],carVel[2]};
        DM trans2 = DM(cv_temp);
        SX carvel_temp = SX(trans2);
        std::vector<float> fv_temp = {fwVel[0],fwVel[1],fwVel[2]};
        float abs_v = sqrt(pow(fwVel[0],2)+pow(fwVel[1],2)+pow(fwVel[2],2));
        DM trans3 = DM(fv_temp);
        SX uavvel_temp = SX(fv_temp);
        std::vector<float> fa_temp = {fwAng[0],fwAng[1],fwAng[2]};
        DM trans4 = DM(fa_temp);
        SX uavang_temp = SX(fa_temp);
        DM trans5 = DM(nmpc_cmd);
        SX nmpc_pre = SX(trans5);
        SX u_log;
        SX g;
        std::vector<float> lbx,lbg,ubx,ubg;
        SX f=0;
        SX Uk = SX::sym("U" ,2);
        u_log=Uk;
        for(int i=0; i<N; i++){
            std::string order = std::to_string(i);
            SX R_flip = SX::eye(3);
            R_flip(1,1)=R_flip(2,2)=-1;
            SX R_g2b = mtimes(R_flip,mtimes(rotationMatrix_SX('X',X_temp(3)),mtimes(rotationMatrix_SX('Y',X_temp(4)),rotationMatrix_SX('Z',X_temp(5)))));
            SX R_b2p = rotationMatrix_SX('Z',X_temp(6));
            SX R_p2t = rotationMatrix_SX('Y',X_temp(7));
            SX R_t2c = mtimes(rotationMatrix_SX('X',M_PI/2),rotationMatrix_SX('Z',M_PI/2));
            SX R_g2c = mtimes(R_t2c,mtimes(R_p2t,mtimes(R_b2p,R_g2b)));

            //write xdot
            SX x1dot = uavvel_temp(0);
            SX x2dot = uavvel_temp(1);
            SX x3dot = uavvel_temp(2);
            // SX x4dot = uavang_temp(0);
            // SX x5dot = uavang_temp(1);
            // SX x6dot = uavang_temp(2);
            // SX x1dot = abs_v*cos(X_temp(5))*cos(X_temp(4));
            // SX x2dot = abs_v*sin(X_temp(5))*cos(X_temp(4));
            // SX x3dot = -abs_v*sin(X_temp(4));
            SX x4dot = nmpc_pre(1);
            SX x5dot = nmpc_pre(2);
            SX x6dot = -(gravity/abs_v)*tan(X_temp(3))*cos(X_temp(4));
            SX x7dot = Uk(0);
            SX x8dot = Uk(1);

            //target and uav motion
            // SX uav_velocity_g = vertcat(x1dot,x2dot,x3dot);
            // SX uav_angular_g = vertcat(x4dot,x5dot,x6dot);

            //all in camera frame
            SX uav_velocity_c = mtimes(R_g2c,uavvel_temp);
            SX uav_angular_c = mtimes(R_g2c,uavang_temp);
            SX target_velocity_c = mtimes(R_g2c,carvel_temp);
            SX camera_angular_c = mtimes(R_t2c,mtimes(R_p2t,vertcat(0,0,Uk(0))))+mtimes(R_t2c,vertcat(0,Uk(1),0));

            //xdot cal
            SX zeta1 = camera_angular_c(0)*X_temp(8)*X_temp(9)-camera_angular_c(1)-camera_angular_c(1)*pow(X_temp(8), 2)+camera_angular_c(2)*X_temp(9);
            SX zeta2 = camera_angular_c(0)+camera_angular_c(0)*pow(X_temp(9), 2)-camera_angular_c(1)*X_temp(8)*X_temp(9)-camera_angular_c(2)*X_temp(8);
            SX zeta3 = camera_angular_c(0)*X_temp(9)*X_temp(10)-camera_angular_c(1)*X_temp(8)*X_temp(10)-camera_angular_c(2)*X_temp(8);
            SX omega1 = uav_angular_c(0)*X_temp(8)*(X_temp(9)+0*X_temp(10))-uav_angular_c(1)-uav_angular_c(1)*pow(X_temp(8), 2)-uav_angular_c(1)*X_temp(10)*(0+0*X_temp(8))+uav_angular_c(2)*(X_temp(1)+0*X_temp(10));
            SX omega2 = uav_angular_c(0)+uav_angular_c(0)*X_temp(10)*(0+0*X_temp(9))+uav_angular_c(0)*pow(X_temp(9), 2)-uav_angular_c(1)*X_temp(9)*(X_temp(8)+0*X_temp(10))+uav_angular_c(2)*(-X_temp(8)-0*X_temp(10));
            SX omega3 = uav_angular_c(0)*(0*pow(X_temp(10), 2)+X_temp(9)*X_temp(10))+uav_angular_c(1)*(-0*pow(X_temp(10), 2)-X_temp(8)*X_temp(10));
            SX eta1 = (uav_velocity_c(2)*X_temp(8)-uav_velocity_c(0))*X_temp(10);
            SX eta2 = (uav_velocity_c(2)*X_temp(9)-uav_velocity_c(1))*X_temp(10);
            SX eta3 = uav_velocity_c(2)*pow(X_temp(10), 2);
            SX x9dot = target_velocity_c(0)*X_temp(10)-target_velocity_c(2)*X_temp(8)*X_temp(10)+zeta1+omega1+eta1;
            SX x10dot = target_velocity_c(1)*X_temp(10)-target_velocity_c(2)*X_temp(9)*X_temp(10)+zeta2+omega2+eta2;
            SX x11dot = -target_velocity_c(2)*pow(X_temp(10), 2)+zeta3+omega3+eta3;
            SX Xdot = vertcat(x1dot,x2dot,x3dot,x4dot,x5dot,x6dot);
            SX Xdot2 = vertcat(x7dot,x8dot,x9dot,x10dot,x11dot);
            SX xdot = vertcat(Xdot,Xdot2);

            //prediction update
            carpos_temp = carpos_temp + carvel_temp*dT;
            SX uavpos_temp = vertcat(X_temp(0),X_temp(1),X_temp(2));
            SX carpos_rel = carpos_temp-uavpos_temp;
            SX carpos_c_temp = mtimes(R_g2c,carpos_rel);

            // SX X_target_1 = vertcat(carpos_temp,X_temp(3),X_temp(4),X_temp(5));
            // SX X_target_2 = vertcat(X_temp(6),X_temp(7),0,0,X_temp(10));
            // SX X_target = vertcat(X_target_1,X_target_2);
            // X_target(2) = 40;
            SX X_target = vertcat(X_temp(6),X_temp(7),0,0,carpos_c_temp(2));
            X_temp = X_temp + xdot*dT;
            SX X_now = vertcat(X_temp(6),X_temp(7),X_temp(8),X_temp(9),X_temp(10));
            if(i==0){
                g = vertcat(X_temp(6),X_temp(7),X_temp(10));
                //g = X_temp(10);
            }
            else{
                g = vertcat(g,X_temp(6),X_temp(7),X_temp(10));
                //g = vertcat(g,X_temp(10));
            }

            //cost function
            SX W_now = SX::eye(5);
            W_now(0,0)=W_now(1,1)=5;
            W_now(2,2)=W_now(3,3)=10;
            W_now(4,4) = 0;
            SX err = X_target-X_now;
            f = f+mtimes(err.T(),mtimes(W_now,err));
        }
        // g = vertcat(g,X_temp(8),X_temp(9));
        SXDict nlp;
        nlp["x"] = u_log;
        nlp["f"] = f;
        nlp["g"] = g;

        Dict opts;
        //opts["ipopt.linear_solver"] = "mumps";
        opts["verbose_init"] = false;
        opts["verbose"] = false;
        opts["print_time"] = false;    

        std::map<std::string, DM> arg, res;
        std::vector<float> lbx_o,ubx_o,lbg_o,ubg_o,final_lbg,final_ubg;
        lbx = {-0.8,-0.8};
        ubx = {0.8,0.8};
        lbg_o = {-3.14,-1.57,0};
        ubg_o = {3.14,1.57,inf};
        // final_lbg = {-0.01,-0.01};
        // final_ubg = {0.01,0.01};
         for (int i = 0; i < N; ++i) {
        // lbx.insert(lbx.end(), lbx_o.begin(), lbx_o.end());
        // ubx.insert(ubx.end(), ubx_o.begin(), ubx_o.end());
        lbg.insert(lbg.end(), lbg_o.begin(), lbg_o.end());
        ubg.insert(ubg.end(), ubg_o.begin(), ubg_o.end());
        }
        arg["lbx"] = lbx;
        arg["ubx"] = ubx;
        arg["lbg"] = lbg;
        arg["ubg"] = ubg;
        arg["x0"]  = x0;
        

        Function solver = nlpsol("solver", "ipopt", nlp, opts);
        res = solver(arg);
        // std::cout<<res.at("x")<<std::endl;
        DM ans = res.at("x");
        std::cout<<ans(0)<<ans(1)<<std::endl;
        ans_cmd.data.resize(5);
        for(int j=0;j<2;j++){
            ans_cmd.data[j] = static_cast<float>(ans(j).scalar());
        }
        for(int j=0;j<2;j++){
            x0[j]=static_cast<float>(ans(j).scalar());
        }
        // nmpc_ans_pub.publish(ans_cmd);
        yawRate.data = x0[0];
        pitchRate.data = x0[1];
        yawVelCommand_pub.publish(yawRate);
        pitchVelCommand_pub.publish(pitchRate);
        // std::cout<<X0[8]<<std::endl<<X0[9]<<std::endl;
        //std::cout<<g<<std::endl;
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
    fwAng << odom->twist.twist.angular.x,odom->twist.twist.angular.y,odom->twist.twist.angular.z;

    
    quat_planeEarth_flu = Eigen::Quaternionf( odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z );
    fweuler = Quaternion2Euler(quat_planeEarth_flu);
    R_planeEarth_frd =  R_frd * Quat2RotaMatrix(quat_planeEarth_flu);
    
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

void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg){
    for(int i=0;i<3;i++){
        nmpc_cmd[i] = msg->data[i];
    }
}

SX rotationMatrix_SX(char axis, SX theta)
{
    SX R = SX::zeros(3,3);
    SX cos_theta = cos(theta);
    SX sin_theta = sin(theta);
    switch(axis)
    {
        case 'Z':
            R(0,0) = cos_theta;
            R(0,1) = sin_theta;
            R(1,0) = -sin_theta;
            R(1,1) = cos_theta;
            R(2,2) = 1;
            break;
        case 'Y':
            R(0,0) = cos_theta;
            R(0,2) = -sin_theta;
            R(1,1) = 1;
            R(2,0) = sin_theta;
            R(2,2) = cos_theta;
            break;
        case 'X':
            R(0,0) = 1;
            R(1,1) = cos_theta;
            R(1,2) = sin_theta;
            R(2,1) = -sin_theta;
            R(2,2) = cos_theta;
            break;
        default:
            break; 
    }
    
    return R;
}