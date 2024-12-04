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
std_msgs::Float32 draw_num;
Eigen::Quaternionf quat_planeEarth_flu;
Eigen::Matrix3f R_planeEarth_frd;
Eigen::Matrix3f R_frd;
Eigen::Matrix3f R_enu;
Eigen::Matrix3f R_panPlane;
Eigen::Matrix3f R_camPan;
Eigen::Vector3f Carpos;  
Eigen::Vector3f Rel_carpos;
Eigen::Vector3f carVel;
Eigen::Vector3f fwVel;
Eigen::Vector3f fweuler;  
std::vector<float> gimbalAng{0.0, 0.0, 0.0};
std::vector<float> gimbalAngVel{0.0, 0.0, 0.0}; 
ros::Subscriber car_odom_sub;
ros::Subscriber fw_pose_sub;
ros::Subscriber gimbal_sub;
ros::Publisher nmpc_ans_pub;
ros::Publisher draw_pub;
int N;                                                               //prediction horizon
int C;                                                               //control horizon
float dT;
float gravity=9.81;
void getFwPose(const nav_msgs::Odometry::ConstPtr& odom);
void getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom);
void getGimbalState(const sensor_msgs::JointState::ConstPtr& state);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_control_nogimbal");
    ros::NodeHandle nh;
    ros::Rate rate = 10;
    car_odom_sub = nh.subscribe("/wamv/base_pose_ground_truth", 10, getAgentOdom);
    fw_pose_sub = nh.subscribe("/uav0/base_pose_ground_truth", 10, getFwPose);
    nmpc_ans_pub = nh.advertise<std_msgs::Float32MultiArray>("/nmpc_ans",10);
    draw_pub = nh.advertise<std_msgs::Float32>("/draw_usage",10);
    std::vector<float> x0;
    x0  = {20,0,0,20,0,0,20,0,0,20,0,0,20,0,0,20,0,0,20,0,0,20,0,0,20,0,0,20,0,0};

    while(ros::ok()){
        // start building nmpc //
        // std::cout<<"Hi"<<std::endl;
        N = 50;
        C = 10;
        dT = 0.1;
        SX W = SX::eye(6);
        W(1,1) = 0.1;
        W(2,2) = 0.05;
        W(3,3) = 10;
        W(4,4) = 1000;
        SX W2 = SX::eye(3);
        W2(1,1)=W2(2,2)=10;
        R_enu << 1,0,0,0,-1,0,0,0,-1;
        std::vector<float> X0 = {fwPos.position.x,fwPos.position.y,fwPos.position.z,fweuler[0],fweuler[1],fweuler[2]};
        DM trans = DM(X0);
        SX X_temp = SX(trans);
        //Eigen::Vector3f carpos_temp = Carpos;
        std::vector<float> cp_temp = {Carpos[0],Carpos[1],Carpos[2]};
        DM trans1 = DM(cp_temp);
        SX carpos_temp = SX(trans1);
        float abs_v = sqrt(pow(fwVel[0],2)+pow(fwVel[1],2)+pow(fwVel[2],2));
        std::vector<float> rel_pointer = {cp_temp[0]-X0[0],cp_temp[1]-X0[1],cp_temp[2]-X0[2]};
        float abs_rel = sqrt(pow(rel_pointer[0],2)+pow(rel_pointer[1],2)+pow(rel_pointer[2],2));
        float angle_dot = fwVel[0]*rel_pointer[0]+fwVel[1]*rel_pointer[1]+fwVel[2]*rel_pointer[2];
        float head_angle = acos(angle_dot/(abs_v*abs_rel));
        float dis_show = sqrt(pow(X0[0]-cp_temp[0],2)+pow(X0[1]-cp_temp[1],2));
        //Eigen::Vector3f carvel_temp = carVel;
        std::vector<float> cv_temp = {carVel[0],carVel[1],carVel[2]};
        DM trans2 = DM(cv_temp);
        SX carvel_temp = SX(trans2);
        SX u_log;
        SX g;
        std::vector<float> lbx,lbg,ubx,ubg;
        SX f=0;
        int divder = N/C;
        int control_order = 0;
        // std::cout<<divder<<std::endl<<control_order<<std::endl;

        // A section for calculating limited angle
        float cal_car_vel, cal_fw_vel, cal_xy_dis;
        Eigen::Matrix3f rot_yaw = rotationMatrix('Z',-fweuler[2]);
        Eigen::Vector3f rel_pose;
        rel_pose << carPos.position.x-fwPos.position.x, carPos.position.y-fwPos.position.y, carPos.position.z-fwPos.position.z;
        cal_xy_dis = sqrt(rel_pose[0]*rel_pose[0]+rel_pose[1]*rel_pose[1]);
        cal_fw_vel = sqrt(fwVel[0]*fwVel[0]+fwVel[1]*fwVel[1]+fwVel[2]*fwVel[2]);
        Eigen::Vector3f yaw_rel = rot_yaw*R_enu*rel_pose;
        float limit_angle = atan2(yaw_rel[2],yaw_rel[1]);
        if(limit_angle>1.57){
            limit_angle = limit_angle - 3.14;
        }


        // construct NMPC from loop
        SX Uk;
        for(int i=0; i<N; i++){
            if(i==divder*control_order){
                std::string order = std::to_string(control_order);
                Uk = SX::sym("U"+order,3);
                if(i==0){
                    u_log=Uk;
                }
                else{
                    u_log=vertcat(u_log,Uk);
                }
                control_order++;
            }

            //write xdot
            // std::cout<<Uk<<std::endl;
            
            SX x1dot = Uk(0)*cos(X_temp(5))*cos(X_temp(4));
            SX x2dot = Uk(0)*sin(X_temp(5))*cos(X_temp(4));
            SX x3dot = -Uk(0)*sin(X_temp(4));
            SX x4dot = Uk(1);
            SX x5dot = Uk(2);
            SX x6dot = -(gravity/Uk(0))*tan(X_temp(3))*cos(X_temp(4));
            // SX x4dot = Uk(1)+Uk(2)*sin(X_temp(3))*tan(X_temp(4));
            // SX x5dot = Uk(2)*cos(X_temp(3));
            // SX x6dot = Uk(2)*sin(X_temp(3))/cos(X_temp(4));
            // SX R_NED_B = SX::zeros(3, 3);
            // R_NED_B(0, 0) = cos(X_temp(4)) * cos(X_temp(5));
            // R_NED_B(0, 1) = cos(X_temp(4)) * sin(X_temp(5));
            // R_NED_B(0, 2) = -sin(X_temp(4));
            // R_NED_B(1, 0) = sin(X_temp(3)) * sin(X_temp(4)) * cos(X_temp(5)) - cos(X_temp(3)) * sin(X_temp(5));
            // R_NED_B(1, 1) = sin(X_temp(3)) * sin(X_temp(4)) * sin(X_temp(5)) + cos(X_temp(3)) * cos(X_temp(5));
            // R_NED_B(1, 2) = sin(X_temp(3)) * cos(X_temp(4));
            // R_NED_B(2, 0) = cos(X_temp(3)) * sin(X_temp(4)) * cos(X_temp(5)) + sin(X_temp(3)) * sin(X_temp(5));
            // R_NED_B(2, 1) = cos(X_temp(3)) * sin(X_temp(4)) * sin(X_temp(5)) - sin(X_temp(3)) * cos(X_temp(5));
            // R_NED_B(2, 2) = cos(X_temp(3)) * cos(X_temp(4));

            // SX R_E_B = SX::zeros(3, 3);
            // R_E_B(0, 0) = 1;
            // R_E_B(0, 1) = sin(X_temp(3)) * tan(X_temp(4));
            // R_E_B(0, 2) = cos(X_temp(3)) * tan(X_temp(4));
            // R_E_B(1, 0) = 0;
            // R_E_B(1, 1) = cos(X_temp(3));
            // R_E_B(1, 2) = -sin(X_temp(3));
            // R_E_B(2, 0) = 0;
            // R_E_B(2, 1) = sin(X_temp(3)) / cos(X_temp(4)); // sin(phi) * sec(theta) is sin(phi) / cos(theta)
            // R_E_B(2, 2) = cos(X_temp(3)) / cos(X_temp(4));

            // SX input_command = vertcat(Uk(1),Uk(2),0);
            // SX input_elur = mtimes(R_E_B,input_command);
                
            SX xdot = vertcat(x1dot,x2dot,x3dot,x4dot,x5dot,x6dot);
            // SX xdot = vertcat(x1dot,x2dot,x3dot,input_elur);
            

            //target and uav motion
            SX uav_velocity_g = vertcat(x1dot,x2dot,x3dot);
            SX uav_angular_g = vertcat(x4dot,x5dot,x6dot);

            //prediction update
            carpos_temp = carpos_temp + carvel_temp*dT;
            SX X_target = vertcat(carpos_temp,X_temp(3),X_temp(4),X_temp(5));
            // SX X_target = vertcat(carpos_temp,X_temp(3),0,X_temp(5));
            // X_target(0) = X_target(0)+10;
            X_target(2) = 300;
            X_temp = X_temp + xdot*dT;

            //bounding angle calculation
            // SX heading_vector = vertcat(x1dot,x2dot,x3dot);
            // SX ref_vector = carpos_temp;
            // ref_vector(0) = ref_vector(0)-X_temp(0);
            // ref_vector(1) = ref_vector(1)-X_temp(1);
            // ref_vector(2) = ref_vector(2)-X_temp(2);
            // SX vector_dot = mtimes(heading_vector.T(),ref_vector);
            // SX length_heading = sqrt(mtimes(heading_vector.T(),heading_vector));
            // SX length_ref = sqrt(mtimes(ref_vector.T(),ref_vector));
            // SX bounding_angle = acos(vector_dot/(length_heading*length_ref));
            // if(i==0){
            //     g = vertcat(X_temp(3),X_temp(4),bounding_angle);
            // }
            // else{
            //     g = vertcat(g,X_temp(3),X_temp(4),bounding_angle);
            // }

            SX xy_dis = sqrt(pow(X_temp(0)-carpos_temp(0),2)+pow(X_temp(1)-carpos_temp(1),2));
            if(i==0){
                g = vertcat(X_temp(3),X_temp(4),xy_dis);
            }
            else{
                g = vertcat(g,X_temp(3),X_temp(4),xy_dis);
            }

            //cost function
            // SX err = X_target-X_temp;
            SX X_recost = X_temp;
            X_recost(0) = xy_dis;
            X_recost(1) = 0;
            X_target(0) = X0[2]*1.8;
            X_target(1) = 0;
            SX err = X_target-X_recost;
            f = f+mtimes(err.T(),mtimes(W,err));
            // std::cout<<f<<std::endl;
        }
        // g = vertcat(g,x0[1]-u_log(1),x0[2]-u_log(2));
        // //smoothlize
        // for (int smooth=0;smooth<N-1;smooth++){
        //     SX smo = vertcat(u_log(smooth*3)-u_log((smooth+1)*3),u_log(smooth*3+1)-u_log((smooth+1)*3+1),u_log(smooth*3+2)-u_log((smooth+1)*3+2));
        //     f = f+mtimes(smo.T(),mtimes(W2,smo));
        // }
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
        std::vector<float> lbx_o,ubx_o,lbg_o,ubg_o,lbg_dot,ubg_dot;
        lbx_o = {25.0,-0.8,-0.8};
        ubx_o = {30.0,0.8,0.8};
        lbg_o = {-0.78,-0.17,-inf};
        ubg_o = {0.78,0.17,inf};
        if(limit_angle>=0 && limit_angle<=0.78){
            ubg_o[0] = limit_angle;
        }
        if(limit_angle<0 && limit_angle>=-0.78){
            lbg_o[0] = limit_angle;
        }
        // lbg_dot = {-1,-0.2,-0.2};
        // ubg_dot = {1,0.2,0.2};
        for (int i = 0; i < C; ++i) {
            lbx.insert(lbx.end(), lbx_o.begin(), lbx_o.end());
            ubx.insert(ubx.end(), ubx_o.begin(), ubx_o.end());
        }
        for (int i = 0; i < N; ++i) {
            lbg.insert(lbg.end(), lbg_o.begin(), lbg_o.end());
            ubg.insert(ubg.end(), ubg_o.begin(), ubg_o.end());
        }
        // for (int i = 0; i < N; ++i) {
        //     lbg.insert(lbg.end(), lbg_dot.begin(), lbg_dot.end());
        //     ubg.insert(ubg.end(), ubg_dot.begin(), ubg_dot.end());
        // }
        arg["lbx"] = lbx;
        arg["ubx"] = ubx;
        arg["lbg"] = lbg;
        arg["ubg"] = ubg;
        arg["x0"]  = x0;
        

        Function solver = nlpsol("solver", "ipopt", nlp, opts);
        res = solver(arg);
        std::cout<<res.at("x")<<std::endl;
        DM ans = res.at("x");
        std::cout<<ans(0)<<ans(1)<<ans(2)<<std::endl;
        ans_cmd.data.resize(3);
        for(int j=0;j<3;j++){
            ans_cmd.data[j] = static_cast<float>(ans(j).scalar());
        }
        for(int j=0;j<(3*C);j++){
            x0[j]=static_cast<float>(ans(j).scalar());
        }
        draw_num.data = static_cast<float>(ans(2).scalar());
        nmpc_ans_pub.publish(ans_cmd);
        draw_pub.publish(draw_num);
        //std::cout<<g<<std::endl;
        ROS_INFO("The distance is %f", dis_show);
        ROS_INFO("The relative angle is %f", limit_angle);
        ROS_INFO("The angle limit %f", limit_angle*180/3.14);
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