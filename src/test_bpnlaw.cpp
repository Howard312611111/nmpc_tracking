//////////
// main 
//////////

#include <iostream>

#include <cassert>
#include <stdlib.h>
#include <UAVState.h>

using namespace std;

// test the BPNGlaw_3D function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_bpnlaw");
    UAVState uav;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10);
    
    double xx4 = -100., yy4 = 100., zz4 = 120., dis_wayp4;
    double ang_azi, ang_ele;
    int s = 1;

    // initial condition
    // cout << "The initial point is (" << xx4 << ", " << yy4 << ", " << zz4 << ")" << endl;
    // dis_wayp4 = uav.get_dis_waypoint(xx4, yy4, zz4);
    // cout << "The distance between UAV and initial point is " << dis_wayp4 << " m" << endl;
    uav.showPosVel();
    int buffer = 0;
    while(ros::ok()) {
        uav.vel_cmd(1., 0., 0.);
        rate.sleep();
        uav.showPosVel();
        // dis_wayp4 = uav.get_dis_waypoint(xx4, yy4, zz4);
        // cout << "The distance between UAV and initial point is " << dis_wayp4 << "m\n" << endl;

        if (buffer > 30){
            break;
        }
        buffer++;
        ros::spinOnce();
    }

    // BPNG part
    while(ros::ok()) {
        uav.showPosVel();
        //uav.test_raw_attitude(2., 2., 0.);
        // uav.acc_cmd_test(0.,0.,-4.);
        uav.bpng_acc_cmd(-30, -30);

        // std::cout << "s = " << s << std::endl; 
        // if (s == 1) {
        //     uav.bpng_acc_ini_cmd(-120, 120);
        // } else {
        //     uav.bpng_acc_cmd(-120, 120);
        // }
        // s = s+1;

        rate.sleep();
        ros::spinOnce();
    }

    return(0);
}








