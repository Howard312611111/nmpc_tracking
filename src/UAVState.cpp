#include <iostream>
#include <iomanip>
#include <math.h>
#include <cmath>
#include <cassert>
#include "UAVState.h"
#define pi acos(-1)

double N = 4.;
double g_uav = 9.8; // g of uav
double g_tar = 0.;  // g of target on the ground
double etat = 3.;
double etap = 0.025;
double mass_UC = 2.065; // mass of uav and camera (kg)
double thr_max = 10.325;  // max thrust (N)
double delta_t = 0.1;


UAVState::UAVState()
{
	car_odom_sub = nhdl.subscribe<nav_msgs::Odometry>
            ("wamv/base_pose_ground_truth", 10, &UAVState::getAgentOdom, this);

	// uav_pose_sub = nhdl.subscribe<geometry_msgs::PoseStamped>
	// 		("mavros/local_position/pose", 1, &UAVState::getUavPos, this);
	// uav_velocity_body_sub = nhdl.subscribe<geometry_msgs::TwistStamped>
	// 		("mavros/local_position/velocity_local", 1, &UAVState::getUavVel, this);
	// uav_cmd_vel_pub = nhdl.advertise<geometry_msgs::TwistStamped>
	// 		("mavros/setpoint_velocity/cmd_vel", 10);
	// uav_cmd_unstamped_pub = nhdl.advertise<geometry_msgs::Twist>
	// 		("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	// uav_cmd_waypint_pub = nhdl.advertise<geometry_msgs::PoseStamped>
	// 		("mavros/setpoint_position/local", 10);
	// uav_cmd_acc_pub = nhdl.advertise<geometry_msgs::Vector3Stamped>
	// 		("mavros/setpoint_accel/accel", 10);
	// uav_cmd_att_thr_pub = nhdl.advertise<mavros_msgs::AttitudeTarget>
	// 		("mavros/setpoint_raw/attitude", 10);
	uav_pose_sub = nhdl.subscribe<geometry_msgs::PoseStamped>
			("uav0/mavros/local_position/pose", 1, &UAVState::getUavPos, this);
	uav_velocity_body_sub = nhdl.subscribe<geometry_msgs::TwistStamped>
			("uav0/mavros/local_position/velocity_local", 1, &UAVState::getUavVel, this);
	uav_cmd_vel_pub = nhdl.advertise<geometry_msgs::TwistStamped>
			("uav0/mavros/setpoint_velocity/cmd_vel", 10);
	uav_cmd_unstamped_pub = nhdl.advertise<geometry_msgs::Twist>
			("uav0/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	uav_cmd_waypint_pub = nhdl.advertise<geometry_msgs::PoseStamped>
			("uav0/mavros/setpoint_position/local", 10);
	uav_cmd_acc_pub = nhdl.advertise<geometry_msgs::Vector3Stamped>
			("uav0/mavros/setpoint_accel/accel", 10);
	uav_cmd_att_thr_pub = nhdl.advertise<mavros_msgs::AttitudeTarget>
			("uav0/mavros/setpoint_raw/attitude", 10);

}

void UAVState::getAgentOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    carVel << odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z;

    carPos.position.x = odom->pose.pose.position.x;
    carPos.position.y = odom->pose.pose.position.y;
    carPos.position.z = odom->pose.pose.position.z;
}

void UAVState::getUavPos(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	uavPos.position.x = pos->pose.position.x;
	uavPos.position.y = pos->pose.position.y;
	uavPos.position.z = pos->pose.position.z;

	uavPos.orientation.x = pos->pose.orientation.x;
	uavPos.orientation.y = pos->pose.orientation.y;
	uavPos.orientation.z = pos->pose.orientation.z;
	uavPos.orientation.w = pos->pose.orientation.w;

	q_x = uavPos.orientation.x;
	q_y = uavPos.orientation.y;
	q_z = uavPos.orientation.z;
	q_w = uavPos.orientation.w;

	double sinr_cosp = 2. * (q_w * q_x + q_y * q_z);
    double cosr_cosp = 1. - 2. * (q_x * q_x + q_y * q_y);
	phi_ang = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = std::sqrt(1. + 2. * (q_w * q_y - q_x * q_z));
    double cosp = std::sqrt(1. - 2. * (q_w * q_y - q_x * q_z));
	theta_ang = 2. * std::atan2(sinp, cosp) - pi / 2.;

	double siny_cosp = 2. * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1. - 2. * (q_y * q_y + q_z * q_z);
	psi_ang = std::atan2(siny_cosp, cosy_cosp);

	// difference of uav's position initial condition between Qground and gazebo
	x = uavPos.position.x - 500.;
	y = uavPos.position.y + 100.;
	z = uavPos.position.z + 51.;
}

void UAVState::getUavVel(const geometry_msgs::TwistStamped::ConstPtr& vel)
{
	uavLinearVel << vel->twist.linear.x, vel->twist.linear.y, vel->twist.linear.z;
	uavAngularVel << vel->twist.angular.x, vel->twist.angular.y, vel->twist.angular.z;

	vx = uavLinearVel[0];
	vy = uavLinearVel[1];
	vz = uavLinearVel[2];
}

void UAVState::showPosVel()
{
	std::cout << "The position of UAV is (" << x << ", " << y << ", " << z << ")" << std::endl;
	std::cout << "The velocity of UAV is (" << vx << ", " << vy << ", " << vz << ") = " << sqrt(vx*vx+vy*vy+vz*vz) << "m/s" << std::endl;
	// std::cout << "The vel_thm is (" << vxm_thm << ", " << vym_thm << ", " << vzm_thm << ")" << std::endl;
}

double UAVState::get_dis_waypoint(double x_wp, double y_wp, double z_wp)
{
	double xd, yd, zd, distance;

	xd = x - x_wp;
	yd = y - y_wp;
	zd = z - z_wp;

	distance = sqrt(xd*xd + yd*yd + zd*zd);

	return (distance);
}

void UAVState::waypoint_cmd(double x_wp, double y_wp, double z_wp)
{
	uavWaypointCmd.pose.position.x = x_wp;
	uavWaypointCmd.pose.position.y = y_wp;
	uavWaypointCmd.pose.position.z = z_wp;

	uav_cmd_waypint_pub.publish(uavWaypointCmd);
}

void UAVState::vel_cmd(double vx, double vy, double vz)
{
	uavLinearVelCmd.twist.linear.x = vx;
	uavLinearVelCmd.twist.linear.y = vy;
	uavLinearVelCmd.twist.linear.z = vz;

	uav_cmd_vel_pub.publish(uavLinearVelCmd);
	std::cout << "The vel_cmd is (" << uavLinearVelCmd.twist.linear.x << ", " << uavLinearVelCmd.twist.linear.y << ", " << uavLinearVelCmd.twist.linear.z << ")" << std::endl;

	ros::spinOnce();
    rate.sleep();
}

void UAVState::test_raw_attitude(double vxm, double vym, double vzm)
{
	double vx_next, vy_next, vz_next, v_next;
	double unit_vx_next, unit_vy_next, unit_vz_next;
	double q_w, q_x, q_y, q_z, thrust;

	// normalize the velocity_next
	v_next = sqrt(vxm*vxm + vym*vym + vzm*vzm);
	unit_vx_next = vxm / v_next;
	unit_vy_next = vym / v_next;
	unit_vz_next = vzm / v_next;

	// generate attitude of uav in quaternion
	q_w = 0.;
	q_x = unit_vx_next;
	q_y = unit_vy_next;
	q_z = unit_vz_next;

	std::cout << "The attitude_cmd in quaternion is (" << q_w << ", " << q_x << ", " << q_y << ", " << q_z << ")\n" << std::endl;

	// generate thrust
	thrust = 0.1;

	uavAttitudeThrustCmd.type_mask = 7;                                     
	uavAttitudeThrustCmd.orientation.x = q_x;
	uavAttitudeThrustCmd.orientation.y = q_y;
	uavAttitudeThrustCmd.orientation.z = q_z;
	uavAttitudeThrustCmd.orientation.w = q_w;
	// uavAttitudeThrustCmd.body_rate.x = roll_ang;
	// uavAttitudeThrustCmd.body_rate.x = pitch_ang;
	// uavAttitudeThrustCmd.body_rate.x = yaw_ang;
	uavAttitudeThrustCmd.thrust = thrust;

	uav_cmd_att_thr_pub.publish(uavAttitudeThrustCmd);
}

void UAVState::acc_cmd_test(double acc_x, double acc_y, double acc_z)
{
	double acc_sum;
	double xm_next, ym_next, zm_next, vx_next, vy_next, vz_next, v_next;

	// print the acc cmd
	acc_sum = sqrt(acc_x*acc_x+acc_y*acc_y+acc_z*acc_z);
	std::cout << "The acc_cmd in gazeboframe is (" << acc_x << ", " << acc_y << ", " << acc_z << ") = " << acc_sum << " m/s^2" << std::endl;

	xm_next = x + vx*delta_t + 0.5*acc_x*delta_t*delta_t;
	ym_next = y + vy*delta_t + 0.5*acc_y*delta_t*delta_t;
	zm_next = z + vz*delta_t + 0.5*acc_z*delta_t*delta_t;

	// publish position command
	uavWaypointCmd.pose.position.x = xm_next;
	uavWaypointCmd.pose.position.y = ym_next;
	uavWaypointCmd.pose.position.z = zm_next;

	std::cout << "The waypoint cmd is (" << uavWaypointCmd.pose.position.x << ", " << uavWaypointCmd.pose.position.y << ", " << uavWaypointCmd.pose.position.z << ")\n" << std::endl;
	uav_cmd_waypint_pub.publish(uavWaypointCmd);

}

void UAVState::bpng_azi_acc_cmd(double vari)
{
	double xm = y, ym = x, zm = z; // xm,ym,and zm are for bpnlaw in matlab
	double vxm = vy, vym = vx, vzm = vz; // vxm, vym, and vzm are for bpnlaw in matlab
	double xt = carPos.position.y, yt = carPos.position.x, zt = carPos.position.z; // xt, yt, and zt are for bpnlaw in matlab 
	double vxt = carVel[1], vyt = carVel[0], vzt = carVel[2]; // vxt, vyt, and vzt are for bpnlaw in matlab 
	double rx, ry, vrex, vrey, r;
	double r1, r2, v1, v2;
	double LOS, gammat, gamma;
	double thet, them, v, vt;
	double vcl, tgo;
	double predict_vt, predict_gammat, predict_v;
	double dLOS, desireLOS, dbias;
	double Ac, Acl, amxc, amyc, amzc, Ac_sum;
	double xm_next, ym_next, zm_next, vx_next, vy_next, v_next, unit_vx_next, unit_vy_next, thetaa, q_w, q_x, q_y, q_z;

	std::cout << "The position of the wamv is (" << carPos.position.x << ", " << carPos.position.y << ", " << carPos.position.z << ")" << std::endl;
	std::cout << "The velocity of the wamv is (" << carVel[0] << ", " << carVel[1] << ", " << carVel[2] << ")" << std::endl;

	// relative position and velocity
	rx = xt - xm; 
	ry = yt - ym;
	vrex = vxt - vxm; 
	vrey = vyt - vym;
	r = sqrt(rx * rx + ry * ry);
	if (r <= 0.01) {
		r = 0.01;
	}

	// change of coordinates
	r1=ry;
	r2=rx;
	v1=vrey;
	v2=vrex;
	std::cout << "The vrex and vrey in matlab are (" << v2 << ", " << v1 << ")" << std::endl; 

	LOS = atan(ry / rx); // azimuth angle of line of sight

	if (vxt >= 0 && vyt >= 0) {
		gammat = atan(vyt / vxt);
	}
	else if (vxt <= 0 && vyt >= 0) {
		gammat = pi + atan(vyt / vxt);
	}
	else if (vxt <= 0 && vyt <= 0) {
		gammat = -pi + atan(vyt / vxt);
	}
	else if (vxt >= 0 && vyt <= 0) {
		gammat = atan(vyt / vxt);
	}

	if (vx >= 0 && vym >= 0) {
		gamma = atan(vym / vxm);
	}
	else if (vxm <= 0 && vym >= 0) {
		gamma = pi + atan(vym / vxm);
	}
	else if (vxm <= 0 && vym <= 0) {
		gamma = -pi + atan(vym / vxm);
	}
	else if (vxm >= 0 && vym <= 0) {
		gamma = atan(vym / vxm);
	}

	thet=gammat-LOS;
	them=gamma-LOS;

	v=sqrt(vxm*vxm+vym*vym);
	vt=sqrt(vxt*vxt+vyt*vyt);

	// closing velocity
	vcl = -(r1 * v1 + r2 * v2) / r;

	// time to go
	tgo = r / vcl;

	// consider gravity
	// predict_vt = abs(vt-tgo*0.*sin(gammat));
	// predict_gammat = gammat-tgo*0.*cos(gammat)/predict_vt;
	// predict_v = abs(v-tgo*0.*sin(gamma));

	// PN law
	dLOS=(vt*sin(thet)-v*sin(them))/r;
	desireLOS=atan((vt*sin(gammat)-v*sin(gammat-vari*pi/180.))/(vt*cos(gammat)-v*cos(gammat-vari*pi/180.)));

	dbias=((etat*vcl*(desireLOS-LOS))/(N*r));

	if (r >= 10.) {
		// 二版
		Ac=N*vcl*dLOS-etap*r*(desireLOS-LOS);
		// 原版
		// Ac=N*vcl*(dLOS-dbias);

		Acl = Ac;

		amxc=-Ac*sin(LOS);
    	amyc=Ac*cos(LOS);  
    	amzc=0.;
	}
	else {
		amxc=-Acl*sin(LOS);
    	amyc=Acl*cos(LOS);
		amzc=0.;
		Ac = Acl;
	}

	Ac_sum = sqrt(amxc*amxc+amyc*amyc);
	std::cout << "The Ac_cmd in gazebo frame is (" << amyc << ", " << amxc << ", " << amzc << ") = " << Ac_sum << " m/s^2" << std::endl;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// calculate the position
	xm_next = xm + vxm*delta_t + 0.5*amxc*delta_t*delta_t;
	ym_next = ym + vym*delta_t + 0.5*amyc*delta_t*delta_t;
	zm_next = zm;

	// publish position command
	uavWaypointCmd.pose.position.x = ym_next; // change to gazebo coordinates
	uavWaypointCmd.pose.position.y = xm_next; // change to gazebo coordinates
	uavWaypointCmd.pose.position.z = zm_next;

	std::cout << "The waypoint cmd is (" << uavWaypointCmd.pose.position.x << ", " << uavWaypointCmd.pose.position.y << ", " << uavWaypointCmd.pose.position.z << ")\n" << std::endl;

	uav_cmd_waypint_pub.publish(uavWaypointCmd);
}

void UAVState::bpng_acc_cmd(double varia, double varie)
{
	// For other points except initial condition
	double xm = y, ym = x, zm = z; // xm,ym,and zm are for bpnlaw in matlab
	double vxm = vy, vym = vx, vzm = vz; // vxm, vym, and vzm are for bpnlaw in matlab
	double xt = carPos.position.y, yt = carPos.position.x, zt = carPos.position.z; // xt, yt, and zt are for bpnlaw in matlab 
	double vxt = carVel[1], vyt = carVel[0], vzt = carVel[2]; // vxt, vyt, and vzt are for bpnlaw in matlab
	double rx, ry, rz, vrex,vrey, vrez, rxy, r; // x is the x vector of UAV position, and xt is the x vector of target
	double r1, r2, r3, v1, v2, v3;
	double vclxy,vclxy_r, vcl, vcl_r, tgo;
	double aLOS, agammat, agamma, athet, athe, av, avt, adLOS, adesireLOS, adbias;
	double vl, vl_r, rl, vtl, vtl_r;
	double eLOS, egammat, egamma, ethet, ethe, ev, ev_r, evt, evt_r, edLOS, edesireLOS, edbias;
	double predict_evt, predict_egammat, predict_ev;
	double Aca, Ace, Acla, Acle, axc, ayc, azc;
	double acc_h_x, acc_h_y, acc_h_z;
	double roll_ang, pitch_ang, yaw_ang;
	double q_w, q_x, q_y, q_z;
	double thrust, thr;
	double acc_sum, acc_h_sum;
	double axc_enu, ayc_enu, azc_enu;
	double xm_next, ym_next, zm_next, vx_next, vy_next, vz_next, v_next, unit_vx_next, unit_vy_next, unit_vz_next, thetaa;
	double roll_rate, pitch_rate, yaw_rate, v_total;

	std::cout << "The position of the boat is (" << (carPos.position.x - 25.)<< ", " << carPos.position.y << ", " << (carPos.position.z + 18.) << ")" << std::endl;
	std::cout << "The velocity of the boat is (" << carVel[0] << ", " << carVel[1] << ", " << carVel[2] << ")" << std::endl;

	// relative position and velocity
	rx = xt - xm; 
	ry = yt - ym;
	rz = zt - zm;
	vrex = vxt - vxm; 
	vrey = vyt - vym;
	vrez = vzt - vzm;
	rxy = sqrt(rx * rx + ry * ry);
	r = sqrt(rx * rx + ry * ry + rz * rz);

	if (r <= 0.01) {
		r = 0.01;
	}
	if (rxy <= 0.01) {
		rxy = 0.01;
	}

	// change of coordinates
	r1 = ry;
	r2 = rx;
	r3 = -rz;
	v1 = vrey;
	v2 = vrex;
	v3 = -vrez;

	// closing velocity
	vclxy = -(r1 * v1 + r2 * v2) / rxy;
	vcl = -(r1 * v1 + r2 * v2 + r3 * v3) / r;

	// time to go
	tgo = r / vcl;

	// azimuth part

	aLOS = atan2(ry, rx); // azimuth line of sight
	agammat = atan2(vyt, vxt);
	agamma = atan2(vym, vxm);

	// if (vxt >= 0 && vyt >= 0) {
	// 	agammat = atan(vyt / vxt);
	// }
	// else if (vxt <= 0 && vyt >= 0) {
	// 	agammat = pi + atan(vyt / vxt);
	// }
	// else if (vxt <= 0 && vyt <= 0) {
	// 	agammat = -pi + atan(vyt / vxt);
	// }
	// else if (vxt >= 0 && vyt <= 0) {
	// 	agammat = atan(vyt / vxt);
	// }

	// if (vxm >= 0 && vym >= 0) {
	// 	agamma = atan(vym / vxm);
	// }
	// else if (vxm <= 0 && vym >= 0) {
	// 	agamma = pi + atan(vym / vxm);
	// }
	// else if (vxm <= 0 && vym <= 0) {
	// 	agamma = -pi + atan(vym / vxm);
	// }
	// else if (vxm >= 0 && vym <= 0) {
	// 	agamma = atan(vym / vxm);
	// }

	// azimuth angle theta of target and UAV
	athet = agammat - aLOS;
	athe = agamma - aLOS;

	// azimuth velocity of UAV and target
	av = sqrt(vxm * vxm + vym * vym);
	avt = sqrt(vxt * vxt + vyt * vyt);

	adLOS = (avt * sin(athet) - av * sin(athe)) / rxy;
	if (athet>pi || athet<0) {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	} else {
		adesireLOS = atan((avt * sin(agammat) - av * sin(agammat + varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat + varia * pi / 180.)));
	}
	// adesireLOS = atan((avt * sin(agammat) - av * sin(agammat - varia * pi / 180.)) / (avt * cos(agammat) - av * cos(agammat - varia * pi / 180.)));
	// std::cout << "The adesireLOS is " << adesireLOS * 180. / pi << std::endl;
	adbias = ((etat * vclxy * (adesireLOS - aLOS)) / (N * rxy));

	// elevation part
	// vl = sqrt((vxm * vxm) + (vym * vym));
	vl_r = av*cos(athe); // 投影到R垂直面
	// rl = sqrt((rx * rx) + (ry * ry));
	// vtl = -sqrt((vxt * vxt) + (vyt * vyt));
	vtl_r = avt*cos(athet); // 投影到R垂直面

	eLOS = atan2(rz, rxy);
	egamma = atan2(vzm, vl_r);
	egammat = atan2(vzt, vtl_r);
	ethet = egammat - eLOS;
	ethe = egamma - eLOS;

	vclxy_r = sqrt(vxm*vxm + vym*vym) * cos(agamma);
	vcl_r = sqrt(vxm*vxm + vym*vym + vzm*vzm) * cos(agamma);

	ev = sqrt(vzm*vzm + av*av);
	ev_r = sqrt(vzm*vzm + vl_r*vl_r);
	evt_r = sqrt(vzt*vzt + vtl_r*vtl_r);

	edLOS = (evt_r * sin(ethet) - ev_r * sin(ethe)) / r;

	// consider gravity
	predict_evt = abs(evt_r - tgo * g_tar * sin(egammat));
	predict_egammat = egammat - tgo * g_tar * cos(egammat) / predict_evt;
	predict_ev = abs(ev_r - tgo * g_uav * sin(egamma));

	if (ethet>pi || ethet<0) {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat + varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat + varie * pi / 180.)));
	} else {
		edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));
	}
	// edesireLOS = atan((predict_evt * sin(predict_egammat) - predict_ev * sin(predict_egammat - varie * pi / 180.)) / (predict_evt * cos(predict_egammat) - predict_ev * cos(predict_egammat - varie * pi / 180.)));

	edbias = ((etat * vcl * (edesireLOS - eLOS)) / (N * r));

	// acceleration command
	if (r >= 10.) {
		// 二版
		Aca = N * vclxy * adLOS - etap * rxy * (adesireLOS - aLOS);
		Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS);
		// Ace = N * vcl * edLOS - etap * r * (edesireLOS - eLOS) - g_uav*cos(egamma);
		// 原版
		// Aca = N*vclxy*(adLOS-adbias);
		// Ace = N*vcl*(edLOS-edbias);

		Acla = Aca;
		Acle = Ace;

		axc = -Ace * cos(aLOS) * sin(eLOS) - Aca * sin(aLOS);
		ayc = -Ace * sin(aLOS) * sin(eLOS) + Aca * cos(aLOS);
		azc = Ace * cos(eLOS);
	}
	else {
		axc = -Acle * cos(aLOS) * sin(eLOS) - Acla * sin(aLOS);
		ayc = -Acle * sin(aLOS) * sin(eLOS) + Acla * cos(aLOS);
		azc = Acle * cos(eLOS);
		Aca = Acla;
		Ace = Acle;
	}

	acc_sum = sqrt(axc*axc+ayc*ayc);
	// std::cout << "The acc_cmd in gazebo frame is (" << ayc << ", " << axc << ", " << azc << ") = " << acc_sum << " m/s^2" << std::endl;
	std::cout << "The acc_azimuth is " << Aca << ", and the acc_elevation is " << Ace << "m/s^2" << std::endl;

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// calculate the uav position
	// xm_next = xm + vxm*delta_t + 0.5*axc*delta_t*delta_t;
	// ym_next = ym + vym*delta_t + 0.5*ayc*delta_t*delta_t;
	// zm_next = zm + vzm*delta_t + 0.5*(azc - 9.81)*delta_t*delta_t;

	// publish position command
	// uavWaypointCmd.pose.position.x = ym_next; // change to gazebo coordinates
	// uavWaypointCmd.pose.position.y = xm_next; // change to gazebo coordinates
	// uavWaypointCmd.pose.position.z = zm_next; // change to gazebo coordinates

	// std::cout << "The waypoint cmd is (" << uavWaypointCmd.pose.position.x << ", " << uavWaypointCmd.pose.position.y << ", " << uavWaypointCmd.pose.position.z << ")" << std::endl;

	// uav_cmd_waypint_pub.publish(uavWaypointCmd);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Publish the linear acceleration command

	// uavLinearAccCmd.vector.x = ayc;
	// uavLinearAccCmd.vector.y = axc;
	// uavLinearAccCmd.vector.z = azc;

	// std::cout << "The acc_cmd is (" << axc << ", " << ayc << ", " << azc << ")\n" << std::endl;

	// uav_cmd_acc_pub.publish(uavLinearAccCmd);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Publish the raw attitude and thrust
	// acc_h_x = axc*cos(agamma) + ayc*sin(agamma);
	// acc_h_y = -axc*sin(agamma) + ayc*cos(agamma);
	// acc_h_z = azc;
	// acc_sum = sqrt(axc*axc + ayc*ayc + azc*azc);
	// acc_h_sum = sqrt(acc_h_x*acc_h_x + acc_h_y*acc_h_y + acc_h_z*acc_h_z);
	// std::cout << "The acc_cmd in ENU frame is (" << -axc << ", " << ayc << ", " << -azc << ") = " << acc_sum << " m/s^2" << std::endl;
	// std::cout << "The acc_cmd in ENU frame is (" << acc_h_x << ", " << acc_h_y << ", " << acc_h_z << ") = " << acc_h_sum << " m/s^2" << std::endl;

	// roll_ang = atan2(ayc_enu, azc_enu);
	// pitch_ang = atan2(-axc_enu, 9.81);
	// yaw_ang = atan2(sin(roll_ang)*cos(pitch_ang), cos(roll_ang)*cos(pitch_ang));

	// roll_ang = atan2(azc,sqrt(axc*axc+ayc*ayc));
	// pitch_ang = atan2(sqrt(axc*axc+ayc*ayc),sqrt(axc*axc+ayc*ayc+azc*azc));
	// yaw_ang = atan2(sin(roll_ang)*cos(pitch_ang), cos(roll_ang)*cos(pitch_ang));

	// std::cout << "The attitude_cmd is (" << roll_ang*180./pi << ", " << pitch_ang*180./pi << ", " << yaw_ang*180./pi << ")" << std::endl;

	// q_w = cos(roll_ang/2) * cos(pitch_ang/2) * cos(yaw_ang/2) + sin(roll_ang/2) * sin(pitch_ang/2) * sin(yaw_ang/2);
	// q_x = sin(roll_ang/2) * cos(pitch_ang/2) * cos(yaw_ang/2) - cos(roll_ang/2) * sin(pitch_ang/2) * sin(yaw_ang/2);
	// q_y = cos(roll_ang/2) * sin(pitch_ang/2) * cos(yaw_ang/2) + sin(roll_ang/2) * cos(pitch_ang/2) * sin(yaw_ang/2);
	// q_z = cos(roll_ang/2) * cos(pitch_ang/2) * sin(yaw_ang/2) - sin(roll_ang/2) * sin(pitch_ang/2) * cos(yaw_ang/2);

	// std::cout << "The attitude_cmd in quaternion is (" << q_w << ", " << q_x << ", " << q_y << ", " << q_z << ")\n" << std::endl;

	// calculate the velocity
	// vx_next = (vx + axc * delta_t);
	// vy_next = (vy + ayc * delta_t);
	// vz_next = (vz + (azc-9.81) * delta_t);
	// std::cout << "The vel_cmd is (" << vx_next << ", " << vy_next << ", " << vz_next << ")" << std::endl;

	// normalize the velocity_next
	// v_next = sqrt(vx_next*vx_next + vy_next*vy_next + vz_next*vz_next);
	// unit_vx_next = vx_next / v_next;
	// unit_vy_next = vy_next / v_next;
	// unit_vz_next = vz_next / v_next;
	// std::cout << "The unit_vel_cmd is (" << unit_vx_next << ", " << unit_vy_next << ", " << unit_vz_next << ")" << std::endl;

	// generate attitude of uav in quaternion
	// q_w = 0.;
	// q_x = unit_vy_next; // change to gazebo coordinates
	// q_y = unit_vx_next; // change to gazebo coordinates
	// q_z = unit_vz_next;
	// std::cout << "The attitude is (" << q_w << ", " << q_x << ", " << q_y << ", " << q_z << ")\n" << std::endl;

	// thrust = mass_UC * (sqrt(acc_h_x*acc_h_x + acc_h_y*acc_h_y + acc_h_z*acc_h_z) + 9.81);
	// std::cout << "The thr_cmd is " << thrust << " N\n" << std::endl;
	// thr = thrust / thr_max;

	////////////////////////////////////////////////////////////////////////////
	// Publish the angular velocity and thrust

	// calculate pitch rate Q and yaw rate R
	v_total = sqrt(vxm*vxm + vym*vym + vzm*vzm);
	pitch_rate = -Ace / v_total;
	yaw_rate = Aca / v_total;

	double phi_bar;
	phi_bar = std::atan2(v_total*yaw_rate, 9.81);
	double k_roll_rate = 2.;
	roll_rate = k_roll_rate*(phi_bar - phi_ang);
	int mask = 132;
	std::cout << "The roll rate = " << roll_rate << ", and pitch rate = " << pitch_rate << ", and yaw rate = " << yaw_rate << std::endl;
	// std::cout << "The mask is " << mask << "\n" << std::endl;

	// P control velocity
	double thr_cmd, v_ref = 33., v_diff;
	v_diff = v_ref - v_total;
	if (v_diff > 0.) {
		thr_cmd = 2.*v_diff/v_ref;
	} else {
		thr_cmd = 0.;
	}
	std::cout << "The thrust command is " << thr_cmd << "\n" << std::endl;

	uavAttitudeThrustCmd.type_mask = mask;
	uavAttitudeThrustCmd.orientation.x = q_x;
	uavAttitudeThrustCmd.orientation.y = q_y;
	uavAttitudeThrustCmd.orientation.z = q_z;
	uavAttitudeThrustCmd.orientation.w = q_w;
	uavAttitudeThrustCmd.body_rate.x = roll_rate;
	uavAttitudeThrustCmd.body_rate.y = pitch_rate;
	uavAttitudeThrustCmd.body_rate.z = yaw_rate;
	uavAttitudeThrustCmd.thrust = thr_cmd;

	uav_cmd_att_thr_pub.publish(uavAttitudeThrustCmd);

	//////////////////////////////////////////////////////////////////////
	// Publish the linear velocity command

	// uavLinearVelCmd.twist.linear.x = (vym + ayc * delta_t); // change to gazebo coordinates
	// uavLinearVelCmd.twist.linear.y = (vxm + axc * delta_t); // change to gazebo coordinates
	// uavLinearVelCmd.twist.linear.z = (vzm + (azc-9.81) * delta_t);
	// uavLinearVelCmd.twist.angular.x = 0.0;
	// uavLinearVelCmd.twist.angular.y = 0.0;
	// uavLinearVelCmd.twist.angular.z = 0.0;

	// std::cout << "The vel_cmd is (" << uavLinearVelCmd.twist.linear.x << ", " << uavLinearVelCmd.twist.linear.y << ", " << uavLinearVelCmd.twist.linear.z << ")\n" << std::endl;

	// uav_cmd_vel_pub.publish(uavLinearVelCmd);
}
