#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

typedef struct
{
    Eigen::Vector3d pos_gnd;
    Eigen::Vector3d vel_gnd;
    Eigen::Vector3d euler;
    Eigen::Vector3d euler_rate;
    double V;
    double cmd_p;
    double cmd_q;
}State_gnd;

void getFwPose(const nav_msgs::Odometry::ConstPtr& pose, State_gnd& state);
void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg, State_gnd& state);
State_gnd state;

//---------------------variable---------------------
void initialize();
void predict();
void correct(Eigen::VectorXd measure);
Eigen::MatrixXd state_to_measurement(Eigen::MatrixXd sigma_state);
Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state);
void noise_estimate(int window_size);

int x_size, y_size, x_sigmavector_size, y_sigmavector_size, statesize, measurementsize;

Eigen::VectorXd x;        // states
Eigen::VectorXd y;        // measurements
Eigen::VectorXd x_hat;    // states mean
Eigen::VectorXd y_hat;    // measurements mean
Eigen::VectorXd x_hat_;   // without noise
Eigen::VectorXd y_hat_;   // without noise

double dt;
double L;
double alpha;
double kappa;
double beta;
double lambda;

Eigen::VectorXd w_c;     // weight cov 
Eigen::VectorXd w_m;     // weight 

Eigen::MatrixXd x_sigmavector;
Eigen::MatrixXd y_sigmavector;
Eigen::MatrixXd x_sigmavector_;  // without noise
Eigen::MatrixXd y_sigmavector_;  // without noise
Eigen::MatrixXd H;       // measurment model transform
Eigen::MatrixXd P;       // covariance matrix
Eigen::MatrixXd Q;       // covarriance of process noise
Eigen::MatrixXd R;       // covarriance of measurement noise
Eigen::MatrixXd P_;       // without process noise covariance

Eigen::VectorXd q;       // process noise
Eigen::VectorXd r;       // measurement noise
// deque<Eigen::VectorXd> q_window, r_window;
// deque<Eigen::MatrixXd> Q_window, R_window;
// deque<double> w_window;

Eigen::MatrixXd P_yy;
Eigen::MatrixXd P_xy;
Eigen::MatrixXd P_yy_;   // without measurement noise covariance

Eigen::MatrixXd Kalman_gain;



int main(int argc, char **argv){
    statesize = 12;
    measurementsize = 6;
    state.cmd_p = 0;
    state.cmd_q = 0;
    ros::init(argc, argv, "disturbance_est");
    ros::NodeHandle nh;
    ros::Rate rate(50);
    ros::Subscriber fw_pose_sub = nh.subscribe<nav_msgs::Odometry>("/uav0/base_pose_ground_truth", 10, 
        boost::bind(&getFwPose, _1, boost::ref(state)));
    ros::Subscriber ans_sub = nh.subscribe<std_msgs::Float32MultiArray>("/nmpc_ans", 10, 
        boost::bind(&getAns, _1, boost::ref(state)));
    ros::Publisher est_pos_pub = nh.advertise<geometry_msgs::Vector3>("/estimated_pose", 10);
    ros::Publisher est_euler_pub = nh.advertise<geometry_msgs::Vector3>("/estimated_attitude", 10);
    ros::Publisher dis_pos_pub = nh.advertise<geometry_msgs::Vector3>("/disturbance_pose", 10);
    ros::Publisher dis_euler_pub = nh.advertise<geometry_msgs::Vector3>("/disturbance_attitude", 10);
    initialize();
    ros::Time current_time = ros::Time::now();
    ros::Time previous_time = ros::Time::now();
    //--------x,y,z,roll,pitch,yaw,d_x,d_y,d_z,d_roll,d_pitch,d_yaw--------
    P(0,0) = P(1,1) = P(2,2) = 90.0;
    P(3,3) = P(4,4) = P(5,5) = 0.8;
    P(6,6) = P(7,7) = P(8,8) = 0.5;
    P(9,9) = P(10,10) = P(11,11) = 0.5;

    //--------x_dot,y_dot,z_dot,roll_dot,pitch_dot,yaw_dot-----------
    R(0,0) = R(1,1) = R(2,2) = 0.0001;
    R(3,3) = R(4,4) = R(5,5) = 0.0005;

    //--------x,y,z,roll,pitch,yaw,d_x,d_y,d_z,d_roll,d_pitch,d_yaw--------
    Q(0,0) = Q(1,1) = Q(2,2) = 0.05;
    Q(3,3) = Q(4,4) = Q(5,5) = 0.05;
    Q(6,6) = Q(7,7) = Q(8,8) = 0.05;
    Q(9,9) = Q(10,10) = Q(11,11) = 0.05;

    while (ros::ok())
    {   
        if(!isnormal(x(0))){
            initialize();
            x(0) = state.pos_gnd[0];
            x(1) = state.pos_gnd[1];
            x(2) = state.pos_gnd[2];
            x(3) = state.euler[0];
            x(4) = state.euler[1];
            x(5) = state.euler[2];
            std::cout<< !isnormal(x(0)) <<std::endl;
        }
        current_time = ros::Time::now();
        dt = current_time.toSec() - previous_time.toSec();
        previous_time = current_time;

        predict();

        Eigen::VectorXd measure_vector;
        measure_vector.setZero(measurementsize);
        measure_vector << state.pos_gnd, state.euler;

        correct(measure_vector);

        //--------------publish---------------------
        geometry_msgs::Vector3 est_pos;
        est_pos.x = x(0);
        est_pos.y = x(1);
        est_pos.z = x(2);
        est_pos_pub.publish(est_pos);

        geometry_msgs::Vector3 est_euler;
        est_euler.x = x(3);
        est_euler.y = x(4);
        est_euler.z = x(5);
        est_euler_pub.publish(est_euler);

        geometry_msgs::Vector3 dis_pos;
        dis_pos.x = x(6);
        dis_pos.y = x(7);
        dis_pos.z = x(8);
        dis_pos_pub.publish(dis_pos);

        geometry_msgs::Vector3 dis_euler;
        dis_euler.x = x(9);
        dis_euler.y = x(10);
        dis_euler.z = x(11);
        dis_euler_pub.publish(dis_euler);

        ros::spinOnce();
        rate.sleep();
    }
}


void initialize()
{
    x_size = statesize;
    y_size = measurementsize;
    L = x_size;
    alpha = 1e-3;
    kappa = 0;
    beta = 2;

    lambda = alpha * alpha * (L + kappa) - L;
    x_sigmavector_size = 2 * L + 1;

    x.setZero(x_size);
    y.setZero(y_size);
    x_hat.setZero(x_size);
    y_hat.setZero(y_size);

    x_sigmavector.setZero(x_size, x_sigmavector_size);
    y_sigmavector.setZero(y_size, y_sigmavector_size);

    H.setZero(y_size, x_size);

    w_c.setZero(x_sigmavector_size);
    w_m.setZero(x_sigmavector_size);
    w_c(0) = (lambda / (L + lambda)) + (1 - alpha*alpha + beta);
    w_m(0) = lambda / (L + lambda);

    for(int i = 1; i < x_sigmavector_size; i++)
    {
        w_c(i) = 1 / (2 * (L + lambda));
        w_m(i) = 1 / (2 * (L + lambda));
    }

    Q = 5e-7 * Eigen::MatrixXd::Identity(x_size, x_size);
    R = 5e-4 * Eigen::MatrixXd::Identity(y_size, y_size);
    P = 1e-3 * Eigen::MatrixXd::Identity(x_size, x_size);

    q.setZero(x_size);
    r.setZero(y_size);

    P_.setZero(x_size, x_size);
    P_yy.setZero(y_size, y_size);
    P_xy.setZero(x_size, y_size);
    P_yy_.setZero(y_size, y_size);
}

void predict()
{
    // calculate sigma point
    P = (L + lambda) * P;
    Eigen::MatrixXd M = (P).llt().matrixL();
    Eigen::VectorXd sigma;

    x_sigmavector.col(0) = x;
    //cout << "x last predict: " << endl << x << endl;
    for(int i = 0; i < x_size; i++)
    {
        sigma = (M.row(i)).transpose();
        x_sigmavector.col( i+1 ) = x + sigma;
        x_sigmavector.col( i+x_size+1 ) = x - sigma;
    }

    x_sigmavector_ = dynamics(x_sigmavector);
    x_sigmavector = x_sigmavector_ ;//+ q * Eigen::MatrixXd::Constant(1, x_sigmavector_size, 1);

    // mean
    x_hat_.setZero(x_size);
    x_hat.setZero(x_size);
    for(int i = 0; i < x_sigmavector_size; i++)
    {
        x_hat_ += w_m(i) * x_sigmavector_.col(i);
    }
    // add process noise
    x_hat = x_hat_;// + q;

    // covariance
    P_.setZero(x_size, x_size);
    P.setZero(x_size, x_size);
    for(int i = 0; i < x_sigmavector_size; i++)
    {
        P_ += w_c(i) * (x_sigmavector.col(i) - x_hat) * ( (x_sigmavector.col(i) - x_hat).transpose() );
    }
    // add process noise covariance
    P = P_ + Q;

    y_sigmavector_ = state_to_measurement(x_sigmavector);
    y_sigmavector = y_sigmavector_; // + r * Eigen::MatrixXd::Constant(1, x_sigmavector_size, 1);
 
    // mean
    y_hat_.setZero(y_size);
    y_hat.setZero(y_size);
    for(int i = 0; i < x_sigmavector_size; i++)
    {
        y_hat_ += w_m(i) * y_sigmavector_.col(i);
    }
    // add measurement noise
    y_hat = y_hat_;// + r;
}

Eigen::MatrixXd dynamics(Eigen::MatrixXd sigma_state)
{
    Eigen::MatrixXd predict_sigma_state(x_size, x_sigmavector_size);

    for(int i = 0; i < x_sigmavector_size; i++)
    {

        double x1 = sigma_state(0, i);
        double x2 = sigma_state(1, i);
        double x3 = sigma_state(2, i);
        double x4 = sigma_state(3, i);
        double x5 = sigma_state(4, i);
        double x6 = sigma_state(5, i);
        double x7 = sigma_state(6, i);
        double x8 = sigma_state(7, i);
        double x9 = sigma_state(8, i);
        double x10 = sigma_state(9, i);
        double x11 = sigma_state(10, i);
        double x12 = sigma_state(11, i);

        //-------------------cal prediction state------------------------
        double x1_ = x1 + state.V*cos(state.euler(2))*cos(state.euler(1))*dt + x7;
        double x2_ = x2 + state.V*sin(state.euler(2))*cos(state.euler(1))*dt + x8;
        double x3_ = x3 - state.V*sin(state.euler(1))*dt + x9;
        double x4_ = x4 + (state.cmd_p + sin(state.euler(0))*tan(state.euler(1))*state.cmd_q)*dt+x10;
        double x5_ = x5 + state.cmd_q*cos(state.euler(0))*dt+x11;
        double x6_ = x6 - 9.81*tan(state.euler(0))*cos(state.euler(1))*dt/state.V + x12;
        // double x4_ = x4 + (state.cmd_p + sin(state.euler(0))*tan(state.euler(1))*state.cmd_q)*dt+state.cmd_p*x10;
        // double x5_ = x5 + state.cmd_q*cos(state.euler(0))*dt+state.cmd_q*x11;
        // double x6_ = x6 - 9.81*tan(state.euler(0))*cos(state.euler(1))*dt/state.V + x12;

        predict_sigma_state(0, i) =  x1_;
        predict_sigma_state(1, i) =  x2_;
        predict_sigma_state(2, i) =  x3_;
        predict_sigma_state(3, i) =  x4_;
        predict_sigma_state(4, i) =  x5_;
        predict_sigma_state(5, i) =  x6_;
        predict_sigma_state(6, i) =  x7;
        predict_sigma_state(7, i) =  x8;
        predict_sigma_state(8, i) =  x9;
        predict_sigma_state(9, i) =  x10;
        predict_sigma_state(10, i) = x11;
        predict_sigma_state(11, i) = x12;
    }

    return predict_sigma_state;
}

Eigen::MatrixXd state_to_measurement(Eigen::MatrixXd sigma_state)
{
    y_size = measurementsize;
    x_sigmavector_size = 2 * statesize + 1;
    Eigen::MatrixXd predict_sigma_measure(y_size, x_sigmavector_size);

    for(int i = 0; i < x_sigmavector_size; i++)
    {
        predict_sigma_measure(0, i) = sigma_state(0, i);
        predict_sigma_measure(1, i) = sigma_state(1, i);
        predict_sigma_measure(2, i) = sigma_state(2, i);
        predict_sigma_measure(3, i) = sigma_state(3, i);
        predict_sigma_measure(4, i) = sigma_state(4, i);
        predict_sigma_measure(5, i) = sigma_state(5, i);
    }
    return predict_sigma_measure;
}

void correct(Eigen::VectorXd measure)
{
    y = measure;

    P_yy_.setZero(y_size, y_size);
    P_yy.setZero(y_size, y_size);
    P_xy.setZero(x_size, y_size);

    for(int i = 0; i < x_sigmavector_size; i++)
    {
        Eigen::MatrixXd err;
        Eigen::MatrixXd err_t;
        err = y_sigmavector.col(i) - y_hat;
        err_t = err.transpose();
        P_yy_ += w_c(i) * err * err_t;
    }
    //add measurement noise covarinace
    P_yy = P_yy_ + R;

    for(int i = 0; i < x_sigmavector_size; i++)
    {
        Eigen::VectorXd err_y , err_x;
        err_y = y_sigmavector.col(i) - y_hat;
        err_x = x_sigmavector.col(i) - x_hat;
        P_xy += w_c(i) * err_x * err_y.transpose();
    }

    Kalman_gain = P_xy * (P_yy.inverse());

    // if(!measurement_flag)
    // {
    //     y(0) = y_hat(0);
    //     y(1) = y_hat(1);
    // }
    x = x_hat + Kalman_gain * (y - y_hat);
    //cout << "x predict: " << endl << x << endl;

    P = P - Kalman_gain * P_yy * (Kalman_gain.transpose());
}

void getFwPose(const nav_msgs::Odometry::ConstPtr& odom, State_gnd& state)
{
    state.pos_gnd <<
        odom->pose.pose.position.x,
        odom->pose.pose.position.y,
        odom->pose.pose.position.z
    ;
    
    state.vel_gnd <<
        odom->twist.twist.linear.x,
        odom->twist.twist.linear.y,
        odom->twist.twist.linear.z
    ;

    Eigen::Quaterniond q(
        odom->pose.pose.orientation.w,
        odom->pose.pose.orientation.x,
        odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z
    );

    Eigen::Matrix3d R = q.toRotationMatrix();
    double roll  = atan2(R(2,1), R(2,2));
    double pitch = -asin(R(2,0));
    double yaw   = atan2(R(1,0), R(0,0));

    state.euler = Eigen::Vector3d(roll, pitch, yaw);
}

void getAns(const std_msgs::Float32MultiArray::ConstPtr& msg, State_gnd& state){
    state.V = static_cast<double>(msg->data[0]);
    state.cmd_p = static_cast<double>(msg->data[1]);
    state.cmd_q = static_cast<double>(msg->data[2]);
}