#include "ros/ros.h"
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include <ethercat_test/vel.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


#define _USE_MATH_DEFINES

using ethercat_test::vel;
using sensor_msgs::Imu;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

using nav_msgs::Odometry;
using geometry_msgs::Twist;

class WheelOdometry{

    public:

    void initiate_variables();
    void publisher_set();
    void subscriber_set();
    void callback_imu(const Imu::ConstPtr& imu_msg);
    void callback_enc(const vel::ConstPtr& enc_msg);
    void calculate_velocity_position();
    double calculate_linear_velocity(Vector3d velocity);
    void velocity_pose_publish();
    Vector3d getTransformVec(double angle1, double angle2, double dt);
    MatrixXd getRotMat(double angle);
    void angleNormalizer(double &angle);
    
    private:
    // Variables
    Vector4d measure_val;
    MatrixXd Jacob = MatrixXd(3,4);
    
    Vector3d v_robot;

    Vector3d v_est;
    Vector3d p_est_curr;
    Vector3d p_est_prev;
    Quaterniond q_imu = Quaterniond::Identity();
    Quaterniond q_est = Quaterniond::Identity();
    Quaterniond q_init = Quaterniond::Identity();

    bool is_yaw_init = false;

    double yaw = 0;
    double yaw_init = 0;
    double linear_velocity = 0;
    double angular_velocity = 0;
    double velocity_heading = 0;
    double angle1 = 0;
    double angle2 = 0;

    // mobile robot spec
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double l_a = 0.2170;
    const double l_b = 0.1687;
    const double l_sep = l_a + l_b;

    // Motor spec and unit conversion ratio
    const double gear_ratio = 73.5;
    const double rpm_to_radps = 2.0*M_PI/60.0;
    
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher publisher_odom;
    ros::Publisher publisher_twist;
    ros::Subscriber subscriber_Imu;
    ros::Subscriber subscriber_encoder;

    // ROS Time info
    double t_curr = 0;
    double dt = 1/200.0;
    double t_prev = ros::Time::now().toSec();
};

void WheelOdometry::initiate_variables()
{
    measure_val.setZero();
    
    Jacob<<r/4.0, r/4.0, r/4.0, r/4.0,
        -r/4.0, r/4.0, -r/4.0, r/4.0,
        -r/4.0/l_sep, r/4.0/l_sep, r/4.0/l_sep, -r/4.0/l_sep;
    
    v_robot.setZero();
    
    v_est.setZero();
    p_est_curr.setZero();
    p_est_prev.setZero();
}

void WheelOdometry::publisher_set()
{
    publisher_odom = nh_.advertise<Odometry>("wheel_odom",1);
    publisher_twist = nh_.advertise<Twist>("vel",1);
}

void WheelOdometry::subscriber_set()
{
    subscriber_Imu = nh_.subscribe("/imu", 1, &WheelOdometry::callback_imu,this);
    subscriber_encoder = nh_.subscribe("/measure", 1, &WheelOdometry::callback_enc,this);
    
}

void WheelOdometry::callback_imu(const Imu::ConstPtr& imu_msg)
{
    
    q_imu.w() = imu_msg->orientation.w;
    q_imu.x() = imu_msg->orientation.x;
    q_imu.y() = imu_msg->orientation.y;
    q_imu.z() = imu_msg->orientation.z;
    
   
    auto euler = q_imu.toRotationMatrix().eulerAngles(0,1,2);
    
    if(is_yaw_init == false){
        yaw_init = euler[2];
        is_yaw_init = true;
        ROS_INFO("Initiate yaw");
    }
    
    yaw = euler[2] - yaw_init;
    angleNormalizer(yaw);

    q_est = AngleAxisd(0,Vector3d::UnitX())
            * AngleAxisd(0,Vector3d::UnitY())
            * AngleAxisd(yaw,Vector3d::UnitZ());

    angular_velocity = imu_msg->angular_velocity.z;

}

void WheelOdometry::callback_enc(const vel::ConstPtr& enc_msg)
{

    measure_val << enc_msg->velocity[0], -(enc_msg->velocity[1]),
                -(enc_msg->velocity[2]), enc_msg->velocity[3];

    calculate_velocity_position();
    std::cout<<"Yaw"<<std::endl;
    std::cout<<p_est_curr(2)<<std::endl;
}

void WheelOdometry::calculate_velocity_position()
{
    v_robot = Jacob * measure_val/gear_ratio*rpm_to_radps;

    velocity_heading = atan2(v_robot(1),v_robot(0));
    linear_velocity = calculate_linear_velocity(v_robot);

    angle1 = velocity_heading + yaw;
    angle2  = angle1 + v_robot(2)*dt;

    v_est = getRotMat(yaw)*v_robot;

//    if(fabs(v_robot(2))>0.1)
//    {
//        p_est_curr = p_est_prev 
//        + linear_velocity/angular_velocity*getTransformVec(angle1,angle2,dt);
//    }
//    else
//    {
        p_est_curr = p_est_prev + v_est*dt;
//    }

    

    p_est_curr(2) = yaw;

    p_est_prev = p_est_curr;

}

void WheelOdometry::velocity_pose_publish()
{
    t_curr = ros::Time::now().toSec();

    dt = t_curr - t_prev;

    Odometry wheel_odom;
    Twist vel;
    wheel_odom.pose.pose.position.x = p_est_curr(0);
    wheel_odom.pose.pose.position.y = p_est_curr(1);
    
    wheel_odom.pose.pose.orientation.w = q_est.w();
    wheel_odom.pose.pose.orientation.x = q_est.x();
    wheel_odom.pose.pose.orientation.y = q_est.y();
    wheel_odom.pose.pose.orientation.z = q_est.z();
    
    wheel_odom.twist.twist.linear.x = v_robot(0);
    wheel_odom.twist.twist.linear.y = v_robot(1);
    wheel_odom.twist.twist.angular.z = v_robot(2);
    
    vel.linear.x = v_robot(0);
    vel.linear.y = v_robot(1);
    vel.angular.z = v_robot(2);
    
    publisher_odom.publish(wheel_odom);
    publisher_twist.publish(vel);
    t_prev = t_curr;
}

double WheelOdometry::calculate_linear_velocity(Vector3d velocity)
{
    return sqrt(pow(velocity(0),2)+pow(velocity(1),2));
}

Vector3d WheelOdometry::getTransformVec(double angle1, double angle2, double dt)
{
    Vector3d TfVec;
    TfVec << sin(angle1+angle2)-sin(angle1),
            -cos(angle1+angle2)+cos(angle1),
            angle1-angle2;

    return TfVec;

}

MatrixXd WheelOdometry::getRotMat(double angle)
{
    MatrixXd R = MatrixXd(3,3);
    R << cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0,  0,  1;

    return R;

}

void WheelOdometry::angleNormalizer(double& angle)
{
    angle = atan2(sin(angle),cos(angle));
}
