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

ros::Time TimeStamp;

class WheelOdometry{

    public:

    void initiate_variables();
    void publisher_set();
    void subscriber_set();
    void callback_camera_imu(const Odometry& camera_imu_msg);
    void callback_enc(const vel& measure_msg);
    void calculate_velocity_position();
    void velocity_pose_publish();
    MatrixXd getRotMat(double angle);
    
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

    double yaw = 0;
    double angular_vel = 0;

    // mobile robot spec
    const double wheel_radious = 0.1520/2.0;
    const double r = wheel_radious;
    const double l_a = 0.2170;
    const double l_b = 0.1687;
    const double l_sep = l_a + l_b;

    // Motor spec and unit conversion ratio
    const double gear_ratio = 74.5;
    const double rpm_to_radps = 2.0*M_PI/60.0;
    
    // ROS
    ros::NodeHandle nh_;
    ros::Publisher publisher_odom;
    ros::Publisher publisher_twist;
    ros::Subscriber subscriber_camera_imu;
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
    publisher_odom = nh_.advertise<Odometry>("odom",1);
}

void WheelOdometry::subscriber_set()
{
    subscriber_camera_imu = nh_.subscribe("/camera/odom/sample", 1, &WheelOdometry::callback_camera_imu,this);
    subscriber_encoder = nh_.subscribe("/measure", 1, &WheelOdometry::callback_enc,this);
    
}

void WheelOdometry::callback_camera_imu(const Odometry& camera_imu_msg)
{
    double qw,qx,qy,qz;
    
    qw = camera_imu_msg.pose.pose.orientation.w;
    qx = camera_imu_msg.pose.pose.orientation.x;
    qy = camera_imu_msg.pose.pose.orientation.y;
    qz = camera_imu_msg.pose.pose.orientation.z;
    
    yaw = atan2(2*(qw*qz + qx*qy),1-2*(qy*qy+qz*qz));
    angular_vel = camera_imu_msg.twist.twist.angular.z;
}
void WheelOdometry::callback_enc(const vel& measure_msg)
{

    measure_val << measure_msg.velocity[0], -(measure_msg.velocity[1]),
                -(measure_msg.velocity[2]), measure_msg.velocity[3];

    calculate_velocity_position();
}

void WheelOdometry::calculate_velocity_position()
{
    v_robot = Jacob * measure_val/gear_ratio*rpm_to_radps;
    v_est = getRotMat(yaw)*v_robot;
    p_est_curr = p_est_prev + v_est*dt;
    p_est_curr(2) = yaw;
    p_est_prev = p_est_curr;

}

void WheelOdometry::velocity_pose_publish()
{
    t_curr = ros::Time::now().toSec();

    dt = t_curr - t_prev;

    Odometry wheel_odom;

    TimeStamp = ros::Time::now();
    wheel_odom.header.stamp = TimeStamp;
    wheel_odom.child_frame_id = "base_footprint";
    wheel_odom.header.frame_id = "odom";
    wheel_odom.pose.pose.position.x = p_est_curr(0);
    wheel_odom.pose.pose.position.y = p_est_curr(1);
    
    wheel_odom.pose.pose.orientation.w = cos(yaw/2.0);
    wheel_odom.pose.pose.orientation.x = 0;
    wheel_odom.pose.pose.orientation.y = 0;
    wheel_odom.pose.pose.orientation.z = sin(yaw/2.0);
    
    wheel_odom.twist.twist.linear.x = v_robot(0);
    wheel_odom.twist.twist.linear.y = v_robot(1);
    wheel_odom.twist.twist.angular.z = angular_vel;    
    
    publisher_odom.publish(wheel_odom);

    t_prev = t_curr;
}

MatrixXd WheelOdometry::getRotMat(double angle)
{
    MatrixXd R = MatrixXd(3,3);
    R << cos(angle), -sin(angle), 0,
        sin(angle), cos(angle), 0,
        0,  0,  1;

    return R;

}