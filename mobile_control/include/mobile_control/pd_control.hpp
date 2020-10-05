#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/TrajectoryPointMsg.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ethercat_test/vel.h>

#define _USE_MATH_DEFINES

using teb_local_planner::TrajectoryPointMsg;
using teb_local_planner::FeedbackMsg;

using geometry_msgs::PoseStamped;
using tf::TransformListener;
using tf::StampedTransform;

using nav_msgs::Odometry;
using ethercat_test::vel;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

class pd_control{
    
    public:

    void init_Variables();
    void publisher_set();
    void subscriber_set();
    void callback_traj(const FeedbackMsg::ConstPtr& traj_msg);
    void callback_vel(const Odometry::ConstPtr& odom);
    void callback_goal(const PoseStamped::ConstPtr& goal_msg);
    void get_SLAM_pose();
    void error_calculation();
    void cmd_vel_calculation();
    Vector4d inverse_kinematics(Vector3d vel_cmd);
    Vector4d clamp(Vector4d motor_input);
    void cmd_vel_publish();
    Matrix3d getRotMat(double yaw);
    void angleNormalizer(double &yaw);
    double distance(Vector3d goal, Vector3d est);

    private:

    // Variables
    Vector3d goal_pose;
    Quaterniond goal_quat = Quaterniond::Identity();

    Vector3d p_des;
    Quaterniond q_des = Quaterniond::Identity();
    Vector3d v_des;
    Vector3d a_des;

    Vector3d p_est;   // Global frame
    Quaterniond q_est = Quaterniond::Identity();
    Vector3d v_est;   // Robot frame

    Vector3d p_err;   // Robot frame
    Vector3d v_err;   // Robot frame

    Vector3d a_cmd;
    Vector3d v_cmd;
    Vector3d v_cmd_prev;
    double v_t_prev = 0;

    MatrixXd Kp = MatrixXd(3,3);
    MatrixXd Kd = MatrixXd(3,3);
    MatrixXd InvJacob = MatrixXd(4,3);

    Vector4d motor_vel_input;

    // mobile robot spec
    const double wheel_radious = 0.1520/2.0;
    const double l_a = 0.2170;
    const double l_b = 0.1687;
    const double l_sep = l_a + l_b;

    // Motor spec and unit conversion ratio
    const double gear_ratio = 74.5;
    const int motor_vel_lim = 6000;
    const double radps_to_rpm = 60/2.0/M_PI;

    // Controller PD Gain
    const double Kp_xy = 5;
    const double Kp_yaw = 1;

    const double Kd_xy = 2;
    const double Kd_yaw = 0.1;

    bool control_enable = true;

    // ROS
    ros::NodeHandle nh_;
    TransformListener listener_;
    StampedTransform transform;

    ros::Publisher publisher_cmd_vel;
    ros::Publisher publisher_des_traj;
    ros::Publisher publisher_slam_pose;
    ros::Publisher publisher_error;

    ros::Subscriber subscriber_goal;
    ros::Subscriber subscriber_traj;
    ros::Subscriber subscriber_vel;

    // ROS Time info
    double curr_time;
    double last_time = ros::Time::now().toSec();
    double dt = 0.005;
};

void pd_control::init_Variables()
{
    goal_pose.setZero();

    p_des.setZero();
    v_des.setZero();
    a_des.setZero();


    p_est.setZero();
    v_est.setZero();

    p_err.setZero();
    v_err.setZero();

    a_cmd.setZero();
    v_cmd.setZero();
    v_cmd_prev.setZero();

    Kp<<Kp_xy, 0, 0,
        0, Kp_xy, 0,
        0, 0, Kp_yaw;

    Kd<<Kd_xy, 0, 0,
        0, Kd_xy, 0,
        0, 0, Kd_yaw;

    InvJacob<<1.0/wheel_radious, -1.0/wheel_radious, -l_sep/wheel_radious,
            1.0/wheel_radious, 1.0/wheel_radious, l_sep/wheel_radious,
            1.0/wheel_radious, -1.0/wheel_radious, l_sep/wheel_radious,
            1.0/wheel_radious, 1.0/wheel_radious, -l_sep/wheel_radious;

    motor_vel_input.setZero();

}

void pd_control::publisher_set()
{
    publisher_cmd_vel = nh_.advertise<vel>("/input_msg",1);

}

void pd_control::subscriber_set()
{
    subscriber_traj = nh_.subscribe("/move_base/TebLocalPlannerROS/teb_feedback", 1, &pd_control::callback_traj, this);
    subscriber_vel = nh_.subscribe("/odom",1,&pd_control::callback_vel,this);
    subscriber_goal = nh_.subscribe("/move_base_simple/goal", 1, &pd_control::callback_goal, this);

}

// TrajectorPointMsg::ConstPtr , FeedbackMsg
void pd_control::callback_traj(const FeedbackMsg::ConstPtr& traj_msg)
{
/** TrajectoryPointMsg
    a_des << traj_msg->acceleration.linear.x,
            traj_msg->acceleration.linear.y,
            traj_msg->acceleration.angular.z;

    v_des << traj_msg->velocity.linear.x,
            traj_msg->velocity.linear.y,
            traj_msg->velocity.angular.z;

    q_des.w() = traj_msg->pose.orientation.w;
    q_des.x() = traj_msg->pose.orientation.x;
    q_des.y() = traj_msg->pose.orientation.y;
    q_des.z() = traj_msg->pose.orientation.z;

    auto euler = q_des.toRotationMatrix().eulerAngles(0,1,2);
    std::cout<<euler[2]<<std::endl;

    p_des << traj_msg->pose.position.x,
            traj_msg->pose.position.y, 
            euler[2];
**/

// FeedbackMsg
    int robot_index = traj_msg -> selected_trajectory_idx;
        
    int traj_index = 1;

    a_des<<traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.linear.x,
        traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.linear.y,
        traj_msg->trajectories[robot_index].trajectory[traj_index].acceleration.angular.z;

    v_des<<traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.x,
        traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.linear.y,
        traj_msg->trajectories[robot_index].trajectory[traj_index].velocity.angular.z;

    q_des.w() = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.w;
    q_des.x() = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.x;
    q_des.y() = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.y;
    q_des.z() = traj_msg->trajectories[robot_index].trajectory[traj_index].pose.orientation.z;
    
    auto euler = q_des.toRotationMatrix().eulerAngles(0,1,2);

    angleNormalizer(euler[2]);

    p_des<<traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.x,
        traj_msg->trajectories[robot_index].trajectory[traj_index].pose.position.y,
        euler[2];


}

void pd_control::callback_vel(const Odometry::ConstPtr& odom_msg)
{
    v_est<<odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y,
        odom_msg->twist.twist.angular.z;
}

void pd_control::callback_goal(const PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("Goal Recieved");

    control_enable = true;

    goal_quat.w() = goal_msg->pose.orientation.w;
    goal_quat.x() = goal_msg->pose.orientation.x;
    goal_quat.y() = goal_msg->pose.orientation.y;
    goal_quat.z() = goal_msg->pose.orientation.z;

    auto euler2 = goal_quat.toRotationMatrix().eulerAngles(0,1,2);

    goal_pose << goal_msg->pose.position.x, 
                goal_msg->pose.position.y,
                euler2[2];
}

void pd_control::get_SLAM_pose()
{
    
    double yaw_est;

    try{
        
        listener_.lookupTransform("map","base_footprint",ros::Time(0),transform);
        tf::Quaternion quat = transform.getRotation();
        yaw_est = tf::getYaw(quat);
        angleNormalizer(yaw_est);
        p_est << transform.getOrigin().x(), 
                transform.getOrigin().y(),
                yaw_est;
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void pd_control::error_calculation()
{
    p_err = p_des - p_est;
    v_err = v_des - v_est;

    angleNormalizer(p_err(2));
    
    ROS_INFO("P err : %lf %lf %lf",p_err(0),p_err(1),p_err(2));
}

void pd_control::cmd_vel_calculation()
{
    Vector3d v_angular;
    v_angular<<0,0,v_des(2);
    a_cmd = Kp*getRotMat(p_est(2))*p_err + Kd*v_err;
    v_cmd = v_des + a_cmd*dt;
    v_cmd_prev = v_cmd;
    //v_cmd = getRotMat(p_est(2))*v_cmd;
    std::cout<<v_cmd<<std::endl;
}

Vector4d pd_control::inverse_kinematics(Vector3d vel_cmd)
{
    return InvJacob * vel_cmd;
}

Vector4d pd_control::clamp(Vector4d motor_input)
{
    double max_val = motor_input.maxCoeff();
    double max_val2 = fabs(motor_input.minCoeff());

    if( max_val2 > max_val)
        max_val = max_val2;
    
    if( max_val > motor_vel_lim)
        motor_input = motor_input/max_val*motor_vel_lim;

    return motor_input;
}

void pd_control::cmd_vel_publish()
{
    vel ethercat_motor_vel;

    curr_time = ros::Time::now().toSec();
    dt = curr_time - last_time;
    get_SLAM_pose();
    error_calculation();
    cmd_vel_calculation();
    motor_vel_input = inverse_kinematics(v_cmd);
    motor_vel_input = clamp(motor_vel_input*gear_ratio*radps_to_rpm);

    if( distance(goal_pose,p_est) < 0.001 || control_enable == false)
    {
        control_enable = false;
        motor_vel_input<<0,0,0,0;
        p_est = goal_pose;
        v_cmd<<0, 0, 0;
        v_cmd_prev<<0, 0, 0;
    }

    ethercat_motor_vel.velocity[0] = (int) motor_vel_input(0);
    ethercat_motor_vel.velocity[1] = (int) -motor_vel_input(1);
    ethercat_motor_vel.velocity[2] = (int) -motor_vel_input(2);
    ethercat_motor_vel.velocity[3] = (int) motor_vel_input(3);
    
    publisher_cmd_vel.publish(ethercat_motor_vel);

    last_time = curr_time;
}

Matrix3d pd_control::getRotMat(double yaw)
{
    Matrix3d R;
    R << cos(yaw), sin(yaw), 0,
        -sin(yaw), cos(yaw), 0,
        0, 0, 1;
    return R; 
}

void pd_control::angleNormalizer(double &yaw)
{
    yaw = atan2(sin(yaw),cos(yaw));
}

double pd_control::distance(Vector3d goal, Vector3d est)
{
    return sqrt(pow(goal(0)-est(0),2)+pow(goal(1)-est(1),2));
}