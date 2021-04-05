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
#include <sim_control/slamposeMsg.h>
#include <sim_control/desiredMsg.h>

#define _USE_MATH_DEFINES

using teb_local_planner::TrajectoryPointMsg;
using teb_local_planner::FeedbackMsg;

using geometry_msgs::PoseStamped;
using tf::TransformListener;
using tf::StampedTransform;

using nav_msgs::Odometry;
using ethercat_test::vel;

using sim_control::slamposeMsg;
using sim_control::desiredMsg;

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
    void callback_traj(const TrajectoryPointMsg::ConstPtr& traj_msg);
    void callback_vel(const Odometry::ConstPtr& odom);
    void callback_goal(const PoseStamped::ConstPtr& goal_msg);
    void get_SLAM_pose();
    void get_base_arm_pose();
    void error_calculation();
    void cmd_vel_calculation();
    Vector4d inverse_kinematics(Vector3d vel_cmd);
    Vector4d clamp(Vector4d motor_input);
    void cmd_vel_publish();
    Matrix3d getRotMat(double yaw);
    void quat_to_eulerAngles(Quaterniond quat);
    void angleNormalizer(double &yaw);
    double distance(Vector3d goal, Vector3d est);
    bool motor_vel_check(Vector4d motor_input);
    void display_base_arm_pose(unsigned int &k);

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

	Vector3d base_arm_position;
	Quaterniond base_arm_quat = Quaterniond::Identity();
	Vector3d base_arm_orientation;

    Vector3d base_arm_position_avg;
    Quaterniond base_arm_quat_avg = Quaterniond::Identity();
	Vector3d base_arm_orientation_avg;

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
    const double gear_ratio = 73.5;
    const int motor_vel_lim = 6000;
    const double radps_to_rpm = 60/2.0/M_PI;

    // Controller PD Gain
    const double Kp_xy = 10;
    const double Kp_yaw = 1;

    const double Kd_xy = 3;
    const double Kd_yaw = 0.1;

    bool control_enable = true;

    unsigned int step = 0;

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

	base_arm_position.setZero();
	base_arm_orientation.setZero();

    base_arm_position_avg.setZero();
    base_arm_orientation_avg.setZero();

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
    publisher_error = nh_.advertise<Odometry>("/error_msg",1);
	publisher_slam_pose = nh_.advertise<slamposeMsg>("/slam_pose",1);
	publisher_des_traj = nh_.advertise<desiredMsg>("/des_traj",1);
}

void pd_control::subscriber_set()
{
    subscriber_traj = nh_.subscribe("/move_base/TebLocalPlannerROS/teb_feedback", 1, &pd_control::callback_traj, this);
    subscriber_vel = nh_.subscribe("/wheel_odom",1,&pd_control::callback_vel,this);
    subscriber_goal = nh_.subscribe("/move_base_simple/goal", 1, &pd_control::callback_goal, this);

}

// TrajectorPointMsg::ConstPtr , FeedbackMsg
void pd_control::callback_traj(const TrajectoryPointMsg::ConstPtr& traj_msg)
{
 
	desiredMsg des_traj_msg;
//TrajectoryPointMsg
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
	
	des_traj_msg.x_des = p_des(0);
	des_traj_msg.y_des = p_des(1);
	
	des_traj_msg.vx_des = v_des(0);
	des_traj_msg.vy_des = v_des(1);

	publisher_des_traj.publish(des_traj_msg);

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
   
	slamposeMsg pose_msg;

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

	pose_msg.x_est = p_est(0);
	pose_msg.y_est = p_est(1);
	pose_msg.phi_est = p_est(2);	

	publisher_slam_pose.publish(pose_msg);

}

void pd_control::get_base_arm_pose()
{
	StampedTransform transform_base_arm;
	try{

			listener_.lookupTransform("map","base_arm",ros::Time(0),transform_base_arm);
			tf::Quaternion base_arm_quat_ = transform_base_arm.getRotation();

			base_arm_quat.x() = base_arm_quat_[0];
			base_arm_quat.y() = base_arm_quat_[1];
			base_arm_quat.z() = base_arm_quat_[2];
			base_arm_quat.w() = base_arm_quat_[3];

            MatrixXd rotMat = base_arm_quat.toRotationMatrix();

			base_arm_position << transform_base_arm.getOrigin().x(),
						  		transform_base_arm.getOrigin().y(),
								transform_base_arm.getOrigin().z();
            quat_to_eulerAngles(base_arm_quat);
            
		}
	catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

}

void pd_control::error_calculation()
{
    Odometry error_msg;
    // p_err : Global frame, v_err : Gl frame
    p_err = p_des - p_est;
    v_err = v_des - getRotMat(-p_est(2))*v_est;

    error_msg.pose.pose.position.x = p_err(0);
    error_msg.pose.pose.position.y = p_err(1);
    
    angleNormalizer(p_err(2));

    publisher_error.publish(error_msg);
}

void pd_control::cmd_vel_calculation()
{
    a_cmd = a_des + Kp*p_err + Kd*v_err;
    v_cmd = v_cmd_prev + a_cmd*dt;
    v_cmd_prev = v_cmd;
    v_cmd = getRotMat(p_est(2))*v_cmd;
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
	get_base_arm_pose();
	error_calculation();
    cmd_vel_calculation();
    motor_vel_input = inverse_kinematics(v_cmd);
    motor_vel_input = clamp(motor_vel_input*gear_ratio*radps_to_rpm);

    if(control_enable == true)
        step = 0;

    if( distance(goal_pose,p_est) < 0.020 || control_enable == false)
    {
        if(motor_vel_check(motor_vel_input) || control_enable == false)
        {
            control_enable = false;
            motor_vel_input<<0,0,0,0;
            //p_est = goal_pose;
            v_cmd<<0, 0, 0;
            v_cmd_prev<<0, 0, 0;
            display_base_arm_pose(step);
			
        }

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

void pd_control::quat_to_eulerAngles(Quaterniond quat)
{
    base_arm_orientation<<atan2(2*(quat.w()*quat.x()+quat.y()*quat.z()),1-2*(pow(quat.x(),2)+pow(quat.y(),2))),
                        asin(2*(quat.w()*quat.y()-quat.z()*quat.x())),
                        atan2(2*(quat.w()*quat.z()+quat.x()*quat.y()),1-2*(pow(quat.y(),2)+pow(quat.z(),2)));
}

void pd_control::angleNormalizer(double &yaw)
{
    yaw = atan2(sin(yaw),cos(yaw));
}

double pd_control::distance(Vector3d goal, Vector3d est)
{
    return sqrt(pow(goal(0)-est(0),2)+pow(goal(1)-est(1),2));
}

bool pd_control::motor_vel_check(Vector4d motor_input)
{
    bool check;
    
    check = true;
    if(fabs(motor_input(0)) < 100 && fabs(motor_input(1)) < 100
    && fabs(motor_input(2)) < 100 && fabs(motor_input(3)) < 100)
    {
        check = false;
    }

    return check;
}

void pd_control::display_base_arm_pose(unsigned int &k)
{
    
    if(k == 1000)
    {
        base_arm_position_avg = base_arm_position;
        base_arm_orientation_avg = base_arm_orientation;
    }
    if(k > 1000)
    {
        base_arm_position_avg = (double) 1/(k-1000)*(base_arm_position + (k-1001)*base_arm_position_avg);
        base_arm_orientation_avg = (double) 1/(k-1000)*(base_arm_orientation + (k-1001)*base_arm_orientation_avg);
    }

    
    if(k%200 == 0 && k>1000)
    {
        ROS_INFO("Avg Position(x,y,z): %lf, %lf, %lf Avg Orientation(r p y): %lf, %lf, %lf",
        base_arm_position_avg(0),base_arm_position_avg(1),base_arm_position_avg(2),
        base_arm_orientation_avg(0),base_arm_orientation_avg(1),base_arm_orientation_avg(2));

        ROS_INFO("Position(x,y,z): %lf, %lf, %lf Orientation(r p y): %lf, %lf, %lf",
        base_arm_position(0),base_arm_position(1),base_arm_position(2),
        base_arm_orientation(0),base_arm_orientation(1),base_arm_orientation(2));
        
    }



    k++;

}