#include <mobile_control/wheel_odometry.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odom");
    WheelOdometry wheel_odom;

    wheel_odom.initiate_variables();
    wheel_odom.publisher_set();
    wheel_odom.subscriber_set();
    
    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        
        wheel_odom.velocity_pose_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}