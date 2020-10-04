#include <mobile_control/pd_control.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pd_controller");

    pd_control pd_controller;

    pd_controller.init_Variables();
    pd_controller.subscriber_set();
    pd_controller.publisher_set();

    ros::Rate loop_rate(200);

    while (ros::ok())
    {
        pd_controller.cmd_vel_publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
        
}