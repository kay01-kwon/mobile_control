
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Imu.h>
#include <ethercat_test/vel.h>
#include <std_msgs/String.h>
#include <sstream>

using namespace sensor_msgs;
using namespace ethercat_test;
using namespace message_filters;


class test{

    public:

    static void callback(const ImuConstPtr& imu_msg, const velConstPtr& vel_msg);
    private:

};

void test::callback(const ImuConstPtr& imu_msg, const velConstPtr& vel_msg)
{
    ROS_INFO("callback function");

}

void callback(const ImuConstPtr& imu_msg, const velConstPtr& vel_msg)
{
    ROS_INFO("callback function");

}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "message_filter_test");

    ros::NodeHandle nh;

    test X;

    message_filters::Subscriber<Imu> imu_sub(nh,"/imu/data",1);
    message_filters::Subscriber<vel> vel_sub(nh,"/measure",1);
    ros::Publisher publisher_test = nh.advertise<std_msgs::String>("chatter",1);
    TimeSynchronizer<Imu, vel> sync(imu_sub,vel_sub,10);
    sync.registerCallback(boost::bind(&X.callback,_1,_2));

    ros::Rate loop_rate(200);
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss<<"hello";

        msg.data = ss.str();

        ROS_INFO("%s",msg.data.c_str());
        publisher_test.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
}