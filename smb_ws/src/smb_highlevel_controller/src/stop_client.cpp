#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/LaserScan.h>

ros::ServiceClient client;

void topicCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    int size = msg->ranges.size();
    float angle_inc = msg->angle_increment;

    float min = msg->ranges[0];
    int index = 0;
    for (int i = 1; i <= size - 1; i++)
    {
        if (msg->ranges[i] < min)
        {
            min = msg->ranges[i];
            index = i;
        }
    }
    float angle_from_right = index * angle_inc;
    float angle_to_middle = -M_PI / 2 + angle_from_right;
    ROS_INFO("min range: %f; index: %d", min, index);
    ROS_INFO("angle to middle: %f", angle_to_middle);

    if (min < 10)
    {
        std_srvs::SetBool srv;
        srv.request.data = true;
        if (client.call(srv))
        {
            ROS_INFO("Success? %d", srv.response.success);
        }
        else{
            ROS_ERROR("Failed to call service");
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stop_client");
    ros::NodeHandle nh("~");

    client = nh.serviceClient<std_srvs::SetBool>("/stop_request");

    ros::Subscriber subscriber = nh.subscribe("/scan", 10, topicCallback);

    ros::spin();
    return 0;
}