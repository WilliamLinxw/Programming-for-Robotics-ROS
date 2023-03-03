#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Imu.h>

ros::ServiceClient client;

void topicCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    double x_acc = msg->linear_acceleration.x;

    double acc_mag = sqrt(x_acc*x_acc);

    ROS_INFO("acceleration: %f", acc_mag);

    if (acc_mag > 2)
    {
        ROS_WARN("Crash detected!");

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


    ros::Subscriber subscriber = nh.subscribe("/imu0", 10, topicCallback);

    ros::spin();
    return 0;
}