#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <algorithm>

namespace smb_highlevel_controller
{

  SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
  {
    std::string topic;
    int queue_size;
    if (!nodeHandle.getParam("topic", topic))
    {
      ROS_ERROR("Could not find topic parameter!");
    }
    if (!nodeHandle.getParam("queue_size", queue_size))
    {
      ROS_ERROR("Could not find queue_size parameter!");
    }
    ROS_INFO("%s", topic.c_str());
    ROS_INFO("%d", queue_size);
    subscriber_ = nodeHandle_.subscribe(topic, queue_size, &SmbHighlevelController::topicCallback, this);
    sub_pointcloud = nodeHandle_.subscribe("/rslidar_points", 10, &SmbHighlevelController::pointcloudCallback, this);
    ROS_INFO("Successfully launched node.");
  }

  SmbHighlevelController::~SmbHighlevelController()
  {
  }

  void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // float array_size = sizeof(msg->ranges);
    // float bit_size = sizeof((msg->ranges)[0]);
    int size = msg->ranges.size();

    // auto start = std::begin(msg->ranges);
    // auto end = start + size;
    // auto min = std::min_element(start, end);
    // for(int )

    float min = msg->ranges[0];
    for (int i = 1; i <= size - 1; i++)
    {
      // ROS_INFO("range: (%d, %f)", i, min);
      if (msg->ranges[i] < min)
      {
        min = msg->ranges[i];
      }
    }
    ROS_INFO("min range: %f", min);
    // ROS_INFO("full_size: %f, bit_size: %f, size: %d", array_size, bit_size, size);
  }

  void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
  {
    int height = cloud_msg->height;
    int width = cloud_msg->width;
    int num = height*width;
    ROS_INFO("height: %d; width: %d; total: %d", height, width, num);
  }

} /* namespace */
