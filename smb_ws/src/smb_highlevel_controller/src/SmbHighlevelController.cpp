#include <smb_highlevel_controller/SmbHighlevelController.hpp>

namespace smb_highlevel_controller
{

  SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle)
  {
    if (!nodeHandle_.getParam("topic", topic))
    {
      ROS_ERROR("Could not find topic parameter!");
    }
    if (!nodeHandle_.getParam("queue_size", queue_size))
    {
      ROS_ERROR("Could not find queue_size parameter!");
    }
    if (!nodeHandle_.getParam("proportional", proportional))
    {
      ROS_ERROR("Could not find proportional parameter!");
    }
    if (!nodeHandle_.getParam("service", service_name))
    {
      ROS_ERROR("Could not find proportional parameter!");
    }

    subscriber_ = nodeHandle_.subscribe(topic, queue_size, &SmbHighlevelController::topicCallback, this);
    sub_pointcloud = nodeHandle_.subscribe("/rslidar_points", 10, &SmbHighlevelController::pointcloudCallback, this);
    go_to_pillar = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    vis_pub = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    stop_service = nodeHandle_.advertiseService(service_name, &SmbHighlevelController::stopCallback, this);

    turned = false;
    arrived = false;
    command_to_go = 11;
    stop = false;
    ROS_INFO("Successfully launched node.");
  }

  SmbHighlevelController::~SmbHighlevelController()
  {
  }

  bool SmbHighlevelController::stopCallback(std_srvs::SetBool::Request &request,
                                            std_srvs::SetBool::Response &response)
  {
    stop = request.data;
    response.success = true;
    response.message = "Service executed successfully";
    return true;
  }

  void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    // float array_size = sizeof(msg->ranges);
    // float bit_size = sizeof((msg->ranges)[0]);
    int size = msg->ranges.size();
    float angle_inc = msg->angle_increment;

    // auto start = std::begin(msg->ranges);
    // auto end = start + size;
    // auto min = std::min_element(start, end);
    // for(int )

    float min = msg->ranges[0];
    int index = 0;
    for (int i = 1; i <= size - 1; i++)
    {
      // ROS_INFO("range: (%d, %f)", i, min);
      if (msg->ranges[i] < min)
      {
        min = msg->ranges[i];
        index = i;
      }
    }
    float angle_from_right = index * angle_inc;
    float angle_to_middle = -M_PI / 2 + angle_from_right;
    ROS_INFO("min range: %f; index: %d", min, index);
    // ROS_INFO("angle increment: %f", angle_inc);
    // ROS_INFO("angle from right: %f", angle_from_right);
    ROS_INFO("angle to middle: %f", angle_to_middle);

    // Construct the command message
    geometry_msgs::Twist velo_command;

    if (!stop)
    {
      velo_command.angular.z = angle_to_middle * proportional;
      if (min > 2)
      {
        velo_command.linear.x = 0.5;
      }
      else
      {
        velo_command.linear.x = 0.0;
        velo_command.angular.z = angle_to_middle * 0.1 * proportional;
      }

      go_to_pillar.publish(velo_command);
    }
    else
    {
      velo_command.linear.x = 0.0;
      velo_command.linear.y = 0.0;
      velo_command.angular.z = 0.0;
      go_to_pillar.publish(velo_command);
    }

    // if (!turned)
    // {
    //   if (abs(angle_to_middle) > 0.01)
    //   {
    //     velo_command.angular.z = proportional * angle_to_middle;
    //     ROS_INFO("angular velocity command: %f", velo_command.angular.z);
    //     go_to_pillar.publish(velo_command);
    //   }
    //   else
    //   {
    //     velo_command.angular.z = 0;
    //     go_to_pillar.publish(velo_command);
    //     turned = true;
    //     ROS_INFO("Turned");
    //   }
    // }
    // else
    // {
    //   velo_command.linear.x = 1;
    //   go_to_pillar.publish(velo_command);
    //   ROS_INFO("linear velocity command: %f", velo_command.linear.x);
    //   if (abs(min - 0.9) < 0.05)
    //   {
    //     arrived = true;
    //   }
    //   if (arrived == true && command_to_go >= 0)
    //   {
    //     velo_command.linear.x = 1;
    //     command_to_go--;
    //     go_to_pillar.publish(velo_command);
    //   }
    //   else if (arrived == true && command_to_go < 0)
    //   {
    //     velo_command.linear.x = 0;
    //     go_to_pillar.publish(velo_command);
    //     ROS_INFO("--------------------Arrived-----------------------");
    //   }
    // }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "pillar";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (min)*sin(angle_from_right);
    marker.pose.position.y = -(min)*cos(angle_from_right);
    ;
    marker.pose.position.z = 0.2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish(marker);
    ROS_INFO("MARKER PUBLISHED");
  }

  void SmbHighlevelController::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
  {
    int height = cloud_msg->height;
    int width = cloud_msg->width;
    int num = height * width;
    // int size = cloud_msg->data.size();
    // int point_step = cloud_msg->point_step;
    ROS_INFO("height: %d; width: %d; total: %d", height, width, num);
    // ROS_INFO("point_step: %d", point_step);
    // ROS_INFO("total: %d", size);

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::fromROSMsg(*cloud_msg, cloud);

    // if (!cloud.empty())
    // {
    //     pcl::PointXYZ point = cloud.points[0];
    //     float x = point.x;
    //     float y = point.y;
    //     float z = point.z;
    //     ROS_INFO("Coordinate of the first point: (%f, %f, %f)", x, y, z);
    // }
  }

} /* namespace */
