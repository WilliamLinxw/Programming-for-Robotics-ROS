#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller
{

	/*!
	 * Class containing the Husky Highlevel Controller
	 */
	class SmbHighlevelController
	{
	public:
		/*!
		 * Constructor.
		 */
		SmbHighlevelController(ros::NodeHandle &nodeHandle);

		/*!
		 * Destructor.
		 */
		virtual ~SmbHighlevelController();

	private:
		ros::NodeHandle nodeHandle_;

		void topicCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

		void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg);

		bool stopCallback(std_srvs::SetBool::Request &request,
						  std_srvs::SetBool::Response &response);

		ros::Subscriber subscriber_;

		ros::Subscriber sub_pointcloud;

		ros::Publisher go_to_pillar;

		ros::Publisher vis_pub;

		ros::ServiceServer stop_service;

		bool turned;

		bool arrived;

		float proportional;

		int queue_size;

		std::string topic;

		std::string service_name;

		int command_to_go;

		bool stop;
	};

} /* namespace */
