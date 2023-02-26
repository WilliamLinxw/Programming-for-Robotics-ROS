#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <visualization_msgs/Marker.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>



namespace smb_highlevel_controller {

/*!
 * Class containing the Husky Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	ros::NodeHandle nodeHandle_;

	void topicCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

	ros::Subscriber subscriber_;

	ros::Subscriber sub_pointcloud;

	ros::Publisher go_to_pillar;

	ros::Publisher vis_pub;

	bool turned;

	bool arrived;

	int proportional;

	int queue_size;

	std::string topic;

	int command_to_go;
};

} /* namespace */
