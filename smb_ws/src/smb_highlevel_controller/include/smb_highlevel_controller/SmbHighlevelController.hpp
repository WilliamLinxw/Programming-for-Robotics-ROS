#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>


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
};

} /* namespace */
