#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"

void chatterCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
	const auto& array = msg.detections;
	
	for(apriltag_ros::AprilTagDetection detection : array)
	{
		const auto& ids = detection.id;
		const auto& sizes = detection.size;
		geometry_msgs::PoseWithCovarianceStamped pose = detection.pose;
		for (int id : ids)
		{
			ROS_INFO("%d", id);
		}
		for (double size : sizes)
		{
			ROS_INFO("%f", size);
		}
		ROS_INFO("%s", pose);
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "client_node");
	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tag_detections",1000, chatterCallback);
	
	ros::spin();
	
	return 0;
}
