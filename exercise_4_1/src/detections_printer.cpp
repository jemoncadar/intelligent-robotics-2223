#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"


void chatterCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
	ROS_INFO("Received tag detection!");

	const auto& array = msg.detections;
	
	for(apriltag_ros::AprilTagDetection detection : array)
	{
		std::string idsString = "";
		std::string sizesString = "";

		const auto& ids = detection.id;
		const auto& sizes = detection.size;

		geometry_msgs::PoseWithCovarianceStamped pose = detection.pose;
		geometry_msgs::PoseWithCovariance poseWithCovariance = pose.pose;

		const auto& covariance = poseWithCovariance.covariance; // PRINT
		geometry_msgs::Pose poseAndOrientation = poseWithCovariance.pose;
		geometry_msgs::Point position = poseAndOrientation.position; // PRINT
		geometry_msgs::Quaternion orientation = poseAndOrientation.orientation; // PRINT


		for (int id : ids)
		{
			idsString += std::to_string(id) + " ";
		}
		ROS_INFO("\tIDs [%s]", idsString.c_str());

		for (double size : sizes)
		{
			sizesString += std::to_string(size)  + " ";
		}
		ROS_INFO("\tsizes [%s]", sizesString.c_str());

		// ROS_INFO("%s", pose);

		for(float item : covariance)
		{
			ROS_INFO("%f", item);
		}

		ROS_INFO("POSITION: (%f, %f, %f)", position.x, position.y, position.z);
		ROS_INFO("ORIENTATION: (%f, %f, %f, %f)", orientation.x, orientation.y, orientation.z, orientation.w);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "detections_printer_node");
	
	ROS_INFO("Se ha inicializado el nodo");
	
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tag_detections",1000, chatterCallback);
	
	ros::spin();
	
	return 0;
}
