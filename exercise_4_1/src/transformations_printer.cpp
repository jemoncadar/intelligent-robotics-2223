#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "var.h"

geometry_msgs::TransformStamped transformStamped;

void chatterCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
	ROS_INFO("Received tag detection!");

	const auto& array = msg.detections;
	
	for(apriltag_ros::AprilTagDetection detection : array)
	{
		geometry_msgs::PoseWithCovarianceStamped pose = detection.pose;
		geometry_msgs::PoseWithCovariance poseWithCovariance = pose.pose;

		const auto& covariance = poseWithCovariance.covariance;
		geometry_msgs::Pose poseAndOrientation = poseWithCovariance.pose;

		//TODO: transformation
        tf2::doTransform(poseAndOrientation, poseAndOrientation, transformStamped);

        geometry_msgs::Point position = poseAndOrientation.position;
		geometry_msgs::Quaternion orientation = poseAndOrientation.orientation;
        ROS_INFO("\tPOSITION: (%f, %f, %f)", position.x, position.y, position.z);
		ROS_INFO("\tORIENTATION: (%f, %f, %f, %f)", orientation.x, orientation.y, orientation.z, orientation.w);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "transformations_printer_node");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tag_detections", 1000, chatterCallback);

	ROS_INFO("Node started");
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	bool gotTransformation = false;

	while(!gotTransformation && ros::ok())
	{
		try
		{
			transformStamped = tfBuffer.lookupTransform("base_link", "camera_base",
			ros::Time(0));
			gotTransformation = true;
			ROS_INFO("Got transformation!");
		}
		catch (tf2::TransformException &ex) 
		{
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	ros::spin();
	
	return 0;
}