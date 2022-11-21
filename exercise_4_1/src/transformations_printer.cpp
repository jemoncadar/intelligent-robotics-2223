#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "var.h"

tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

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
        geometry_msgs::TransformStamped transformStamped;
        
        try
        {
            transformStamped = tfBuffer->lookupTransform("camera_base", "base_link",
            ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        tf2::doTransform(poseAndOrientation, poseAndOrientation, transformStamped);

        geometry_msgs::Point position = poseAndOrientation.position;
		geometry_msgs::Quaternion orientation = poseAndOrientation.orientation;
        ROS_INFO("POSITION: (%f, %f, %f)", position.x, position.y, position.z);
		ROS_INFO("ORIENTATION: (%f, %f, %f, %f)", orientation.x, orientation.y, orientation.z, orientation.w);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "transformations_printer_node");
	
    //tfBuffer = new tf2_ros::Buffer();
    //tfListener = new tf2_ros::TransformListener(&tfBuffer);

	ROS_INFO("Se ha inicializado el nodo");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tag_detections",1000, chatterCallback);
	
	ros::spin();
	
	return 0;
}