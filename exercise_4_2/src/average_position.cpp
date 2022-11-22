#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"

double x_sum;
double y_sum;
double z_sum;

double x_mean;
double y_mean;
double z_mean;

int numberDetections = 0;

bool first_detection = true;

void chatterCallbackTagDetections(const apriltag_ros::AprilTagDetectionArray& msg)
{
	ROS_INFO("Received tag detection!");

	const auto& array = msg.detections;
	
	for(apriltag_ros::AprilTagDetection detection : array)
	{
		geometry_msgs::PoseWithCovarianceStamped pose = detection.pose;
		geometry_msgs::PoseWithCovariance poseWithCovariance = pose.pose;

		geometry_msgs::Pose poseAndOrientation = poseWithCovariance.pose;
		geometry_msgs::Point position = poseAndOrientation.position;

        x_sum += position.x;
        y_sum += position.y;
        z_sum += position.z;

        numberDetections++;

        x_mean = x_sum/numberDetections;
        y_mean = y_sum/numberDetections;
        z_mean = z_sum/numberDetections;

        ROS_INFO("AVERAGE POSITION: (%f, %f, %f)", x_mean, y_mean, z_mean);
	}
}

void chatterCallbackImages(const sensor_msgs::Image& img)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "average_position_node");
	
	ROS_INFO("Node started");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tag_detections",1000, chatterCallbackTagDetections);
    ros::Subscriber subImages = n.subscribe("kinect/rgb/image_rect/color",1000, chatterCallbackImages);
	
	ros::spin();
	
	return 0;
}
