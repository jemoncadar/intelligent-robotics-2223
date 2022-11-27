#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// Global variables to manage average
double x_sum;
double y_sum;
double z_sum;

double x_mean;
double y_mean;
double z_mean;

int numberDetections = 0;

bool first_detection = true;

std::vector<double> cameraK;
std::vector<double> cameraD;

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
    const auto& rvec = (cv::Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    const auto& tvec = (cv::Mat_<float>(3,1) << x_mean, y_mean, z_mean);

    //cv::projectPoints( InputArray objectPoints,InputArray rvec, InputArray tvec, InputArray cameraMatrix, InputArray distCoeffs, 
    //OutputArray imagePoints, OutputArray jacobian=noArray(), double aspectRatio=0);

    // Necesitamos "cameraMatrix" y "distCoeffs" que se encuentran en el topic "/kinect/rgb/camera_info". Debemos suscribirnos
    // a dicho topic también
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "average_position_node");
	
	ROS_INFO("Node started");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tag_detections",1000, chatterCallbackTagDetections);
    ros::Subscriber subImages = n.subscribe("kinect/rgb/image_rect/color",1000, chatterCallbackImages);
	
    //Receiving camera info
    boost::shared_ptr<sensor_msgs::CameraInfo const> spCameraInfo;
    sensor_msgs::CameraInfo cameraInfo;
    spCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect/rgb/camera_info");
    if(spCameraInfo != NULL){
        cameraInfo = *spCameraInfo;
    }
    ROS_INFO("Got camera info!");
    //cameraK = cameraInfo.K;
    //for (double d : cameraK)
    //{
    //    ROS_INFO("%f", d);
    //}
    //cameraD = cameraInfo.D;

	ros::spin();
	
	return 0;
}
