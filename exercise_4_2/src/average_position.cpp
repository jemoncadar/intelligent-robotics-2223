#include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

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

std::vector<unsigned int> outputImage;

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

void chatterCallbackImages(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Got image!");

    const auto& rvec = (cv::Mat_<float>(3,1) << 0.0, 0.0, 0.0);
    const auto& tvec = (cv::Mat_<float>(3,1) << x_mean, y_mean, z_mean);

    /*cv_bridge::CvImagePtr cvImagePtr;
    try {
        cvImagePtr = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }*/
    //https://answers.ros.org/question/241499/cmakeliststxt-and-packagexml-files-of-ros-image-to-cv-image-conversion/
    //http://www.cs.cmu.edu/~cga/xxx/notes-images.pdf
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "average_position_node");
	
	ROS_INFO("Node started");

	ros::NodeHandle n;
	ros::Subscriber subTags = n.subscribe("tag_detections",1000, chatterCallbackTagDetections);

    //ros::Subscriber subImages = n.subscribe("kinect/rgb/image_rect/color",1000, chatterCallbackImages);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber subImages = it.subscribe("kinect/rgb/image_rect_color", 1000, chatterCallbackImages);
	
    //Receiving camera info
    boost::shared_ptr<sensor_msgs::CameraInfo const> spCameraInfo;
    sensor_msgs::CameraInfo cameraInfo;
    spCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/kinect/rgb/camera_info");
    if(spCameraInfo != NULL){
        cameraInfo = *spCameraInfo;
    }
    ROS_INFO("Got camera info!");
    const auto& cameraInfoD = cameraInfo.D;
    for (double e : cameraInfoD)
    {
        cameraD.push_back(e);
    }

    const auto& cameraInfoK = cameraInfo.K;
    for (double e : cameraInfoK)
    {
        cameraK.push_back(e);
    }

	ros::spin();
	
	return 0;
}
