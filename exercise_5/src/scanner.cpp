#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>

void chatterCallback(const sensor_msgs::LaserScan& msg)
{
	ROS_INFO("Received scan message!");

    ROS_INFO("\tangle_min: %f", msg.angle_min);
    ROS_INFO("\tangle_max: %f", msg.angle_max);
    ROS_INFO("\tangle_increment: %f", msg.angle_increment);
    ROS_INFO("\ttime_increment: %f", msg.time_increment);
    ROS_INFO("\tscan_time: %f", msg.scan_time);
    ROS_INFO("\trange_min: %f", msg.range_min);
    ROS_INFO("\trange_max: %f", msg.range_max);

    const auto& ranges = msg.ranges;
    const auto& intensities = msg.intensities;

    std::string rangesStr = "";
    for (float r : ranges)
    {
        if (!isinf(r)) // if range is not inf -> print
        {
            rangesStr += std::to_string(r) + " ";
        }
    }
    ROS_INFO("\tranges: [%s]", rangesStr.c_str());

    std::string intensitiesStr = "";
    for (float i : intensities)
    {
        if (i > 0) // if intensity is not 0 -> print
        {
            intensitiesStr += std::to_string(i) + " ";
        }
    }
    ROS_INFO("\tintensities: [%s]", intensitiesStr.c_str());

    std::vector <float> pol_distances;
    std::vector <float> pol_angles;
    for (int i = 0; i < ranges.size(); i++)
    {
        float r = ranges.at(i);
        if (!isinf(r)) // if range is not inf
        {
            pol_distances.push_back(r);
            pol_angles.push_back(msg.angle_min + (float)i * msg.angle_increment);
        }
    }

    for (int i = 0; i < pol_distances.size(); i++)
    {
        ROS_INFO("\t\tPOINT (polar): (%f, %f)", pol_angles.at(i), pol_distances.at(i));
    }

    // We have all the points (in polar coordinates) in the two vectors: 
    // pol_distances and pol_angles
    // The first point, for example, is (pol_distances.at(0), pol_angles.at(0))

    // Next, we will transform the given polar coordinates to cartesian coordinates

    std::vector <float> x_coordinates;
    std::vector <float> y_coordinates;

    float x;
    float y;

    for (int i = 0; i < pol_distances.size(); i++)
    {
        x = pol_distances.at(i) * cos(pol_angles.at(i));
        y = pol_distances.at(i) * sin(pol_angles.at(i));

        x_coordinates.push_back(x);
        y_coordinates.push_back(y);
    }

    // Printing the cartesian coordinates

    for (int i = 0; i < x_coordinates.size(); i++)
    {
        ROS_INFO("\t\tPOINT (cartesian): (%f, %f)", x_coordinates.at(i), y_coordinates.at(i));
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scanner_node");
	
	ROS_INFO("Node started");

	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan",1000, chatterCallback);
	
	ros::spin();
	
	return 0;
}
