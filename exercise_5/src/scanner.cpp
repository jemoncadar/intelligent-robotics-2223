#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>

const float THRESHOLD = 0.6;

float vecMean(std::vector<float> vector)
{
    float sum = 0.0;
    for (float e : vector)
    {
        sum += e;
    }
    return sum / vector.size();
}

bool isInThreshold(float currentX, float currentY, float lastX, float lastY)
{
    return sqrt(pow(currentX - lastX,2) + pow(currentY - lastY,2)) < THRESHOLD;
}

void printLaserScan(const sensor_msgs::LaserScan& msg)
{
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
}

void printCoordinates(std::string title, std::vector<float> c1, std::vector<float> c2)
{
    ROS_INFO("%s", title.c_str());
    for (int i = 0; i < c1.size(); i++)
    {
        ROS_INFO("\tPOINT: (%f, %f)", c1.at(i), c2.at(i));
    }
}

void chatterCallback(const sensor_msgs::LaserScan& msg)
{
	ROS_INFO("Received scan message!");

    //printLaserScan(msg); //Print all info

    const auto& ranges = msg.ranges;
    const auto& intensities = msg.intensities;

    //Get all polar coordinates in (pol_distances, pol_angles)
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

    //printCoordinates("Polar coordinates", pol_distances, pol_angles);
    // The first point, for example, is (pol_distances.at(0), pol_angles.at(0))

    //Get all cartesians coordinates in (x_coordinates, y_coordinates)
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

    //printCoordinates("Cartesians coordinates", x_coordinates, y_coordinates);

    // Now we iterate through all the points (cartesian coordinates) to detect
    // the people recognized by the scanner
    int i = 1;
    float current_x; //x_coordinates.at(i)
    float current_y; //y_coordinates.at(i)

    std::vector<std::vector<float>> peopleX;
    std::vector<std::vector<float>> peopleY;

    while(i < x_coordinates.size())
    {
        current_x = x_coordinates.at(i);
        current_y = y_coordinates.at(i);

        //ROS_INFO("Point (%f, %f)", current_x, current_y);
        if (peopleX.empty()) { //No person registered yet
            //ROS_INFO("Created new person");
            std::vector<float> newPersonX;
            std::vector<float> newPersonY;

            newPersonX.push_back(current_x);
            newPersonY.push_back(current_y);

            peopleX.push_back(newPersonX);
            peopleY.push_back(newPersonY);
        } else {
            //ROS_INFO("Searching person where it fits");
            bool fitted = false;
            int j = 0;
            while (j < peopleX.size() && !fitted) //For every person registered
            {
                std::vector<float> currentPersonX = peopleX.at(j);
                std::vector<float> currentPersonY = peopleY.at(j);
                if (isInThreshold(current_x, current_y, vecMean(currentPersonX), vecMean(currentPersonY)))
                {
                    //ROS_INFO("Added in person %d", j);
                    fitted = true;
                    currentPersonX.push_back(current_x);
                    currentPersonY.push_back(current_y);
                }
                j++;
            }
            if (!fitted) //This point belogs to a new person
            {
                //ROS_INFO("Doesn't fit in any person, creating one");
                std::vector<float> newPersonX;
                std::vector<float> newPersonY;

                newPersonX.push_back(current_x);
                newPersonY.push_back(current_y);

                peopleX.push_back(newPersonX);
                peopleY.push_back(newPersonY);
            }
        }
        i++;
    }

    ROS_INFO("People detected: %d", peopleX.size());
    for (int i = 0; i < peopleX.size(); i++)
    {
        std::vector<float> personX = peopleX.at(i);
        std::vector<float> personY = peopleY.at(i);

        ROS_INFO("Person %d at (%4.2f, %4.2f)", (i+1), vecMean(personX), vecMean(personY));
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
