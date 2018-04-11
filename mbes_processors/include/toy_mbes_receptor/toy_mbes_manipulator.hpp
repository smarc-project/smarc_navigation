#ifndef TOY_MBES_MANIPULATOR_HPP
#define TOY_MBES_MANIPULATOR_HPP

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

/**
 * @brief The SonarManipulator class
 * Simple landmark extractor from LaserScan readings.
 * It outputs Vector3 positions in the sensor frame
 */

auto print = [](const double& n) { std::cout << " " << n; };

class SonarManipulator{

public:

    SonarManipulator();
    ~SonarManipulator();

    void processSonarInput(const sensor_msgs::LaserScanConstPtr &mbes_msg);

   std::vector<tf::Vector3> landmarks_;
};

#endif // TOY_MBES_MANIPULATOR_HPP
