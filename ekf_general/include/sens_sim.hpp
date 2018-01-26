#ifndef SENS_SIM_HPP
#define SENS_SIM_HPP

#include <ros/ros.h>
#include <ekf_general/sensors_read.h>

#include <fstream>
#include <iostream>
#include <queue>

#include <boost/array.hpp>
#include <boost/scoped_ptr.hpp>

typedef struct{
    float t;
    boost::array<float, 3> odom;
    boost::array<int, 2> encoders;
    boost::array<float, 3> true_pose;
    int n;
    std::vector<int> ids;
    std::vector<double> bearings;
    std::vector<double> ranges;
} sensor_in_t;


class SensSim{
public:
    SensSim(std::string node_name, ros::NodeHandle &nh);
    void readSensorFile(std::string &path_to_sensors);

private:
    ros::NodeHandle *nh_;
    std::string node_name_;
    std::queue<sensor_in_t> sensors_readings_; //TODO_NACHO: change to queue of pointers?
    void split(const std::string &s, char delim, std::vector<std::string> &elems);
};

#endif // SENS_SIM_HPP
