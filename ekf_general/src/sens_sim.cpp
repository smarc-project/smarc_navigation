#include "sens_sim.hpp"


SensSim::SensSim(std::__cxx11::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh){

    //TODO_NACHO: use a launch file
    std::string path = "/home/nacho/Documents/PhDCourses/AppliedEstimation/Lab1_EKF/DataSets/so_o3_ie.txt";
    std::string sensors_top = "sensors_sim";
    ros::Publisher sensors_pub = nh_->advertise<ekf_general::sensors_read>(sensors_top, 10);

    // Read input file with simulated readings
    readSensorFile(path);

    ekf_general::sensors_read sensors_msg;
    ros::Rate rate(1);
    boost::scoped_ptr<sensor_in_t> sensor_ptr(new sensor_in_t);

    int i=0;

    while(ros::ok() && !sensors_readings_.empty()){
        *sensor_ptr.get() = sensors_readings_.front();

        sensors_msg.header.stamp = ros::Time::now();
        sensors_msg.acq_time = sensor_ptr->t;
        sensors_msg.odom = sensor_ptr->odom;
        sensors_msg.encoders = sensor_ptr->encoders;
        sensors_msg.true_pose = sensor_ptr->true_pose;
        sensors_msg.n = sensor_ptr->n;
        sensors_msg.ids = sensor_ptr->ids;
        sensors_msg.bearings = sensor_ptr->bearings;
        sensors_msg.ranges = sensor_ptr->ranges;

        sensors_pub.publish(sensors_msg);
        sensors_readings_.pop();
        rate.sleep();
    }
}

void SensSim::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

void SensSim::readSensorFile(std::__cxx11::string &path_to_sensors){

    std::ifstream sensors_file;
    std::string line;
    std::vector<std::string> readings;
    sensor_in_t sensor_i;
    unsigned int i = 0;

    try{
        sensors_file.open(path_to_sensors, std::ios::in);
    }catch(std::ios_base::failure & fail){
        std::cout << "Could not open file" << std::endl;
    }

    while (std::getline(sensors_file,line))
    {
        split(line, ' ', readings);
        sensor_i.t = std::stof(readings.at(0));
        sensor_i.odom = {std::stof(readings.at(1)),
                        std::stof(readings.at(2)),
                        std::stof(readings.at(3))};
        sensor_i.encoders = {std::stoi(readings.at(4)),
                             std::stoi(readings.at(5))};
        sensor_i.true_pose = {std::stof(readings.at(6)),
                              std::stof(readings.at(7)),
                              std::stof(readings.at(8))};
        sensor_i.n = std::stoi(readings.at(9));

        if(sensor_i.n > 0){
            i = 10;
            while(i <= readings.size() - 2){
                sensor_i.ids.push_back(std::stoi(readings.at(i)));
                sensor_i.bearings.push_back(std::stod(readings.at(i + 1)));
                sensor_i.ranges.push_back(std::stod(readings.at(i + 2)));
                i = i + 3;
            }
        }
        else{
            sensor_i.ids.push_back(0);
            sensor_i.bearings.push_back(0);
            sensor_i.ranges.push_back(0);
        }

        // Store and clean up for next round
        sensors_readings_.push(sensor_i);
        sensor_i.ids.clear();
        sensor_i.bearings.clear();
        sensor_i.ranges.clear();
        readings.erase(readings.begin(), readings.end());
    }
    sensors_file.close();
}

