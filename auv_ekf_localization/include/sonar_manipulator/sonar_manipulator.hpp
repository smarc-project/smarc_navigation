#ifndef SONAR_MANIPULATOR_HPP
#define SONAR_MANIPULATOR_HPP

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

    SonarManipulator(){}
    ~SonarManipulator(){}

    void processSonarInput(const sensor_msgs::LaserScanConstPtr &mbes_msg){

        //    std::cout << "SSS: " << std::endl;
        //    std::for_each(mbes_r_msg->intensities.begin(), mbes_r_msg->intensities.end(), print);
        //    std::cout << std::endl;

            // Mean filter
        //    std::vector<double> filtered_right;
        //    std::vector<double> median {0.333,0.333,0.333};
        //    filtered_right.push_back(0);
        //    for(unsigned int i=1; i<mbes_r_msg->intensities.size() -1; i++){
        //        std::vector<double>aux {mbes_r_msg->intensities.at(i-1),
        //                                mbes_r_msg->intensities.at(i),
        //                                mbes_r_msg->intensities.at(i+1)};
        //        filtered_right.push_back(std::inner_product(aux.begin(), aux.end(), median.begin(), 0));
        //    }
        //    filtered_right.push_back(0);

        std::vector<double> edges_right;
        std::vector<double> mask {-1,0,1};
        edges_right.push_back(0);
        for(unsigned int i=1; i<mbes_msg->intensities.size() -1; i++){
            std::vector<double>aux {mbes_msg->intensities.at(i-1),
                                    mbes_msg->intensities.at(i),
                                    mbes_msg->intensities.at(i+1)};
            edges_right.push_back(std::inner_product(aux.begin(), aux.end(), mask.begin(), 0));
        }
        edges_right.push_back(0);

        std::vector<int> target_pose;
        int i = 0;
        std::for_each(edges_right.begin(), edges_right.end(), [&target_pose, &i](const double &edge_i){
                if(edge_i > 2 || edge_i < -2){
                    target_pose.push_back(i);
                }
                i++;
        });

        // If any higher intensity value detected
        if(target_pose.size() > 1){
            // Compute polar coordinates of landmark
            int reminder = target_pose.size()%2;
            int landmark_idx = (reminder == 0)? target_pose.at((target_pose.size()/2)): target_pose.at(((target_pose.size()+1)/2));
            double alpha = mbes_msg->angle_min + mbes_msg->angle_increment * landmark_idx;

            tf::Vector3 point (mbes_msg->ranges.at(landmark_idx) * std::cos(alpha),
                       mbes_msg->ranges.at(landmark_idx) * std::sin(alpha),
                       0);
            landmarks_.push_back(point);
        }
    }

   std::vector<tf::Vector3> landmarks_;
};


#endif // SONAR_MANIPULATOR_HPP
