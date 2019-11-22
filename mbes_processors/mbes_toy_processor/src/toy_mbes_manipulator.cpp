/* Copyright 2018 Ignacio Torroba (ignaciotb@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "toy_mbes_receptor/toy_mbes_manipulator.hpp"
#include <numeric>


SonarManipulator::SonarManipulator(){}
SonarManipulator::~SonarManipulator(){}

void SonarManipulator::processSonarInput(const sensor_msgs::LaserScanConstPtr &mbes_msg){

    // Print raw input
//        std::for_each(mbes_msg->intensities.begin(),mbes_msg->intensities.end(), print);
//        std::cout << std::endl;

    // Mean filter to smooth intensities
    std::vector<double> smoothed;
    std::vector<double> mask {1/5.0,1/5.0,1/5.0,1/5.0,1/5.0};
    std::vector<double>aux;
    smoothed.push_back(mbes_msg->intensities.at(0));
    smoothed.push_back(mbes_msg->intensities.at(1));
    for(unsigned int i=2; i<mbes_msg->intensities.size()-2; i++){
        aux = {mbes_msg->intensities.at(i-2),
                mbes_msg->intensities.at(i-1),
                mbes_msg->intensities.at(i),
                mbes_msg->intensities.at(i+1),
                mbes_msg->intensities.at(i+2)};
        smoothed.push_back(std::inner_product(aux.begin(), aux.end(), mask.begin(), 0));
    }
    smoothed.push_back(mbes_msg->intensities.at(mbes_msg->intensities.size()-2));
    smoothed.push_back(mbes_msg->intensities.at(mbes_msg->intensities.size()-1));

    // Collect beams with intensity over threshold
    std::vector<int> targets_poses;
    double mean_ints = std::accumulate(smoothed.begin(), smoothed.end(), 0.0);
    mean_ints = mean_ints/smoothed.size();
    std::vector<double>::iterator it_max = std::max_element(smoothed.begin(), smoothed.end());
    double int_thres = (mean_ints >= *it_max*0.9 && mean_ints <= *it_max*1.1)? mbes_msg->range_max*10: mean_ints;
    int i = 0;
    std::for_each(smoothed.begin(), smoothed.end(), [&targets_poses, &i, &int_thres](const double &intensity_i){
            double input = (intensity_i >= int_thres*1.05)? i: 0;
            targets_poses.push_back(input);
            i++;
    });

    // If any higher intensity value detected
    std::vector<double> cluster_i;
    for(unsigned int i=0; i<targets_poses.size(); i++){
        if(targets_poses.at(i) != 0){
            cluster_i.push_back(targets_poses.at(i));
        }
        else{
            if(!cluster_i.empty()){
                if(cluster_i.size()>1){
                    // Compute polar coordinates of landmark
                    int reminder = cluster_i.size()%2;
                    int landmark_idx = (reminder == 0)? cluster_i.at((cluster_i.size()/2)): cluster_i.at(((cluster_i.size()+1)/2));
                    double alpha = mbes_msg->angle_min + mbes_msg->angle_increment * landmark_idx;
                    // Store new vector3 with landmark coordinates
                    tf::Vector3 point = tf::Vector3(mbes_msg->ranges.at(landmark_idx) * std::cos(alpha),
                               mbes_msg->ranges.at(landmark_idx) * std::sin(alpha),
                               0);
                    this->landmarks_.push_back(point);
                }
                // Empty cache
                cluster_i.clear();
            }
        }
    }
}



