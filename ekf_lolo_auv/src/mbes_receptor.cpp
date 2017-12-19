#include "mbes_receptor/mbes_receptor.hpp"


MBESReceptor::MBESReceptor(std::string node_name, ros::NodeHandle &nh): node_name_(node_name), nh_(&nh){

    std::string mbes_left_tp;
    std::string mbes_right_tp;
    std::string pcl_out_top;

    nh_->param<std::string>((node_name_ + "/mbes_left_topic"), mbes_left_tp, "/sss_left");
    nh_->param<std::string>((node_name_ + "/mbes_right_topic"), mbes_right_tp, "/sss_right");
    nh_->param<std::string>((node_name_ + "/pcl_out"), pcl_out_top, "/pcl_out");

    // Synch reception of mbes msgs
    mbes_l_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_, mbes_left_tp, 20);
    mbes_r_subs_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(*nh_, mbes_right_tp, 20);
    mbes_synch_ = new message_filters::Synchronizer<MyTimerPolicy>(MyTimerPolicy(50), *mbes_l_subs_, *mbes_r_subs_);
    mbes_synch_->registerCallback(boost::bind(&MBESReceptor::mbesReadingsCB, this, _1, _2));

    // RVIZ pcl output for testing
    pcl_pub_ = nh_->advertise<pcl::PointCloud<pcl::PointXYZ>>(pcl_out_top, 10);
    landmark_pub_ = nh_->advertise<geometry_msgs::PointStamped>("/lolo_auv/landmark_detected", 10);
}

auto print = [](const double& n) { std::cout << " " << n; };

void MBESReceptor::mbesReadingsCB(const sensor_msgs::LaserScanConstPtr &mbes_l_msg,
                                  const sensor_msgs::LaserScanConstPtr &mbes_r_msg){

    // Compute mean of the intensities
    double mean_r = std::accumulate(mbes_r_msg->intensities.begin(), mbes_r_msg->intensities.end(), 0) / mbes_r_msg->intensities.size();
    std::vector<double> intensities_right;
    std::vector<double> edges_right;
    std::vector<double> mask {-1,0,1};

    // Substract the mean to each value
    std::for_each(mbes_r_msg->intensities.begin(), mbes_r_msg->intensities.end(), [&mean_r, &intensities_right](const double& n) {
        intensities_right.push_back(n - mean_r);
    });
    std::cout << std::endl;

    // Prewitt mask for edges
    edges_right.push_back(0);
    for(unsigned int i=1; i<intensities_right.size() -1; i++){
        std::vector<double>aux {intensities_right.at(i-1),
                                intensities_right.at(i),
                                intensities_right.at(i+1)};
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

    if(!target_pose.empty()){
//        std::cout << "Possible poses: " << std::endl;
//        std::for_each(target_pose.begin(), target_pose.end(), print);
//        std::cout << std::endl;

        int reminder = target_pose.size()%2;
        int landmark_idx = (reminder == 0)? target_pose.at((target_pose.size()/2)): target_pose.at(((target_pose.size()+1)/2));

        double alpha = mbes_r_msg->angle_min + mbes_r_msg->angle_increment * landmark_idx;
        std::cout << "alpha: " << alpha << std::endl;

        geometry_msgs::PointStamped landmark_msg;
        landmark_msg.header.stamp = mbes_r_msg->header.stamp;
        landmark_msg.header.frame_id = "lolo_auv/sonarright_link";
        landmark_msg.point.x = mbes_r_msg->ranges.at(landmark_idx) * std::cos(alpha);
        landmark_msg.point.y = mbes_r_msg->ranges.at(landmark_idx) * std::sin(alpha);
        landmark_msg.point.z = 0;
        landmark_pub_.publish(landmark_msg);
    }
}


MBESReceptor::~MBESReceptor(){

}

















