#include "localization.hpp"

Localization::Localization(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh)
{
    std::string path_2_map;
    std::string sens_top_name;
    std::string map_srv_name;
    std::string nu_est_top;
    std::string nu_real_top;
    std::string error_top;

    double initial_R;
    double initial_Q;
    double delta;

    // Topics and services
    nh_->param<std::string>((ros::this_node::getName() + "/path_2_map"), path_2_map, "/home/nacho/Documents/PhDCourses/AppliedEstimation/Lab1_EKF/DataSets/map_o3.txt");
    nh_->param<std::string>((ros::this_node::getName() + "/sens_top_name"), sens_top_name, "/sensors_sim");
    nh_->param<std::string>((ros::this_node::getName() + "/map_srv_name"), map_srv_name, "/ekf_output_srv");
    nh_->param<std::string>((ros::this_node::getName() + "/nu_est_top"), nu_est_top, "/nu_estimated");
    nh_->param<std::string>((ros::this_node::getName() + "/nu_real_top"), nu_real_top, "/nu_real");
    nh_->param<std::string>((ros::this_node::getName() + "/error_top"), error_top, "/angle_error");

    // EKF initial params
    nh_->param<double>((ros::this_node::getName() + "/initial_R"), initial_R, 0.1);
    nh_->param<double>((ros::this_node::getName() + "/initial_Q"), initial_Q, 0.1);
    nh_->param<double>((ros::this_node::getName() + "/delta"), delta, 0.999);

    sens_sub_ = nh_->subscribe(sens_top_name, 50, &Localization::sensorsCB, this);
    map_client_ = nh_->serviceClient<ekf_general::plot_map>(map_srv_name);
    nu_est_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(nu_est_top, 20);
    nu_real_pub_ = nh_->advertise<std_msgs::Float32MultiArray>(nu_real_top, 20);
    error_pub_ = nh_->advertise<std_msgs::Float32>(error_top, 10);

    // Read map file
    getMapFile(path_2_map);
    // Initialize params
    init(initial_R, initial_Q, delta);
    // MAIN LOOP
    ekfLocalize();
}

void Localization::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

void Localization::getMapFile(std::__cxx11::string path_to_map){
    std::ifstream map_file;
    std::string line;
    std::vector<std::string> landmark;
    boost::numeric::ublas::vector<int> aux_vec(3);
    ekf_general::plot_map map_srv;

    // Read map from .txt
    try{
        ROS_INFO("Opening map");
        map_file.open(path_to_map, std::ios::in);

    }catch(std::ios_base::failure & fail){
        std::cout << "Could not open file" << std::endl;
    }
    while (std::getline(map_file,line))
    {
        split(line, ' ', landmark);
        aux_vec(0) = std::stoi(landmark.at(0));
        aux_vec(1) = std::stoi(landmark.at(1));
        aux_vec(2) = std::stoi(landmark.at(2));
        map_srv.request.map_landmarks.push_back(aux_vec(1));
        map_srv.request.map_landmarks.push_back(aux_vec(2));
        map_.push_back(aux_vec);
        landmark.clear();
    }
    map_file.close();

    // Plot map service request
    while(!map_client_.call(map_srv) && ros::ok()){
        ros::Duration(0.5).sleep();
        ROS_INFO("Waiting for map plotter service");
    }
    ROS_INFO("Map plot service called");

}

void Localization::sensorsCB(const ekf_general::sensors_read::Ptr& sensor_msg){
    msgs_queue_.push(sensor_msg);
}

void Localization::init(const double &initial_R, const double &initial_Q, const double &delta){

    // Initial estimate of the state
    mu_ = boost::numeric::ublas::zero_vector<double>(3);
    sigma_ = boost::numeric::ublas::identity_matrix<double>(3) * 0.0000000001;
    R_ = boost::numeric::ublas::identity_matrix<double> (3, 3) * initial_R;
    Q_ = boost::numeric::ublas::identity_matrix<double> (2, 2) * initial_Q; //TODO_NACHO: tune noise models
    // Outlier rejection variables
    delta_m_ = delta;
    boost::math::chi_squared chi2_dist(2);
    lambda_M_ = boost::math::quantile(chi2_dist, delta_m_);
    // Initial control variables
    t_prev_ = 0;
    u_t_ =boost::numeric::ublas::vector<double>(3);
    encoders_prev_ = boost::numeric::ublas::vector<int>(2);
    encoders_prev_(0) = 0;
    encoders_prev_(1) = 0;
}

void Localization::computeOdom(){
    double E_T = 2048.0;
    double B = 0.35;
    double R_L = 0.1;
    double R_R = R_L;

    // Update incremental variables
    double t_now = sensor_in_->acq_time;
    double delta_t = t_now - t_prev_;

    int delta_enc_L = sensor_in_->encoders.at(1) - encoders_prev_(1);
    int delta_enc_R = sensor_in_->encoders.at(0) - encoders_prev_(0);
    double omega_R_t = 2*M_PI*(delta_enc_R)/(E_T*delta_t);
    double omega_L_t = 2*M_PI*(delta_enc_L)/(E_T*delta_t);
    double omega_t = (omega_R_t*R_R - omega_L_t*R_L)/B;
    double vel_t = (omega_R_t*R_R + omega_L_t*R_L)/2;

    u_t_(0) = vel_t*delta_t*std::cos(mu_(2));
    u_t_(1) = vel_t*delta_t*std::sin(mu_(2));
    u_t_(2) = angleLimit(omega_t*delta_t);

    t_prev_ += delta_t;
    encoders_prev_(1) = sensor_in_->encoders.at(1);
    encoders_prev_(0) = sensor_in_->encoders.at(0);
}

void Localization::predictionStep(){
    // Compute jacobian of the prediction model
    boost::numeric::ublas::matrix<double> G_t = boost::numeric::ublas::identity_matrix<double>(3,3);
    G_t(0,2) = -1*u_t_(1);
    G_t(1,2) = u_t_(0);

    // Prediction (keep theta in the limits)
    mu_hat_ =  mu_ + u_t_;
    mu_hat_(2) = angleLimit(mu_hat_(2));
    boost::numeric::ublas::matrix<double> aux = boost::numeric::ublas::prod(G_t, sigma_);
    sigma_hat_ = boost::numeric::ublas::prod(aux, boost::numeric::ublas::trans(G_t));
    sigma_hat_ += R_;
}


void Localization::predictMeasurementModel(const boost::numeric::ublas::vector<int>& landmark_j,
                                      boost::numeric::ublas::vector<double>& z_i,
                                      std::vector<LandmarkML*> &ml_i_list){
    // Predicted z_hat_t_j
    LandmarkML *landmark_j_ptr;
    boost::numeric::ublas::vector<double> z_hat (2);
    z_hat(0) = std::sqrt(std::pow((landmark_j(1) - mu_hat_(0)),2) + std::pow((landmark_j(2) - mu_hat_(1)),2));
    z_hat(1) = angleLimit(std::atan2(landmark_j(2) - mu_hat_(1), landmark_j(1) - mu_hat_(0)) - mu_hat_(2));

    // Compute ML of observation z_i with M_j
    landmark_j_ptr = new LandmarkML(landmark_j);
    landmark_j_ptr->computeH(z_hat, mu_hat_);
    landmark_j_ptr->computeS(sigma_hat_, Q_);
    landmark_j_ptr->computeNu(z_hat, z_i);
    landmark_j_ptr->computeLikelihood();

    // Outlier detection based on Mahalanobis distance (z_i, z_j_hat)
//    std::cout << "Likelihood of landmark j=" << landmark_j_ptr->landmark_id_ << std::endl;
//    std::cout << landmark_j_ptr->psi_ << std::endl;
    if(landmark_j_ptr->d_m_ < lambda_M_){
//        std::cout << "Adding landmark j=" << landmark_j_ptr->landmark_id_ << std::endl;
//        std::cout << landmark_j_ptr->d_m_ << std::endl;
        ml_i_list.push_back(landmark_j_ptr);
    }
}

void Localization::dataAssociation(std::vector<LandmarkML*> &ml_t_list){
    int num_observs = sensor_in_->n;
    boost::numeric::ublas::vector<double> z_i(2);
    std::vector<boost::numeric::ublas::vector<double>> z_t;
    for(int i=0; i<num_observs; ++i){
        z_i(0) = sensor_in_->ranges.at(i);
        z_i(1) = angleLimit(sensor_in_->bearings.at(i));
        z_t.push_back(z_i);
    }
    // Main ML loop
    std::vector<LandmarkML*> ml_i_list;
    // For each observation z_i at time t
    for(auto z_i_j: z_t){
        // For each possible landmark j in M
        for(auto landmark_j: map_){
            predictMeasurementModel(landmark_j, z_i_j, ml_i_list);
        }
        // Select the association with the maximum likelihood
        if(!ml_i_list.empty()){
            if(ml_i_list.size() > 1){
                std::sort(ml_i_list.begin(), ml_i_list.end(), sortLandmarksML);
                double psi_min = ml_i_list.back()->psi_;
                double psi_max = (ml_i_list.front()->psi_ <= 0.01)? 0: ml_i_list.front()->psi_;
                if(psi_max != psi_min){
                    std::cout << "************************ " << std::endl;
                    std::cout << "ML min: " << psi_min << "ML max: " << psi_max << std::endl;
                    for(auto ml_i: ml_i_list){
                        std::cout << "ML values: " << ml_i->psi_ << std::endl;
                        ml_i->psi_ = (ml_i->psi_ - psi_min)/(psi_max - psi_min);
                        std::cout << "Normalized values: " << ml_i->psi_ << std::endl;
                    }
                }
                std::sort(ml_i_list.begin(), ml_i_list.end(), sortLandmarksML);
            }
            std::cout << "psi_ value selected: " << ml_i_list.front()->psi_ << std::endl;
            ml_t_list.push_back(ml_i_list.front());
        }
        ml_i_list.clear();
    }
    std::cout << "Num of assoc vs expected number: " << ml_t_list.size() << "-"<< sensor_in_->ids.size() << std::endl;
    int j_ids = 0;
    for(auto observ: ml_t_list){
        std::cout << "Obtained vs known association: " << observ->landmark_id_ << ", " <<sensor_in_->ids.at(j_ids) << std::endl;
        std::cout << observ->d_m_ << std::endl;
        j_ids++;
    }
}

bool sortLandmarksML(LandmarkML *ml_1, LandmarkML *ml_2){

    return (ml_1->psi_ > ml_2->psi_)? true: false;
}

void Localization::sequentialUpdate(std::vector<LandmarkML *> &observ_list){
    using namespace boost::numeric::ublas;
    matrix<double> K_t_i;
    matrix<double> H_trans;
    identity_matrix<double> I(sigma_hat_.size1(), sigma_hat_.size2());
    matrix<double> aux_mat;

    // Sequential update for each observation i in t
    for(auto observ_i: observ_list){
        // Compute Kalman gain
        H_trans = trans(observ_i->H_);
        K_t_i = prod(sigma_hat_, H_trans);
        K_t_i = prod(K_t_i, observ_i->S_inverted_);
        // Update mu_hat and sigma_hat
        mu_hat_ = mu_hat_ + prod(K_t_i, observ_i->nu_);
        mu_hat_(2) = angleLimit(mu_hat_(2));
        aux_mat = (I  - prod(K_t_i, observ_i->H_));
        sigma_hat_ = prod(aux_mat, sigma_hat_);
    }

    // State estimates for time t
    mu_ = mu_hat_;
    mu_(2) = angleLimit(mu_(2));
    sigma_ = sigma_hat_;
//    std::cout << "mu_" << std::endl;
//    std::cout << mu_ << std::endl;
//    std::cout << "real mu_" << std::endl;
//    std::cout << sensor_in_->true_pose.at(0) <<"," << sensor_in_->true_pose.at(1) <<"," << sensor_in_->true_pose.at(2)  << std::endl;
//    std::cout << "-------------------" << std::endl;
//    std::cout << "error in the angle:" << mu_(2) - sensor_in_->true_pose.at(2)  << std::endl;
}

void Localization::ekfLocalize(){

    std::vector<LandmarkML*> observs_list_t;
    boost::numeric::ublas::vector<double> mu_real = boost::numeric::ublas::vector<double>(3);
    ros::Rate rate(10);

    std_msgs::Float32MultiArray mu_est_msg;
    mu_est_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mu_est_msg.layout.dim[0].label = "mu_";
    mu_est_msg.layout.dim[0].size = mu_.size();
    mu_est_msg.layout.dim[0].stride = 1;

    std_msgs::Float32MultiArray mu_real_msg;
    mu_real_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    mu_real_msg.layout.dim[0].label = "mu_real";
    mu_real_msg.layout.dim[0].size = mu_.size();
    mu_real_msg.layout.dim[0].stride = 1;

    std_msgs::Float32 angle_error_msg;

    ROS_INFO("Initialized");
    ROS_INFO("-------------------------");
    int cnt = 0;
    bool loop = true;
    while(ros::ok()&& loop){
        ros::spinOnce();
        if(!msgs_queue_.empty()){
            ROS_INFO("*************************");
            ROS_INFO("Sensors readings received");
            ROS_INFO("*************************");
            sensor_in_ = msgs_queue_.front();
            ROS_DEBUG("----Computing odometry----");
            computeOdom();
            ROS_DEBUG("----Prediction step----");
            predictionStep();
            ROS_DEBUG("----Data association----");
            dataAssociation(observs_list_t);
            ROS_DEBUG("----Sequential update----");
            sequentialUpdate(observs_list_t);

            // Publish mu_t
            mu_est_msg.data.clear();
            mu_est_msg.data.insert(mu_est_msg.data.end(), mu_.begin(), mu_.end());
            nu_est_pub_.publish(mu_est_msg);

            // Publish real mu_t
            mu_real(0) = sensor_in_->true_pose.at(0);
            mu_real(1) = sensor_in_->true_pose.at(1);
            mu_real(2) = sensor_in_->true_pose.at(2);

            mu_real_msg.data.clear();
            mu_real_msg.data.insert(mu_real_msg.data.end(), mu_real.begin(), mu_real.end());
            nu_real_pub_.publish(mu_real_msg);

            angle_error_msg.data = mu_(2) - sensor_in_->true_pose.at(2);
            error_pub_.publish(angle_error_msg);

            observs_list_t.clear();
            msgs_queue_.pop();

            std::cout << "number of loops:" << cnt++ << std::endl;

        }
        else{
            ROS_INFO("Still waiting for some good, nice sensor readings...");
        }
        rate.sleep();
    }
}

