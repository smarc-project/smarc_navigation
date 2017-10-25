#include "localization.hpp"

Localization::Localization(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh)
{
    std::string sens_top_name = "sensors_sim";
    ros::Subscriber sens_sub = nh_->subscribe(sens_top_name, 10, &Localization::sensorsCB, this);

    // MAIN LOOP
    init();
    ekfLocalize();
}

void Localization::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

double Localization::angleLimit (double angle){ // keep angle within [0;2*pi)
        while (angle >= M_PI2)
            angle -= M_PI2;
        while (angle < 0)
            angle += M_PI2;
        return angle;
}

void Localization::readMapFile(std::__cxx11::string path_to_map){
    std::ifstream map_file;
    std::string line;
    std::vector<std::string> landmark;
    boost::numeric::ublas::vector<int> aux_vec(2);

    try{
        map_file.open(path_to_map, std::ios::in);
    }catch(std::ios_base::failure & fail){
        std::cout << "Could not open file" << std::endl;
    }

    while (std::getline(map_file,line))
    {
        split(line, ' ', landmark);
        aux_vec(0) = std::stoi(landmark.at(1));
        aux_vec(1) = std::stoi(landmark.at(2));
        map_.push_back(aux_vec);
    }
    map_file.close();
}

void Localization::sensorsCB(const ekf_general::sensors_read::Ptr& sensor_msg){
    msgs_queue_.push(sensor_msg);
}

void Localization::init(){
    // Initial estimate of the state
    readMapFile("/home/nacho/Documents/PhDCourses/AppliedEstimation/Lab1_EKF/DataSets/map_o3.txt");
    mu_ = boost::numeric::ublas::zero_vector<double>(3);
    sigma_ = boost::numeric::ublas::identity_matrix<double>(3) * 0.0000000001;
    delta_m_ = 0.999;
    boost::math::chi_squared chi2_dist(2);
    lambda_M_ = boost::math::quantile(chi2_dist, delta_m_);
    R_ = boost::numeric::ublas::identity_matrix<double> (3, 3) * 0.01;
    Q_ = boost::numeric::ublas::identity_matrix<double> (2, 2) * 0.01; //TODO_NACHO: tune covariances

    // Initialize control variables
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
    t_now_ = sensor_in_->acq_time;
    double delta_t = t_now_ - t_prev_;
    int delta_enc_R = sensor_in_->encoders.at(0) - encoders_prev_(0);
    int delta_enc_L = sensor_in_->encoders.at(1) - encoders_prev_(1);
    // TODO_NACHO: encoders left and right pos in the vector??
    double omega_R_t = 2*M_PI*(delta_enc_R)/E_T*(delta_t);
    double omega_L_t = 2*M_PI*(delta_enc_L)/E_T*(delta_t);
    double omega_t = omega_R_t*R_R - omega_L_t*R_L/B;
    double vel_t = omega_R_t*R_R + omega_L_t*R_L/2;

    u_t_(0) = vel_t*delta_t*std::cos(mu_(2));
    u_t_(1) = vel_t*delta_t*std::sin(mu_(2));
    u_t_(2) = angleLimit(omega_t*delta_t);

    t_prev_ += delta_t;
}

void Localization::predictionStep(){
    // Compute prediction model jacobian
    boost::numeric::ublas::matrix<double> G_t(3,3);
    G_t = boost::numeric::ublas::identity_matrix<double>(3,3);
    G_t(0,2) = -1*u_t_(1);
    G_t(1,2) = u_t_(0);

    // Prediction
    mu_hat_ =  mu_ + u_t_;
    boost::numeric::ublas::matrix<double> aux = boost::numeric::ublas::prod(G_t, sigma_);
    sigma_hat_ = boost::numeric::ublas::prod(aux, boost::numeric::ublas::trans(G_t));
    sigma_hat_ += R_;

}

void Localization::dataAssociation(){
    // TODO_NACHO: add upper loop for every ith observation per measurement
    int num_observs = sensor_in_->n;
    boost::numeric::ublas::vector<double> z_i(2);
    std::vector<boost::numeric::ublas::vector<double>> z_t;
    for(int i=0; i<num_observs; ++i){
        z_i(0) = sensor_in_->ranges.at(i);
        z_i(1) = sensor_in_->bearings.at(i);
        z_t.push_back(z_i);
    }

    // Main ML loop
    unsigned int j = 0;
    // For each observation z_i at time t
    for(auto z_i_j: z_t){
        // For each possible landmark j in M
        for(auto landmark_j: map_){
            predictMeasurementModel(j, landmark_j, z_i_j);
            j++;
        }
        // Select the association with the ML
        // TODO_NACHO:
        std::cout << "Measurements predicted" << std::endl;
        std::cout << ml_i_list_.size() << std::endl;
        std::sort(ml_i_list_.begin(), ml_i_list_.end(), sortLandmarksML);
        ml_t_list_.push_back(ml_i_list_.front());
        // Clear rest of objects from aux list i
        ml_i_list_.clear();
    }
}

void Localization::predictMeasurementModel(unsigned int &j,
                                      const boost::numeric::ublas::vector<int>& landmark_j,
                                      boost::numeric::ublas::vector<double>& z_i){
    // Predicted z_hat_t_j
    boost::numeric::ublas::vector<double> z_hat (2);
    z_hat(0) = std::sqrt(std::pow((landmark_j(0) - mu_hat_(0)),2) + std::pow((landmark_j(1) - mu_hat_(1)),2));
    z_hat(1) = std::atan2(landmark_j(1) - mu_hat_(1), landmark_j(0) - mu_hat_(0)) - mu_hat_(2);
    // Jacobian of h for t, j and M
    landmark_j_ptr_ = new LandmarkML(j, landmark_j);
    landmark_j_ptr_->computeH(z_hat, mu_hat_);
    landmark_j_ptr_->computeS(sigma_, Q_);
    // TODO_NACHO: implement z_i as a smart pointer?
    landmark_j_ptr_->computeNu(z_hat, z_i);
    landmark_j_ptr_->computeLikelihood();
    // Outlier detection based on Mahalanobis distance (z_i, z_j_hat)
    // TODO_NACHO: correct outlier rejection!!!!!
    if(landmark_j_ptr_->d_m_(0) > lambda_M_){
        std::cout << "Adding landmark association" << std::endl;
        ml_i_list_.push_back(landmark_j_ptr_);
    }
    else{
        std::cout << "Landmark association outlier" << std::endl;
    }
}

bool sortLandmarksML(LandmarkML *ml_1, LandmarkML *ml_2){

    return (ml_1->psi_ > ml_2->psi_)? true: false;
}

void Localization::sequentialUpdate(){

}

void Localization::ekfLocalize(){

    ros::Rate rate(1.0);
    while(ros::ok()){
        ros::spinOnce();
        if(!msgs_queue_.empty()){
            ROS_INFO("Sensors readings received");
            sensor_in_ = msgs_queue_.front();
            ROS_INFO("Sensors readings handled");
            computeOdom();
            ROS_INFO("Odometry computed");
            predictionStep();
            ROS_INFO("Prediction step carried out");
            dataAssociation();
            ROS_INFO("Data association finished");
            sequentialUpdate();
            ROS_INFO("Sequential update acomplished");
            ml_t_list_.clear();
        }
        else{
            ROS_INFO("Still waiting for some good, nice sensor readings...");
        }
        rate.sleep();
    }
    // NACHO_TODO: do some clean up here
}

