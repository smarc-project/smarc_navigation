#include "localization.hpp"

Localization::Localization(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh)
{
    std::string sens_top_name = "sensors_sim";
    ros::Subscriber sens_sub = nh_->subscribe(sens_top_name, 50, &Localization::sensorsCB, this);

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
        landmark.clear();
    }
    map_file.close();
}

void Localization::sensorsCB(const ekf_general::sensors_read::Ptr& sensor_msg){
    msgs_queue_.push(sensor_msg);
}

void Localization::init(){
    // Read map file
    readMapFile("/home/nacho/Documents/PhDCourses/AppliedEstimation/Lab1_EKF/DataSets/map_o3.txt");

    // Initial estimate of the state
    mu_ = boost::numeric::ublas::zero_vector<double>(3);
    sigma_ = boost::numeric::ublas::identity_matrix<double>(3) * 0.0000000001;
    delta_m_ = 0.999;
    boost::math::chi_squared chi2_dist(2);
    lambda_M_ = boost::math::quantile(chi2_dist, delta_m_);
    std::cout << "Lambda" << std::endl;
    std::cout << lambda_M_ << std::endl;

    R_ = boost::numeric::ublas::identity_matrix<double> (3, 3) * 0.001;
    Q_ = boost::numeric::ublas::identity_matrix<double> (2, 2) * 0.001; //TODO_NACHO: tune covariances

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
    double t_now = sensor_in_->acq_time;
    double delta_t = t_now - t_prev_;
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
    std::cout << "mu hat" << std::endl;
    std::cout << mu_hat_ << std::endl;
    boost::numeric::ublas::matrix<double> aux = boost::numeric::ublas::prod(G_t, sigma_);
    sigma_hat_ = boost::numeric::ublas::prod(aux, boost::numeric::ublas::trans(G_t));
    sigma_hat_ += R_;
    std::cout << "Sigma hat" << std::endl;
    std::cout << sigma_hat_ << std::endl;
}

void Localization::dataAssociation(std::vector<LandmarkML*> &ml_t_list){
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
    std::vector<LandmarkML*> ml_i_list;
    // For each observation z_i at time t
    for(auto z_i_j: z_t){
        // For each possible landmark j in M
        for(auto landmark_j: map_){
            predictMeasurementModel(j, landmark_j, z_i_j, ml_i_list);
            j++;
        }
        // Select the association with the ML
        std::cout << "Number of possible associations for z_i and M_j" << std::endl;
        std::cout << ml_i_list.size() << std::endl;
        std::sort(ml_i_list.begin(), ml_i_list.end(), sortLandmarksML);
        ml_t_list.push_back(ml_i_list.front());
        // Clear rest of objects from aux list i
        ml_i_list.clear();
    }
    std::cout << "Number of observations in t" << std::endl;
    std::cout << ml_t_list.size() << std::endl;
}

void Localization::predictMeasurementModel(unsigned int &j,
                                      const boost::numeric::ublas::vector<int>& landmark_j,
                                      boost::numeric::ublas::vector<double>& z_i,
                                      std::vector<LandmarkML*> &ml_i_list){
    // Predicted z_hat_t_j
    LandmarkML *landmark_j_ptr;
    boost::numeric::ublas::vector<double> z_hat (2);
    z_hat(0) = std::sqrt(std::pow((landmark_j(0) - mu_hat_(0)),2) + std::pow((landmark_j(1) - mu_hat_(1)),2));
    z_hat(1) = angleLimit(std::atan2(landmark_j(1) - mu_hat_(1), landmark_j(0) - mu_hat_(0)) - mu_hat_(2));

    landmark_j_ptr = new LandmarkML(j, landmark_j);
    landmark_j_ptr->computeH(z_hat, mu_hat_);
    landmark_j_ptr->computeS(sigma_, Q_);
    // TODO_NACHO: implement z_i as a smart pointer?
    landmark_j_ptr->computeNu(z_hat, z_i);
    landmark_j_ptr->computeLikelihood();

    // Outlier detection based on Mahalanobis distance (z_i, z_j_hat)
    // TODO_NACHO: correct outlier rejection!!!!!
    std::cout << "Likelihood of landmark j=" << landmark_j_ptr->landmark_id_ << std::endl;
    std::cout << landmark_j_ptr->d_m_(0) << std::endl;
    if(landmark_j_ptr->d_m_(0) < lambda_M_){
        ml_i_list.push_back(landmark_j_ptr);
    }
    else{
        //std::cout << "Landmark association outlier" << std::endl;
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
        // Update mu and sigma
        mu_hat_ = mu_hat_ + prod(K_t_i, observ_i->nu_);
        aux_mat = (I  - prod(K_t_i, observ_i->H_));
        sigma_hat_ = prod(aux_mat, sigma_hat_);
    }
    // Store state estimates for time t
    mu_ = mu_hat_;
    sigma_ = sigma_hat_;
}

void Localization::ekfLocalize(){

    std::vector<LandmarkML*> observs_list_t;
    ros::Rate rate(0.5);
    ROS_INFO("Initialized");
    int cnt = 0;
    while(ros::ok()){
        ros::spinOnce();
        if(!msgs_queue_.empty()){
            ROS_INFO("*************************");
            ROS_INFO("Sensors readings received");
            ROS_INFO("-------------------------");
            sensor_in_ = msgs_queue_.front();
            computeOdom();
            ROS_INFO("Odometry computed");
            ROS_INFO("-------------------------");
            predictionStep();
            ROS_INFO("Prediction step carried out");
            ROS_INFO("-------------------------");
            dataAssociation(observs_list_t);
            ROS_INFO("Data association finished");
            ROS_INFO("-------------------------");
            sequentialUpdate(observs_list_t);
            ROS_INFO("Sequential update acomplished");
            ROS_INFO("-------------------------");
            observs_list_t.clear();
            msgs_queue_.pop();
            std::cout << "number of loops:" << cnt+1 << std::endl;
        }
        else{
            ROS_INFO("Still waiting for some good, nice sensor readings...");
        }
        rate.sleep();
    }
    // NACHO_TODO: do some clean up here
}

