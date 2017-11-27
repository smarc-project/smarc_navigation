#include "ekf_class/ekf_class.hpp"

GeneralEKF::GeneralEKF(std::string node_name, ros::NodeHandle &nh):node_name_(node_name), nh_(&nh)
{
    double initial_R;
    double initial_Q;
    double delta;

    // Topics and services

    // EKF initial params
    nh_->param<bool>((ros::this_node::getName() + "/sequential_update"), sequential_update_, true);
    nh_->param<int>((ros::this_node::getName() + "/state_vec_size"), size_n_, 3);
    nh_->param<int>((ros::this_node::getName() + "/measurement_vec_size"), size_k_, 2);
    nh_->param<double>((ros::this_node::getName() + "/initial_R"), initial_R, 0.1);
    nh_->param<double>((ros::this_node::getName() + "/initial_Q"), initial_Q, 0.1);
    nh_->param<double>((ros::this_node::getName() + "/delta"), delta, 0.999);

    // Initialize params
    init(initial_R, initial_Q, delta, size_n_, size_k_);
}

GeneralEKF::~GeneralEKF(){
    // TODO_NACHO: do some cleaning up here
}


void GeneralEKF::init(const double &initial_R, const double &initial_Q, const double &delta, const unsigned int &size_n, const unsigned int &size_k){

    // Initial estimate of the state
    mu_ = boost::numeric::ublas::zero_vector<double>(size_n);
    sigma_ = boost::numeric::ublas::identity_matrix<double>(size_n) * 0.0000000001;
    // Noise models params
    R_ = boost::numeric::ublas::identity_matrix<double> (size_n, size_n) * initial_R;
    Q_ = boost::numeric::ublas::identity_matrix<double> (size_k, size_k) * initial_Q;
    // Outlier rejection threshold
    delta_m_ = delta;
    boost::math::chi_squared chi2_dist(2);
    lambda_M_ = boost::math::quantile(chi2_dist, delta_m_);
    // Initial control variables
    u_t_ =boost::numeric::ublas::vector<double>(size_n);
}

void GeneralEKF::computeOdom(){

}

void GeneralEKF::predictionStep(){
    // Compute jacobian of the prediction model


    // Prediction (mu_hat_, sigma_hat_)

}


void GeneralEKF::predictMeasurementModel(){
}

void GeneralEKF::dataAssociation(){

}

void GeneralEKF::sequentialUpdate(){

}

void GeneralEKF::ekfLocalize(){

    ros::Rate rate(10);

    ROS_INFO("Initialized");
    ROS_INFO("-------------------------");
    int cnt = 0;
    bool loop = true;
    while(ros::ok()&& loop){
        ros::spinOnce();
        // If queue of sensors not empty (change content of if!!
        if(true){
            ROS_INFO("*************************");
            ROS_INFO("Sensors readings received");
            ROS_INFO("*************************");
            // Read next sensor msgs from queue

            ROS_DEBUG("----Computing odometry----");
            this->computeOdom();
            ROS_DEBUG("----Prediction step----");
            this->predictionStep();
            ROS_DEBUG("----Data association----");
            this->dataAssociation();
            ROS_DEBUG("----Sequential update----");
            this->sequentialUpdate();

            // Publish output data

            // Remove sensor msgs utilized from queue

            std::cout << "number of loops:" << cnt++ << std::endl;

        }
        else{
            ROS_INFO("Still waiting for some good, nice sensor readings...");
        }
        rate.sleep();
    }
}

