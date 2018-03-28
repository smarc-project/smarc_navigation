#include "ekf_slam_core/ekf_slam_core.hpp"

EKFCore::EKFCore(Eigen::VectorXd &mu, Eigen::MatrixXd &Sigma, Eigen::MatrixXd &R, Eigen::MatrixXd &Q, double &lambda, tf::StampedTransform &tf_base_sensor, const bool &mbes_input){

    // Initialize internal params
    mu_ = mu;
    mu_hat_ = mu_;
    mu_auv_odom_.setZero(3);
    Sigma_ = Sigma;
    Sigma_hat_ = Sigma_;
    R_ = R;
    Q_ = Q;
    lambda_M_ = lambda;
    lm_num_ = (mu_.rows() - 6) / 3;
    tf_base_sensor_ = tf_base_sensor;
    tf_sensor_base_ = tf_base_sensor.inverse();
    map_lm_num_ = lm_num_;
    mbes_input_ = mbes_input;
}


void EKFCore::predictMotion(nav_msgs::Odometry odom_reading){

    // Construct Fx (6,3N) for dimension mapping
    Eigen::SparseMatrix<double> F_x(6, 6 + 3*lm_num_);
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(6);
    unsigned int j;
    int input = 1;
    for(unsigned int i=0; i<6; i++){
        j = i;
        tripletList.push_back(Eigen::Triplet<double>(i,j,input));
    }
    F_x.setFromTriplets(tripletList.begin(), tripletList.end());
    Eigen::SparseMatrix<double> F_x_transp = F_x.transpose();

    // Extract latest orientation from odom at time t
    tf::Quaternion q_odom;
    tf::quaternionMsgToTF(odom_reading.pose.pose.orientation, q_odom);
    q_odom.normalize();

    // Compute predicted mu_hat
    Eigen::Vector3d u_t(odom_reading.pose.pose.position.x,
                        odom_reading.pose.pose.position.y,
                        odom_reading.pose.pose.position.z);

    u_t -= mu_auv_odom_;    // Increment in x,y,z
    mu_hat_.segment(0, 3) = mu_.segment(0,3);
    mu_hat_.segment(0, 3) += u_t;
    mu_auv_odom_ += u_t;

    // Global orientation roll, pitch and yaw
    tf::Quaternion q(
        odom_reading.pose.pose.orientation.x,
        odom_reading.pose.pose.orientation.y,
        odom_reading.pose.pose.orientation.z,
        odom_reading.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(mu_hat_(3), mu_hat_(4), mu_hat_(5));
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));

    // Derivative of motion model in mu_ (t-1)
    Eigen::MatrixXd g_t(6,6);
    g_t.setZero(6, 6);
    using namespace std;

    g_t(0,3) = u_t(1)*(sin(mu_hat_(3))*sin(mu_hat_(5)) + cos(mu_hat_(3))*cos(mu_hat_(5))*sin(mu_hat_(4)))
                + u_t(2)*(cos(mu_hat_(3))*sin(mu_hat_(5)) - cos(mu_hat_(5))*sin(mu_hat_(4))*sin(mu_hat_(3)));
    g_t(0,4) = cos(mu_hat_(5))*(u_t(2)*cos(mu_hat_(4))*cos(mu_hat_(3)) - u_t(0)*sin(mu_hat_(4))
                + u_t(1)*cos(mu_hat_(4))*sin(mu_hat_(3)));
    g_t(0,5) = u_t(2)*(cos(mu_hat_(5))*sin(mu_hat_(3)) - cos(mu_hat_(3))*sin(mu_hat_(4))*sin(mu_hat_(5)))
                - u_t(1)*(cos(mu_hat_(3))*cos(mu_hat_(5)) + sin(mu_hat_(4))*sin(mu_hat_(3))*sin(mu_hat_(5)))
                - u_t(0)*cos(mu_hat_(4))*sin(mu_hat_(5));

    g_t(1,3) = - u_t(1)*(cos(mu_hat_(5))*sin(mu_hat_(3)) - cos(mu_hat_(3))*sin(mu_hat_(4))*sin(mu_hat_(5)))
                - u_t(2)*(cos(mu_hat_(3))*cos(mu_hat_(5)) + sin(mu_hat_(4))*sin(mu_hat_(3))*sin(mu_hat_(5)));
    g_t(1,4) = sin(mu_hat_(5))*(u_t(2)*cos(mu_hat_(4))*cos(mu_hat_(3)) - u_t(0)*sin(mu_hat_(4))
                + u_t(1)*cos(mu_hat_(4))*sin(mu_hat_(3)));
    g_t(1,5) = u_t(2)*(sin(mu_hat_(3))*sin(mu_hat_(5)) + cos(mu_hat_(3))*cos(mu_hat_(5))*sin(mu_hat_(4)))
               - u_t(1)*(cos(mu_hat_(3))*sin(mu_hat_(5)) - cos(mu_hat_(5))*sin(mu_hat_(4))*sin(mu_hat_(3)))
               + u_t(0)*cos(mu_hat_(4))*cos(mu_hat_(5));

    g_t(2,3) = cos(mu_hat_(4))*(u_t(1)*cos(mu_hat_(3)) - u_t(2)*sin(mu_hat_(3)));
    g_t(2,4) = - u_t(0)*cos(mu_hat_(4)) - u_t(2)*cos(mu_hat_(3))*sin(mu_hat_(4))
                - u_t(1)*sin(mu_hat_(4))*sin(mu_hat_(3));
    g_t(2,5) = 0;

    // Compute Jacobian G_t
    Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(6 + 3*lm_num_, 6 + 3*lm_num_);
    G_t(3,3) = 0;   // G_t is zero here because the motion model uses abs values for RPY
    G_t(4,4) = 0;
    G_t(5,5) = 0;
    G_t += F_x_transp * g_t * F_x;

    // Predicted covariance matrix
    Sigma_hat_ = G_t * Sigma_ * G_t.transpose();
    Sigma_hat_ += F_x_transp * R_ * F_x;

}

void EKFCore::predictMeasurement(const Eigen::Vector3d &landmark_j,
                                 const Eigen::Vector3d &z_i,
                                 unsigned int i,
                                 unsigned int j,
                                 const tf::Transform &transf_base_map,
                                 const Eigen::MatrixXd &temp_sigma,
                                 h_comp h_comps,
                                 std::vector<CorrespondenceClass> &corresp_i_list){

    using namespace boost::numeric::ublas;
    tf::Vector3 landmark_j_map(landmark_j(0),
                                landmark_j(1),
                                landmark_j(2));


    // Measurement model: z_hat_i
    CorrespondenceClass corresp_i_j(i, j);
    if(mbes_input_){    // MBES
        tf::Vector3 z_hat_base = transf_base_map * landmark_j_map;
        Eigen::Vector3d z_k_hat_base( z_hat_base.getX(),
                                     z_hat_base.getY(),
                                     z_hat_base.getZ());

        // Compute ML of observation z_i with M_j
        corresp_i_j.computeH(h_comps, landmark_j_map, Eigen::Vector3d());
        corresp_i_j.computeNu(z_k_hat_base, z_i);
        corresp_i_j.computeMHLDistance(temp_sigma, Q_);
    }
    else {  // FLS
        double scaling = 400.0/17.0;    // TODO: check this value
        tf::Vector3 z_hat_fls = tf_sensor_base_ * transf_base_map * landmark_j_map;   // Expected meas in fls frame and m
        double arccos_azimut = 1/std::cos(std::atan2(z_hat_fls.getZ(), z_hat_fls.getX()));
        Eigen::Vector3d z_hat_fls_m(arccos_azimut * z_hat_fls.getX(), arccos_azimut * z_hat_fls.getY(), 0);
        Eigen::Vector3d z_hat_fls_pix = z_hat_fls_m * scaling;   // Expected meas in fls frame and pixels

        std::cout << "Predicted meas: " << z_hat_fls_pix << std::endl;
        std::cout << "Real meas: " << z_i << std::endl;

        // Compute ML of observation z_i with M_j
        corresp_i_j.computeH(h_comps, landmark_j_map, Eigen::Vector3d(z_hat_fls.getX(), z_hat_fls.getY(), z_hat_fls.getZ()));
        corresp_i_j.computeNu(z_hat_fls_pix, z_i);  // The innovation is now computed in pixels
        corresp_i_j.computeMHLDistance(temp_sigma, Q_);
    }

    ROS_INFO_STREAM("Mahalanobis dist: " << corresp_i_j.d_m_);

    // Outlier rejection
    if(corresp_i_j.d_m_ < lambda_M_){
        corresp_i_list.push_back(std::move(corresp_i_j));
    }
    else{
        ROS_INFO("Outlier rejected");
    }
}

void EKFCore::dataAssociation(std::vector<Eigen::Vector3d> z_t){

//    double epsilon = 9;
    double alpha = 0.5;   // TODO: find suitable value!!
    std::vector<CorrespondenceClass> corresp_i_list;
    tf::Transform transf_base_map;
    tf::Transform transf_map_base;
    Eigen::MatrixXd temp_sigma(9,9);

    double sigma_x, sigma_y;
    double sigma_uncertain = 30000;

    lm_num_ = (mu_.rows() - 6) / 3;

    tf::Vector3 new_lm_map;

    // For each observation z_i at time t
    for(unsigned int i = 0; i<z_t.size(); i++){
        // Compute transform map --> base from current state state estimate at time t
        transf_map_base = tf::Transform(tf::createQuaternionFromRPY(mu_hat_(3), mu_hat_(4), mu_hat_(5)).normalize(),
                                         tf::Vector3(mu_hat_(0), mu_hat_(1), mu_hat_(2)));
        transf_base_map = transf_map_base.inverse();

        // Back-project new possible landmark (in map frame)
        if(mbes_input_){    // MBES sensor input
            new_lm_map = transf_map_base * tf::Vector3(z_t.at(i)(0), z_t.at(i)(1),z_t.at(i)(2));

        }
        else{   // FLS sensor input
            tf::Vector3 new_lm_fls = tf::Vector3(z_t.at(i)(0), z_t.at(i)(1), z_t.at(i)(2));  // To polar coordinates to scale from pixels to meters
            double theta = std::atan2(new_lm_fls.getY(), new_lm_fls.getX());
            double rho = std::sqrt(std::pow(new_lm_fls.getX(),2) + std::pow(new_lm_fls.getY(),2));
            double rho_scaled = 17.0/400.0 * rho;

            tf::Vector3 lm_fls_real(rho_scaled * std::cos(theta),
                                    rho_scaled * std::sin(theta),
                                    0);

            // Transform to odom frame
            new_lm_map = transf_map_base  * tf_base_sensor_ *  lm_fls_real;
        }

        // Add new possible landmark to mu_hat_
        Eigen::VectorXd aux_mu = mu_hat_;
        mu_hat_.resize(mu_hat_.size()+3, true);
        mu_hat_ << aux_mu, Eigen::Vector3d(new_lm_map.getX(),
                                           new_lm_map.getY(),
                                           new_lm_map.getZ());

        // Increase Sigma_hat_
        sigma_x = sigma_uncertain * std::abs(std::cos(mu_hat_(5))) / (std::abs(std::cos(mu_hat_(5))) + std::abs(std::sin(mu_hat_(5))));
        sigma_y = sigma_uncertain * std::abs(std::sin(mu_hat_(5))) / (std::abs(std::cos(mu_hat_(5))) + std::abs(std::sin(mu_hat_(5))));

        Sigma_hat_.conservativeResize(Sigma_hat_.rows()+3, Sigma_hat_.cols()+3);
        Sigma_hat_.bottomRows(3).setZero();
        Sigma_hat_.rightCols(3).setZero();
        Sigma_hat_(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3) = sigma_x;  // TODO: initialize with uncertainty on the measurement in x,y,z
        Sigma_hat_(Sigma_hat_.rows()-2, Sigma_hat_.cols()-2) = sigma_y;
        Sigma_hat_(Sigma_hat_.rows()-1, Sigma_hat_.cols()-1) = 100;

        // Store current mu_hat_ estimate in struct for faster computation of H in DA
        h_comp h_comps;
        tf::Matrix3x3 m;
        m.setRotation(tf_sensor_base_.getRotation());
        tf::matrixTFToEigen(m, h_comps.R_fls_base_);
        {
            using namespace std;
            h_comps.mu_0 = mu_hat_(0);
            h_comps.mu_1 = mu_hat_(1);
            h_comps.mu_2 = mu_hat_(2);
            h_comps.c_3 = cos(mu_hat_(3));
            h_comps.c_4 = cos(mu_hat_(4));
            h_comps.c_5 = cos(mu_hat_(5));
            h_comps.s_3 = sin(mu_hat_(3));
            h_comps.s_4 = sin(mu_hat_(4));
            h_comps.s_5 = sin(mu_hat_(5));
        }

        // Store block of sigma common to all landmarks analysis
        temp_sigma.block(0,0,6,6) = Sigma_hat_.block(0,0,6,6);
        // For each possible landmark j in M
        Eigen::Vector3d landmark_j;
        for(unsigned int j=0; j<(mu_hat_.rows()-6)/3; j++){
            landmark_j = mu_hat_.segment(3 * j + 6, 3);
            temp_sigma.block(6,0,3,6) = Sigma_hat_.block(j * 3 + 6, 0, 3, 6);
            temp_sigma.block(0,6,6,3) = Sigma_hat_.block(0, j * 3 + 6, 6, 3);
            temp_sigma.block(6,6,3,3) = Sigma_hat_.block(j * 3 + 6, j * 3 + 6, 3, 3);
            predictMeasurement(landmark_j, z_t.at(i), i, j + 1, transf_base_map, temp_sigma, h_comps, corresp_i_list);
        }

        // Select the association with the minimum Mahalanobis distance
        if(!corresp_i_list.empty()){

            // Set init Mahalanobis distance for new possible landmark
            corresp_i_list.back().d_m_ = alpha;

            // Select correspondance with minimum Mh distance
            std::sort(corresp_i_list.begin(), corresp_i_list.end(), [](const CorrespondenceClass& corresp_1, const CorrespondenceClass& corresp_2){
                return corresp_1.d_m_ > corresp_2.d_m_;
            });

            // Update landmarks in the map
            if(lm_num_ >= corresp_i_list.back().i_j_.second){
                ROS_INFO("Known landmark detected");
                mu_hat_.conservativeResize(mu_hat_.rows()-3);
                Sigma_hat_.conservativeResize(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3);
//                if(map_lm_num_ >= corresp_i_list.back().i_j_.second){
                    // No new landmark added --> remove candidate from mu_hat_ and sigma_hat_
                    temp_sigma.block(6,0,3,6) = Sigma_hat_.block((corresp_i_list.back().i_j_.second - 1) * 3 + 6, 0, 3, 6);
                    temp_sigma.block(0,6,6,3) = Sigma_hat_.block(0, (corresp_i_list.back().i_j_.second - 1) * 3 + 6, 6, 3);
                    temp_sigma.block(6,6,3,3) = Sigma_hat_.block((corresp_i_list.back().i_j_.second - 1) * 3 + 6, (corresp_i_list.back().i_j_.second - 1) * 3 + 6, 3, 3);
//                }
            }
            else{
                // New landmark
                ROS_INFO("Added new landmark");
                lm_num_ = corresp_i_list.back().i_j_.second;
            }
            // Sequential update
            sequentialUpdate(corresp_i_list.back(), temp_sigma);
            corresp_i_list.clear();
        }
    }
    // Make sure mu and sigma have the same size at the end!
    while(mu_hat_.size() < Sigma_hat_.rows()){
        ROS_WARN("Sizes of mu and sigma differ!!");
        Sigma_hat_.conservativeResize(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3);
    }
}

void EKFCore::sequentialUpdate(CorrespondenceClass const& c_i_j, Eigen::MatrixXd temp_sigma){

    // Compute Kalman gain
    temp_sigma.block(6,0,3,6) = Sigma_hat_.block((c_i_j.i_j_.second - 1) * 3 + 6, 0, 3, 6);
    temp_sigma.block(0,6,6,3) = Sigma_hat_.block(0, (c_i_j.i_j_.second - 1) * 3 + 6, 6, 3);
    temp_sigma.block(6,6,3,3) = Sigma_hat_.block((c_i_j.i_j_.second - 1) * 3 + 6, (c_i_j.i_j_.second - 1) * 3 + 6, 3, 3);

    std::cout << "H_t : " << std::endl;
    std::cout << c_i_j.H_t_ << std::endl;
    std::cout << "S_inverted : " << std::endl;
    std::cout << c_i_j.S_inverted_ << std::endl;
    std::cout << "temp_sigma: " << std::endl;
    std::cout << temp_sigma << std::endl;

    Eigen::MatrixXd K_t_i = temp_sigma * c_i_j.H_t_.transpose() * c_i_j.S_inverted_;
    std::cout << "Kalman gain: " << std::endl;
    std::cout << K_t_i << std::endl;

    std::cout << "Innovation: " << std::endl;
    std::cout << c_i_j.nu_ << std::endl;

    // Update mu_hat and sigma_hat
    Eigen::VectorXd aux_vec = K_t_i * c_i_j.nu_;

    std::cout << "Correction for mu_hat: " << std::endl;
    std::cout << aux_vec << std::endl;

    mu_hat_.head(6) += aux_vec.head(6);
    mu_hat_(3) = angleLimit(mu_hat_(3));
    mu_hat_(4) = angleLimit(mu_hat_(4));
    mu_hat_(5) = angleLimit(mu_hat_(5));
    mu_hat_.segment((c_i_j.i_j_.second - 1) * 3 + 6, 3) += aux_vec.segment(6, 3);
    std::cout << "Mu updated " << std::endl;

    Eigen::MatrixXd aux_mat = (Eigen::MatrixXd::Identity(temp_sigma.rows(), temp_sigma.cols()) - K_t_i * c_i_j.H_t_) * temp_sigma;
    Sigma_hat_.block(0,0,6,6) = aux_mat.block(0,0,6,6);
    Sigma_hat_.block((c_i_j.i_j_.second - 1) * 3 + 6, (c_i_j.i_j_.second - 1) * 3 + 6, 3, 3) = aux_mat.block(6, 6, 3, 3);
    Sigma_hat_.block((c_i_j.i_j_.second - 1) * 3 + 6, 0, 3, 6) = aux_mat.block(6,0,3,6);
    Sigma_hat_.block(0, (c_i_j.i_j_.second - 1) * 3 + 6, 6, 3) = aux_mat.block(0,6,6,3);
    std::cout << "Update completed: " << std::endl;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> EKFCore::ekfUpdate(){

    // Update step
    if (mu_.rows()!= mu_hat_.rows()){
        int n_t = mu_hat_.rows() - mu_.rows();
        mu_.conservativeResize(mu_.size() + n_t, true);
        Sigma_.conservativeResize(Sigma_.rows() + n_t, Sigma_.cols() + n_t);
        // TODO: check that Sigma_ is still semi-definite positive
    }
    mu_ = mu_hat_;
    Sigma_ = Sigma_hat_;
    std::cout << "Mu: " << std::endl;
    std::cout << mu_<< std::endl;
    std::cout << "Sigma: " << std::endl;
    std::cout << Sigma_ << std::endl;
    std::cout << "Number of landmarks: " << lm_num_ << " or " << (Sigma_.rows() - 6) / 3 << std::endl;

    return std::make_pair(mu_, Sigma_);

}

EKFCore::~EKFCore(){
    // Empty queues

    // Delete instance pointers
}
