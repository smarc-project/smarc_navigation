#include "ekf_slam_core/ekf_slam_core.hpp"


EKFCore::EKFCore(Eigen::VectorXd &mu, Eigen::MatrixXd &Sigma, Eigen::MatrixXd &R, Eigen::MatrixXd &Q_fls, Eigen::MatrixXd &Q_mbes,
                 double &lambda_fls, double &lambda_mbes, tf::StampedTransform &tf_base_sensor, const double mh_dist_fls, const double mh_dist_mbes){

    // Initialize internal params
    mu_ = mu;
    mu_hat_ = mu_;
    mu_auv_odom_.setZero(3);
    Sigma_ = Sigma;
    Sigma_hat_ = Sigma_;
    R_ = R;
    Q_fls_ = Q_fls;
    Q_mbes_ = Q_mbes;
    lambda_fls_ = lambda_fls;
    lambda_mbes_ = lambda_mbes;
    lm_num_ = (mu_.rows() - 6) / 3;
    tf_base_sensor_ = tf_base_sensor;
    tf_sensor_base_ = tf_base_sensor.inverse();
    mh_dist_fls_ = mh_dist_fls;
    mh_dist_mbes_ = mh_dist_mbes;

//    const Eigen::MatrixXd CorrespondenceMBES::Q_ = Q;
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
    mu_hat_(3) = utils::angleLimit(mu_hat_(3));
    mu_hat_(4) = utils::angleLimit(mu_hat_(4));
    mu_hat_(5) = utils::angleLimit(mu_hat_(5));

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


void EKFCore::predictBatchMeasurement(const Eigen::Vector3d &landmark_j,
                                 const Eigen::Vector3d &z_i,
                                 unsigned int i,
                                 unsigned int j,
                                 const tf::Transform &transf_base_map,
                                 const Eigen::MatrixXd &temp_sigma,
                                 h_comp h_comps,
                                 const utils::MeasSensor &sens_type,
                                 std::vector<CorrespondenceClass> &corresp_i_list,
                                 Eigen::MatrixXd &corresp_table){


    CorrespondenceClass* corresp_i_j;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> z_hat_tuple;
    tf::Vector3 landmark_j_map(landmark_j(0),
                               landmark_j(1),
                               landmark_j(2));

    // Measurement model: z_expected and z_expected_sensor (in sensor frame)
    Eigen::MatrixXd Q_t;
    tf::Transform tf_sensor_map;
    double lambda_meas;
    switch(sens_type){
        case utils::MeasSensor::MBES:    // MBES
            corresp_i_j = new CorrespondenceMBES(i, j);
            tf_sensor_map = transf_base_map;
            Q_t = Q_mbes_;
            lambda_meas = lambda_mbes_;
            break;
        case utils::MeasSensor::FLS:    // FLS
            corresp_i_j = new CorrespondenceFLS(i, j);
            tf_sensor_map = tf_sensor_base_ * transf_base_map;
            Q_t = Q_fls_;
            lambda_meas = lambda_fls_;
            break;
    }
    z_hat_tuple = corresp_i_j->measModel(landmark_j_map, tf_sensor_map);

    // Compute MHL distance of correspondence cij: z_i with m_j
    corresp_i_j->computeH(h_comps, landmark_j_map, std::get<1>(z_hat_tuple));
    corresp_i_j->computeNu(std::get<0>(z_hat_tuple), z_i);
    corresp_i_j->computeMHLDistance(temp_sigma, Q_t);

    // Store correspondence object
    corresp_i_list.push_back(std::move(*corresp_i_j));

    // Add MHD distance to assignment table
    // Outlier rejection
    if(corresp_i_j->d_m_ < lambda_meas){
        corresp_table(j, i) = corresp_i_j->d_m_;
    }
    else{
        corresp_table(j, i) = 10000;    // Infinite value
        ROS_INFO("Outlier rejected");
    }

    delete (corresp_i_j);
}

void EKFCore::batchDataAssociation(std::vector<Eigen::Vector3d> z_t, const utils::MeasSensor &sens_type){

    std::vector<CorrespondenceClass> corresp_list;
    tf::Transform transf_base_map;
    tf::Transform transf_map_base;
    Eigen::MatrixXd temp_sigma(9,9);

    // For each observation z_i at time t
    lm_num_ = (mu_.rows() - 6) / 3;
    h_comp h_comps;
    tf::Matrix3x3 m;

    // Compute transform map --> base from current state state estimate at time t
    transf_map_base = tf::Transform(tf::createQuaternionFromRPY(mu_hat_(3), mu_hat_(4), mu_hat_(5)).normalize(),
                                     tf::Vector3(mu_hat_(0), mu_hat_(1), mu_hat_(2)));
    transf_base_map = transf_map_base.inverse();

    // Store current mu_hat_ estimate in struct for faster computation of H in DA
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

    // Select meas model depending on the sensor input type
    CorrespondenceClass* sensor_input;
    tf::Transform tf_map_sensor;
    std::tuple<double, double, double> new_lm_cov;
    double new_mh_dist;
    switch(sens_type){
        case utils::MeasSensor::MBES:
            sensor_input = new CorrespondenceMBES();
            tf_map_sensor = transf_map_base;
            // Covariance of new lm
            new_lm_cov = std::make_tuple(100,100,100);  // TODO: make dependent on the sensor
            // MH dist of new lm
            new_mh_dist = mh_dist_mbes_;
            break;

        case utils::MeasSensor::FLS:
            sensor_input = new CorrespondenceFLS();
            tf_map_sensor = transf_map_base  * tf_base_sensor_;
            // Covariance of new lm
            new_lm_cov = std::make_tuple(400,200,1000);
            // MH dist of new lm
            new_mh_dist = mh_dist_fls_;
            break;
    }

    // Store latest mu_hat_ and sigma_hat_ for next steps
    Eigen::VectorXd mu_hat_temp = mu_hat_;
    Eigen::MatrixXd Sigma_hat_temp = Sigma_hat_;

    // Back-project new possible landmark (in map frame) for every z_i
    Eigen::Vector3d new_lm_map;
    for(unsigned int i = 0; i<z_t.size(); i++){
        new_lm_map = sensor_input->backProjectNewLM(z_t.at(i), tf_map_sensor);
        // Add new possible lm to filter
        utils::addLMtoFilter(mu_hat_temp, Sigma_hat_temp, new_lm_map, new_lm_cov);
    }

    // Construct correspondences table for global assignment
    unsigned int temp_num_lm = (mu_hat_temp.rows()-6)/3;
    Eigen::MatrixXd corresp_table(temp_num_lm, z_t.size());

    // For every meas z_i in z_t
    for(unsigned int i = 0; i<z_t.size(); i++){
        // For each possible landmark j in Map
        Eigen::Vector3d landmark_j;
        for(unsigned int j=0; j<temp_num_lm; j++){
            landmark_j = mu_hat_temp.segment(3 * j + 6, 3);
            utils::updateMatrixBlock(Sigma_hat_temp, temp_sigma, j);
            predictBatchMeasurement(landmark_j, z_t.at(i), i, j, transf_base_map, temp_sigma, h_comps, sens_type, corresp_list, corresp_table);
        }
    }

    // Add new_mh_dist to backprojected lm candidates in correspondence table
    int cnt = 0;
    for(int j=lm_num_; j<temp_num_lm; j++){
        for(int i=0; i<z_t.size(); i++){
            if(i == cnt){ // Diagonal elements of submatrix with new backprojected landmarks
                corresp_table(j, i) = new_mh_dist;
            }
            else{
                corresp_table(j, i) = 10000;   // Infinite value
            }
        }
        cnt += 1;
    }

    // Initialize Munkres matrix from Eigen matrix
    Matrix<double> munkres_matrix(temp_num_lm, z_t.size());
    for ( int row = 0 ; row < temp_num_lm ; row++ ) {
        for ( int col = 0 ; col < z_t.size() ; col++ ) {
            munkres_matrix(row,col) = corresp_table(row,col);
        }
    }

    // Solve correspondence problem
    Munkres<double> munkres_solver;
    munkres_solver.solve(munkres_matrix);

    // Update step with selected correspondences
    double lm;
    for(unsigned int i=0; i<z_t.size(); i++){
        for (unsigned int j=0; j<temp_num_lm; j++){
            if(munkres_matrix(j,i) == 0){
                if(j >= lm_num_){ // If new landmark added
                    ROS_INFO_STREAM("New lm added");
                    // Resize mu and sigma with final landmarks in map (if any new one added)
                    utils::addLMtoFilter(mu_hat_, Sigma_hat_, corresp_list.at(j + temp_num_lm * i).landmark_pos_, new_lm_cov);
                    // Recompute new lm index based on its order in the filter
                    lm = ((Sigma_hat_.rows() - 6) / 3) - 1;
                    corresp_list.at(j + temp_num_lm * i).i_j_.second = lm;
                }
                else{
                    ROS_INFO_STREAM("Known lm seen");
                    // Known lm index based on its order in the filter
                    lm  = j;
                }
                utils::updateMatrixBlock(Sigma_hat_, temp_sigma, lm);
                sequentialUpdate(corresp_list.at(j + temp_num_lm * i), temp_sigma);
                continue;
            }
        }
    }
    delete (sensor_input);

    // Make sure mu and sigma have the same size at the end!
    while(mu_hat_.size() < Sigma_hat_.rows()){
        ROS_WARN("Sizes of mu and sigma differ!!");
        Sigma_hat_.conservativeResize(Sigma_hat_.rows()-3, Sigma_hat_.cols()-3);
    }
}


void EKFCore::sequentialUpdate(CorrespondenceClass const& c_i_j, Eigen::MatrixXd temp_sigma){

    // Compute Kalman gain
    utils::updateMatrixBlock(Sigma_hat_, temp_sigma, c_i_j.i_j_.second);
    Eigen::MatrixXd K_t_i = temp_sigma * c_i_j.H_t_.transpose() * c_i_j.S_inverted_;

    // Update mu_hat and sigma_hat
    Eigen::VectorXd aux_vec = K_t_i * c_i_j.nu_;

    mu_hat_.head(6) += aux_vec.head(6);
    mu_hat_(3) = utils::angleLimit(mu_hat_(3));
    mu_hat_(4) = utils::angleLimit(mu_hat_(4));
    mu_hat_(5) = utils::angleLimit(mu_hat_(5));
    mu_hat_.segment((c_i_j.i_j_.second) * 3 + 6, 3) += aux_vec.segment(6, 3);

    Eigen::MatrixXd aux_mat = (Eigen::MatrixXd::Identity(temp_sigma.rows(), temp_sigma.cols()) - K_t_i * c_i_j.H_t_) * temp_sigma;
    Sigma_hat_.block(0,0,6,6) = aux_mat.block(0,0,6,6);
    Sigma_hat_.block((c_i_j.i_j_.second) * 3 + 6, (c_i_j.i_j_.second) * 3 + 6, 3, 3) = aux_mat.block(6, 6, 3, 3);
    Sigma_hat_.block((c_i_j.i_j_.second) * 3 + 6, 0, 3, 6) = aux_mat.block(6,0,3,6);
    Sigma_hat_.block(0, (c_i_j.i_j_.second) * 3 + 6, 6, 3) = aux_mat.block(0,6,6,3);
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
    lm_num_ = (mu_.rows() - 6) / 3;

    return std::make_pair(mu_, Sigma_);
}

EKFCore::~EKFCore(){
    // Empty queues

    // Delete instance pointers
}
