#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "landmark_ml.hpp"

// Sorts the ML data association list to find the max MLj
bool sortLandmarksML(LandmarkML *ml_1, LandmarkML *ml_2);

class Localization{
public:
    Localization(std::string node_name, ros::NodeHandle &nh);

private:
    // ROS variables
    ros::NodeHandle *nh_;
    std::string node_name_;
    ros::Subscriber sens_sub_;
    ros::Publisher nu_est_pub_;
    ros::Publisher nu_real_pub_;
    ros::Publisher error_pub_;
    ros::ServiceClient map_client_;

    // Sensors readings handlers
    std::vector<boost::numeric::ublas::vector<int>> map_;
    std::queue<ekf_general::sensors_read::Ptr> msgs_queue_;
    ekf_general::sensors_read::Ptr sensor_in_;
    // State estimate variables
    boost::numeric::ublas::vector<double> mu_;
    boost::numeric::ublas::vector<double> mu_hat_;
    boost::numeric::ublas::matrix<double> sigma_;
    boost::numeric::ublas::matrix<double> sigma_hat_;
    // Noise covariances
    boost::numeric::ublas::matrix<double> R_;
    boost::numeric::ublas::matrix<double> Q_;
    // Measurement variables
    boost::numeric::ublas::vector<double> h_;
    boost::numeric::ublas::vector<double> z_;
    // Outliers rejection variables
    double lambda_M_;
    double delta_m_;
    // Control variables
    boost::numeric::ublas::vector<int> encoders_prev_;
    boost::numeric::ublas::vector<double> u_t_;
    double t_prev_;

    // Aux methods
    void getMapFile(std::string path_to_map);
    void split (const std::string &s, char delim, std::vector<std::string> &elems);

    void sensorsCB(const ekf_general::sensors_read::Ptr &sensor_msg);

    // Lab1 methods
    void init(const double &initial_R, const double &initial_Q, const double &delta);
    void computeOdom();
    void predictionStep();
    void predictMeasurementModel(const boost::numeric::ublas::vector<int> &landmark_j,
                                 boost::numeric::ublas::vector<double> &z_i,
                                 std::vector<LandmarkML *> &ml_i_list);
    void dataAssociation(std::vector<LandmarkML *> &ml_t_list);
    void sequentialUpdate(std::vector<LandmarkML *> &observ_list);
    void ekfLocalize();
};

#endif // LOCALIZATION_HPP

