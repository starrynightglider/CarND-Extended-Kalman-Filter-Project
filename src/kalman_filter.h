#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  KalmanFilter();
  virtual ~KalmanFilter();
  
  void Init (const Eigen::VectorXd &x, const Eigen::MatrixXd &P, 
             const Eigen::MatrixXd &lidar_H,
             const Eigen::MatrixXd &lidar_R, const Eigen::MatrixXd & radar_R);
  void Predict (float dt);
  void Update (const Eigen::VectorXd &z); // for LIDAR
  void UpdateEKF (const Eigen::VectorXd &z); // for RADAR
  Eigen::VectorXd GetX() {return x_;}
  private: 
  // state vector
  Eigen::VectorXd x_;
  // state covariance matrix
  Eigen::MatrixXd P_;
  // measurement matrix
  Eigen::MatrixXd lidar_H_;
  // measurement covariance matrix
  Eigen::MatrixXd radar_R_;
  Eigen::MatrixXd lidar_R_;
};

#endif /* KALMAN_FILTER_H_ */
