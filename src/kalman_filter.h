#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  KalmanFilter(const Eigen::MatrixXd &lidar_H,
               const Eigen::MatrixXd &lidar_R, const Eigen::MatrixXd & radar_R):
               lidar_H_(lidar_H), lidar_R_(lidar_R), radar_R_(radar_R) {}
  virtual ~KalmanFilter(){}
  
  void Init (const Eigen::VectorXd &x, const Eigen::MatrixXd &P){ 
       x_ = x; 
       P_ = P;
  }
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
  const Eigen::MatrixXd lidar_H_;
  // measurement covariance matrix
  const Eigen::MatrixXd radar_R_;
  const Eigen::MatrixXd lidar_R_;
};

#endif /* KALMAN_FILTER_H_ */
