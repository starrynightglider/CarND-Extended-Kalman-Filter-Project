#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  FusionEKF();
  virtual ~FusionEKF();

  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  Eigen::VectorXd GetX() { return ekf_.GetX();}

private:

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  long long previous_timestamp_;
  KalmanFilter ekf_;
  Eigen::MatrixXd lidar_H_;
  Eigen::MatrixXd lidar_R_;
  Eigen::MatrixXd radar_R_;
};

#endif /* FusionEKF_H_ */
