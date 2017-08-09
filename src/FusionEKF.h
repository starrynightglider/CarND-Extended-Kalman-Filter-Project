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
  FusionEKF(const MatrixXd & lidar_H, const MatrixXd & lidar_R, const MatrixXd & radar_R):
  ekf_(lidar_H, lidar_R, radar_R),
  is_initialized_(false),
  previous_timestamp_(0){}

  virtual ~FusionEKF(){};

  void ProcessMeasurement(const MeasurementPackage &measurement_pack);
  Eigen::VectorXd GetX() { return ekf_.GetX();}

private:

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  long long previous_timestamp_;
  KalmanFilter ekf_;
};

#endif /* FusionEKF_H_ */
