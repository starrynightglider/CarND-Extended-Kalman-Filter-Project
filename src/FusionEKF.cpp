#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

   //  Initialization
   if (!is_initialized_) {
    
    //Initialize the state  with the first measurement.
    float px = 0, py = 0, vx = 0, vy = 0;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      const double  &ro = measurement_pack.raw_measurements_(0);
      const double  &theta = measurement_pack.raw_measurements_(1);
      const double  &ro_dot = measurement_pack.raw_measurements_(2);
      px = ro * cos(theta);
      py = ro * sin(theta);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      px = measurement_pack.raw_measurements_(0);
      py = measurement_pack.raw_measurements_(1);
    }

    VectorXd x = VectorXd(4);
    MatrixXd P = MatrixXd(4, 4);
    x << px, py, vx, vy;
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1000, 0,
         0, 0, 0, 1000;
    
    ekf_.Init(x, P);
    previous_timestamp_ = measurement_pack.timestamp_; 
    is_initialized_ = true;
    return;
  }

  // Prediction
  long long  current_time = measurement_pack.timestamp_;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //dt - expressed in seconds
  previous_timestamp_ = current_time;
  ekf_.Predict(dt);

  //Update
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
