#include "kalman_filter.h"
#include "tools.h"
#include "math.h"
#include <algorithm>
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(const VectorXd &x, const MatrixXd &P, 
                        const MatrixXd &lidar_H,
                        const MatrixXd &lidar_R, const MatrixXd &radar_R) {
  x_ = x;
  P_ = P;
  lidar_H_ = lidar_H;
  lidar_R_ = lidar_R;
  radar_R_ = radar_R;
}

void KalmanFilter::Predict(float dt) {
 
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9, noise_ay = 9;
  
  // state transition matrix
  MatrixXd F = MatrixXd(4,4);
  F << 1, 0, dt, 0,
       0, 1, 0, dt,
       0, 0, 1, 0,
       0, 0, 0, 1;
  
  // process covairance
  MatrixXd Q = MatrixXd(4, 4);
  Q <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
        0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
        dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
        0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  x_ = F * x_;
  MatrixXd Ft = F.transpose();
  P_ = F * P_ * Ft + Q;
}

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd z_pred = lidar_H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = lidar_H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = lidar_H_ * PHt + lidar_R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * lidar_H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  // Compute Jacobian
  MatrixXd Hj = Tools::CalculateJacobian(x_);

  VectorXd x(3);
  // Convert to polar 
  float rho = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  float phi = atan2(x_(1),x_(0));
  rho = std::max(rho, (float) 0.000001);  
  float rho_dot = (x_(0)*x_(2)+x_(1)*x_(3))/rho;
  x << rho, phi, rho_dot;  
  
  VectorXd y = z - x;
 
  // normalize y(1)
  while (y(1) > M_PI) y(1) -= (M_PI*2);
  while (y(1) < -M_PI) y(1) += (M_PI*2);

  MatrixXd Ht = Hj.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = Hj * PHt + radar_R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}
