#include "kalman_filter.h"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

#if 0
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
#endif

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose(); // TODO Should be calculated only once!
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose(); // TODO Should be calculated only once!
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
//  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  long x_rows = x_.size();  // TODO Use rows() here!
  MatrixXd I = MatrixXd::Identity(x_rows, x_rows); // TODO Should be calculated only once!
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  const double px_min = 0.001;
  const double rho_min = 0.001;
  const double px = x_(0);
  const double py = x_(1);
  const double vx = x_(2);
  const double vy = x_(3);
  const double rho    = std::sqrt(px*px + py*py);

  double phi    = std::atan2(py, px);
  const double rhodot = (px*vx + py*vy) / ((rho > rho_min) ? rho : rho_min) ;
  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;


  MatrixXd Hj = Tools::CalculateJacobian(x_);
  VectorXd y = z - z_pred;

  // Normalize the angle to prevent problem when y becomes negative
  // Refer to https://discussions.udacity.com/t/ekf-gets-off-track/276122/9
  y(1) = std::atan2( sin(y(1)), cos(y(1)));


  MatrixXd PHjt = P_ * Hj.transpose();
  MatrixXd S = Hj * PHjt + R_;
//  MatrixXd Si = S.inverse();
  MatrixXd K = PHjt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);

  long x_rows = x_.rows();  // TODO Use rows() here!
  MatrixXd I = MatrixXd::Identity(x_rows, x_rows); // TODO Should be calculated only once!
  P_ = (I - K * Hj) * P_;
}
