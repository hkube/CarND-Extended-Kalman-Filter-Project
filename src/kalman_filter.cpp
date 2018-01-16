#include "kalman_filter.h"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter()
: x_(4)
, P_(4, 4)
, F_(4, 4)
, Q_(4, 4)
, H_(2, 4)
, Ht_(4, 2) {
  // Set all matrices to zero
  x_.setZero();
  P_.setZero();
  F_.setZero();
  Q_.setZero();
  H_.setZero();
  Ht_.setZero();
}


KalmanFilter::~KalmanFilter() {}


void KalmanFilter::Init(const MatrixXd & P_in,
                        const MatrixXd & F_in,
                        const MatrixXd & H_in) {
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  Ht_ = H_.transpose();
}


void KalmanFilter::InitX(const VectorXd & x_in) {
  x_ = x_in;
  // Initialize the identity matrix
  const long x_rows = x_.rows();
  I_ = MatrixXd::Identity(x_rows, x_rows);
}


void KalmanFilter::Predict(const double deltaT,
                           const double noise_ax,
                           const double noise_ay) {
  // Update the state transition matrix
  F_(0, 2) = deltaT;
  F_(1, 3) = deltaT;

  // Update the process noise covariant matrix
  const double deltaT_2 = deltaT   * deltaT;
  const double deltaT_3 = deltaT_2 * deltaT;
  const double deltaT_4 = deltaT_3 * deltaT;

  Q_.setZero();
  Q_(0, 0) = deltaT_4 / 4 * noise_ax;
  Q_(1, 1) = deltaT_4 / 4 * noise_ay;
  Q_(0, 2) = deltaT_3 / 2 * noise_ax;
  Q_(1, 3) = deltaT_3 / 2 * noise_ay;
  Q_(2, 0) = deltaT_3 / 2 * noise_ax;
  Q_(3, 1) = deltaT_3 / 2 * noise_ay;
  Q_(2, 2) = deltaT_2 * noise_ax;
  Q_(3, 3) = deltaT_2 * noise_ay;

  /**
   * predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}


void KalmanFilter::Update(const VectorXd &z, const MatrixXd & R) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  DoUpdate(y, R, H_, Ht_);
}


void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd & R) {
  /**
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

  DoUpdate(y, R, Hj, Hj.transpose());
}

void KalmanFilter::DoUpdate(const VectorXd & y,
                            const MatrixXd & R,
                            const MatrixXd & H,
                            const MatrixXd & Ht) {

  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H * PHt + R;
  MatrixXd K = PHt * S.inverse();

  //new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H) * P_;
}


const Eigen::VectorXd & KalmanFilter::getX() const {
  return x_;
}


const Eigen::MatrixXd & KalmanFilter::getP() const {
  return P_;
}
