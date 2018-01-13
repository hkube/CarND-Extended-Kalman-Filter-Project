#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // 4D state vector x
  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;

  // State covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0,    0,    0,
             0, 1,    0,    0,
             0, 0, 1000,    0,
             0, 0,    0, 1000;

  // Initial transition matrix F
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Measurement matrix H
  ekf_.H_ = MatrixXd(2, 4);
  ekf_.H_ << 1, 0, 0, 0,
             0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF - Initialization: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << "First Measurement RADAR  -ignored " << &std::endl;
      return;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "First Measurement LASER" << &std::endl;
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                 measurement_pack.raw_measurements_[1],
                 0, 0;
      std::cout << "ekf_.x_: [" << ekf_.x_ << "]" << std::endl;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
//    return;
  }
  else {

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    // Calculate the time difference and update the state transition matrix
    float deltaT = float(measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
    ekf_.F_(0, 2) = deltaT;
    ekf_.F_(1, 3) = deltaT;


    // Update the process noise covariant matrix
    const float deltaT_2 = deltaT   * deltaT;
    const float deltaT_3 = deltaT_2 * deltaT;
    const float deltaT_4 = deltaT_3 * deltaT;
    const float noise_ax = 9;
    const float noise_ay = 9;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_.setZero();
    ekf_.Q_(0, 0) = deltaT_4 / 4 * noise_ax;
    ekf_.Q_(1, 1) = deltaT_4 / 4 * noise_ay;
    ekf_.Q_(0, 2) = deltaT_3 / 2 * noise_ax;
    ekf_.Q_(1, 3) = deltaT_3 / 2 * noise_ay;
    ekf_.Q_(2, 0) = deltaT_3 / 2 * noise_ax;
    ekf_.Q_(3, 1) = deltaT_3 / 2 * noise_ay;
    ekf_.Q_(2, 2) = deltaT_2 * noise_ax;
    ekf_.Q_(3, 3) = deltaT_2 * noise_ay;

    std::cout << "-- Before prediction" << std::endl;
//    std::cout << " ekf_.F_: [" << ekf_.F_ << "]" << std::endl;
//    std::cout << " ekf_.P_: [" << ekf_.P_ << "]" << std::endl;
//    std::cout << " ekf_.Q_: [" << ekf_.Q_ << "]" << std::endl;
    std::cout << " ekf_.x_: [" << ekf_.x_.transpose() << "]" << std::endl;

    // Predict the new state
    ekf_.Predict();

    std::cout << "-- After predition" << std::endl;
//    std::cout << " ekf_.P_: [" << ekf_.P_ << "]" << std::endl;
    std::cout << " ekf_.x_: [" << ekf_.x_.transpose() << "]" << std::endl;

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
      std::cout << "Measurement RADAR  -ignored " << &std::endl;
      ekf_.R_ = R_radar_;
    } else {
      // Laser updates
      std::cout << "Measurement LASER" << &std::endl;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
//    cout << "x_ = " << ekf_.x_ << endl;
//    cout << "P_ = " << ekf_.P_ << endl;
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}
