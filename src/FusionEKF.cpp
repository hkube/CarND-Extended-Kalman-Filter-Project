#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <cmath>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define IGNORE_RADAR_DATA 0
#define IGNORE_LASER_DATA 0

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
: is_initialized_(false)
, previous_timestamp_(0LL)
, R_laser_(2, 2)
, R_radar_(3, 3) {

  // initializing matrices

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  /**
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */
  // State covariance matrix P
  MatrixXd P(4, 4) ;
  P << 1, 0,    0,    0,
       0, 1,    0,    0,
       0, 0, 1000,    0,
       0, 0,    0, 1000;

  // Initial transition matrix F
  MatrixXd F(4, 4);
  F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // Measurement matrix H
  MatrixXd H(2, 4);
  H << 1, 0, 0, 0,
       0, 1, 0, 0;

  ekf_.Init(P, F, H);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

#if IGNORE_LASER_DATA
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    return;
  }
#elif IGNORE_RADAR_DATA
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    return;
  }
#endif
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "EKF - Initialization: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << "First Measurement RADAR" << &std::endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      const double rho    = measurement_pack.raw_measurements_[0];
      const double phi    = measurement_pack.raw_measurements_[1];
      const double rhodot = measurement_pack.raw_measurements_[2];
      const double sin_phi = std::sin(phi);
      const double cos_phi = std::cos(phi);
      const double x = rho * cos_phi;
      const double y = rho * sin_phi;
      const double vx = rhodot * cos_phi;
      const double vy = rhodot * sin_phi;

      VectorXd xInit(4);
      xInit << x, y, vx, vy;
      ekf_.InitX(xInit);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "First Measurement LASER" << &std::endl;
      /**
       * Initialize state.
       */
      VectorXd xInit(4);
      xInit << measurement_pack.raw_measurements_[0],
               measurement_pack.raw_measurements_[1],
               0, 0;
      ekf_.InitX(xInit);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
  }
  else {

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    // Calculate the time difference and update the state transition matrix
    double deltaT = double(measurement_pack.timestamp_ - previous_timestamp_) / 1e6;

     // Predict the new state
    ekf_.Predict(deltaT, 9, 9);


    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
      ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_);
    } else {
      // Laser updates
      ekf_.Update(measurement_pack.raw_measurements_, R_laser_);
    }

    // print the output
    cout << "x_ = " << ekf_.getX() << endl;
    cout << "P_ = " << ekf_.getP() << endl;
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}

const Eigen::VectorXd & FusionEKF::getX() const {
  return ekf_.getX();
}

