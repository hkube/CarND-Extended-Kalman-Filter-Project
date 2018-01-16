#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // measurement covariance matrix
//  Eigen::MatrixXd R_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  void Init(const Eigen::MatrixXd & P_in,
            const Eigen::MatrixXd & F_in,
            const Eigen::MatrixXd & H_in);

  /**
   * InitX Initializes the state vector of the Kalman filter
   * @param x_in Initial state
   */
  void InitX(const Eigen::VectorXd & x_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const double deltaT,
               const double noise_ax,
               const double noise_ay);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   * @param R The measurement covariance matrix
   */
  void Update(const Eigen::VectorXd &z, const Eigen::MatrixXd & R);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   * @param R The measurement covariance matrix
   */
  void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd & R);

  /**
   * Return the state vector
   */
  const Eigen::VectorXd & getX() const;

  /**
   * Return the state covariance matrix
   */
  const Eigen::MatrixXd & getP() const;

private:

  /**
   * The common parts of Update() and UPdateEKF()
   * @param z The measurement at k+1
   * @param R The measurement covariance matrix
   * @param H The measurement matrix
   * @param Ht The transposed measurement matrix
   */
  void DoUpdate(const Eigen::VectorXd & y,
                const Eigen::MatrixXd & R,
                const Eigen::MatrixXd & H,
                const Eigen::MatrixXd & Ht);

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // transposed measurement matrix
  Eigen::MatrixXd Ht_;

  // identity matrix
  Eigen::MatrixXd I_;

};

#endif /* KALMAN_FILTER_H_ */
