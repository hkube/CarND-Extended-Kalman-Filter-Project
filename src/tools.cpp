#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse.setZero();

  if ((0 < estimations.size()) && (estimations.size() == ground_truth.size()))
  {
    //accumulate squared residuals
    for(int i=0; i < estimations.size(); ++i){
      VectorXd diff(4);
      diff = estimations.at(i) - ground_truth.at(i);
      diff = diff.array() * diff.array();
      rmse += diff;
    }

    //calculate the mean
    rmse = rmse / static_cast<double>(estimations.size());

    //calculate the squared root
    rmse = rmse.array().sqrt();
  }
  else
  {
    std::cout << "DATA ERROR: estimations.size(): " << estimations.size() << "  ground_truth.size(): " << ground_truth.size() << std::endl;
  }

  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */

  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);

  MatrixXd Hj(3,4);
  Hj.setZero();

  const double p_dist_2 = px*px + py*py;
  const double p_dist   = sqrt(p_dist_2);

  // Check if division by zero would happen with the current values
  if (p_dist > 1e-3)
  {
    // Compute the Jacobian matrix
    Hj(0, 0) =  px / p_dist;
    Hj(0, 1) =  py / p_dist;
    Hj(1, 0) = -py / p_dist_2;
    Hj(1, 1) =  px / p_dist_2;
    Hj(2, 0) =  py * (vx*py - vy*px) / sqrt(p_dist_2 * p_dist_2 * p_dist_2);
    Hj(2, 1) =  px * (vy*px - vx*py) / sqrt(p_dist_2 * p_dist_2 * p_dist_2);
    Hj(2, 2) =  px / p_dist;
    Hj(2, 3) =  py / p_dist;
  }
  else
  {
    std::cout << "Calculate Jacobian () - Error - Division by Zero" << &std::endl;
  }
  return Hj;
}
