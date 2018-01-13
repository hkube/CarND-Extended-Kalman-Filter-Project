#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
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
    rmse = rmse / static_cast<float>(estimations.size());

    //calculate the squared root
    rmse = rmse.array().sqrt();
  }

  return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
   * Calculate a Jacobian here.
   */

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  MatrixXd Hj(3,4);
  Hj.setZero();

  //check division by zero
  if ((0.0 != px) && (0.0 != py))
  {
    const float p_dist_2 = px*px + py*py;
    const float p_dist = sqrt(p_dist_2);

    //compute the Jacobian matrix
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
