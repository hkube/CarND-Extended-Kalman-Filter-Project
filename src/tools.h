#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Tools {
public:
  /**
  * A helper method to calculate RMSE.
  */
  static VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations,
                                const std::vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  static MatrixXd CalculateJacobian(const VectorXd& x_state);

};

#endif /* TOOLS_H_ */
