#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  // root mean squared error.
  VectorXd rmse = VectorXd(4);
  rmse << 0,0,0,0;
  
  // make sure that both vector size is same.
  if((estimations.size() != ground_truth.size()) && (estimations.size() == 0)) {
    std::cout << "Invalid estimations or ground_truth data" << std::endl;
   	return rmse;
  }
  
  for(int i = 0; i < estimations.size(); i++) {
  	// calculate error and sum up.
    VectorXd error = estimations[i] - ground_truth[i];
    VectorXd error_power_2 = error.array() * error.array();
  	rmse += error_power_2;
  }
  
  // divide by the size
  rmse = rmse/estimations.size();
  
  // root
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  MatrixXd H_jacobian = MatrixXd(3, 4);
  
  // recover state parameters 
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // px^2 + py^2
  float p_power_2 = px*px + py*py;
  // sqrt(px^2 + py^2)
  float p_root = sqrt(p_power_2);
  // p_power_2^(3/2)
  float p_root_mul_power = p_power_2 * p_root;
  
  // check division by zero
  if(p_power_2 == 0) {
    std::cout << "Error - Division by Zero" << std::endl;
  	return H_jacobian;
  }
  
  H_jacobian << px/p_root, py/p_root, 0, 0,
  			   -py/p_power_2, px/p_power_2, 0, 0,
  			   py*(vx*py-vy*px)/p_root_mul_power, px*(vy*px-vx*py)/p_root_mul_power, px/p_root, py/p_root;
  
  return H_jacobian;
  
}
