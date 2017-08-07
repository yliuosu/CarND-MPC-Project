#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const int N = 10; // how many states we "lookahead" in the future
const double dt = 0.1; // how much time we expect environment changes

typedef CPPAD_TESTVECTOR(double) ADvector;

class MPC {
 public:
  MPC();

  virtual ~MPC();
  
  double steer;
  double throttle;
  
  ADvector x; // where all the state and actuation variables will be stored
  ADvector x_lowerbound; //lower limit for each corresponding variable in x
  ADvector x_upperbound; //upper limit for each corresponding variable in x
  ADvector g_lowerbound; // value constraint for each corresponding constraint expression
  ADvector g_upperbound; // value constraint for each corresponding constraint expression

  std::vector<double> future_xs;
  std::vector<double> future_ys;


  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
