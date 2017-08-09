#ifndef MPC_H
#define MPC_H

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>



#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const int N = 12; // how many states we "lookahead" in the future
const double dt = 0.05; // how much time we expect environment changes

// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//typedef CPPAD_TESTVECTOR(double) ADvector;

struct Solution {
		vector<double> X;
		vector<double> Y;
		vector<double> Psi;
		vector<double> V;
		vector<double> Cte;
		vector<double> EPsi;
		vector<double> Delta;
		vector<double> A;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();
  
  Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  double delta_prev {0};
  double a_prev {0.1};
  
};

#endif /* MPC_H */
