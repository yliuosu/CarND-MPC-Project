#include "MPC.h"

using CppAD::AD;


// This value assumes the model presented in the classroom is used.

double ref_cte = 0;
double ref_epsi = 0;
double ref_v = 85;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// weights for cost computations
double w_cte = 1.0;
double w_epsi = 1.0;
double w_a = 25.0;   //20
double w_delta = 1.0;
double w_ddelta = 1200.0; // weight cost for high difference between consecutive steering actuations 1000
double w_da = 1000.0;  //900


class FG_eval {
 public:
 
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
 
  FG_eval(Eigen::VectorXd coeffs) { 
    this->coeffs = coeffs; 
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
	
	// The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0.0;
	
	// The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
		
      fg[0] += w_cte*  CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += w_epsi*  CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += /*w_delta**/ CppAD::pow(vars[delta_start + t], 2);
      fg[0] +=  w_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += w_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += w_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);  
    }
	
	// add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
	// This bumps up the position of all the other values.
	fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
	fg[1 + epsi_start] = vars[epsi_start];
	
	// constraints based on our kinematic model
      for (int i = 0; i < N - 1; ++i) {

        // state at time t
        auto px0 = vars[x_start + i];
        auto py0 = vars[y_start + i];
        auto psi0 = vars[psi_start + i];
        auto v0 = vars[v_start + i];
        auto cte0 = vars[cte_start + i];
        auto epsi0 = vars[epsi_start + i];
		
        auto delta0 = vars[delta_start + i];
        auto a0 = vars[a_start + i];

        // state at time t+1 
		auto px1 = vars[x_start + i + 1];
        auto py1 = vars[y_start + i + 1];
        auto psi1 = vars[psi_start + i + 1];
        auto v1 = vars[v_start + i + 1];
        auto cte1 = vars[cte_start + i + 1];
        auto epsi1 = vars[epsi_start + i + 1];

        // desired py and psi
        const auto py_desired = coeffs[3] * px0 * px0 * px0 + coeffs[2] * px0 * px0 + coeffs[1] * px0 + coeffs[0];
        const auto psi_desired = CppAD::atan(3.0 * coeffs[3] * px0 * px0 + 2.0 * coeffs[2] * px0 + coeffs[1]);

		// Use the equations for the model:
        // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
        // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
        // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
        // v_[t+1] = v[t] + a[t] * dt
		// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
		// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
        // store the constraint expression of two consecutive states
        fg[x_start + i  + 2] = px1 -( px0 + v0 * CppAD::cos(psi0) * dt);
        fg[y_start + i + 2] = py1 - (py0 + v0 * CppAD::sin(psi0) * dt);
        fg[psi_start + i + 2] = psi1 - (psi0 + v0 * (-delta0) / Lf * dt);
        fg[v_start + i + 2] = v1 - (v0 + a0 * dt);
        fg[cte_start + i  + 2] = cte1 - (py_desired - py0 + v0 * CppAD::sin(epsi0) * dt);
        fg[epsi_start + i + 2] = epsi1 - (psi0 - psi_desired + v0 * (-delta0) / Lf * dt);
	}
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
	
	
}

MPC::~MPC() {}


Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  
}
