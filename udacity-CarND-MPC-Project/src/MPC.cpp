#include <iostream>
#include <string>
#include <vector>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>


#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"



using CppAD::AD;
using Eigen::VectorXd;

/**
 * TODO: Set the timestep length and duration
 */
size_t N = 10;
double dt = .1;



// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;


const double ref_cte      = 0;
const double ref_epsi     = 0;
const double ref_v        = 100;

const size_t x_start      = 0;
const size_t y_start      = x_start     +N;
const size_t psi_start    = y_start     +N;
const size_t v_start      = psi_start   +N;
const size_t cte_start    = v_start     +N;
const size_t epsi_start   = cte_start   +N;
const size_t delta_start  = epsi_start  +N;
const size_t a_start      = delta_start +N-1;



class FG_eval {
 public:
  // Fitted polynomial coefficients
  VectorXd coeffs_;
  VectorXd weights_;

  // Constructor
  FG_eval(VectorXd coeffs, VectorXd weights) { 
    this->coeffs_         = coeffs;
    this->weights_        = weights;
    assert(weights_.size() == 7);
    }
  
  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
    * `fg` is a vector of the cost constraints, 
    * `vars` is a vector of variable values (state & actuators)
    */

    // Variables and Constraints
    size_t n_vars         = 6 * N + 2 * (N - 1);
    size_t n_constraints  = 6* N+1;

    assert( fg.size()     == n_constraints );
    assert( vars.size()   == n_vars );

    // ------------------------------------------------------
    //                      fg[0] ~ f(x)
    // ------------------------------------------------------
    fg[0] = 0;

    // cte  -> 0
    // epsi -> 0
    // v    -> ref_v
    // weights = [1000, 1000, 1]
    for (int i =0; i<N; i++){
      fg[0] += weights_[0]*CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      fg[0] += weights_[1]*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      fg[0] += weights_[2]*CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // minimize control input u 
    // {a, delta} -> {0,0}
    // weights = [50, 50]
    for (int i = 0; i < N-1; i++){
      fg[0] += weights_[3]* CppAD::pow(vars[delta_start + i], 2);
      fg[0] += weights_[4]* CppAD::pow(vars[a_start + i], 2);
    }

    // minimize gap between sequential actuations
    // weights = [250000, 5000]
    for (int i = 0; i < N-2; i++){
      fg[0] += weights_[5] * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start +i], 2);
      fg[0] += weights_[6] * CppAD::pow(vars[a_start+i+1] - vars[a_start +i], 2);
    }


    // ------------------------------------------------------
    //                      fg[1:] ~ g(x)
    // ------------------------------------------------------

    // Initial constraints
    fg[1+ x_start]    = vars[x_start];
    fg[1+ y_start]    = vars[y_start];
    fg[1+ psi_start]  = vars[psi_start];
    fg[1+ v_start]    = vars[v_start];
    fg[1+ cte_start]  = vars[cte_start];
    fg[1+ epsi_start] = vars[epsi_start];

    for (int t=1; t<N; t++){

      // state[t+1]
      AD<double> x1     = vars[x_start + t];
      AD<double> y1     = vars[y_start + t];
      AD<double> psi1   = vars[psi_start + t];
      AD<double> v1     = vars[v_start + t];
      AD<double> cte1   = vars[cte_start + t];
      AD<double> epsi1  = vars[epsi_start + t];

      // state[t]
      AD<double> x0     = vars[x_start + t -1];
      AD<double> y0     = vars[y_start + t -1];
      AD<double> psi0   = vars[psi_start + t -1];
      AD<double> v0     = vars[v_start + t -1];
      AD<double> cte0   = vars[cte_start + t -1];
      AD<double> epsi0  = vars[epsi_start + t -1];

      // u[t]
      AD<double> delta0 = vars[delta_start + t -1];
      AD<double> a0     = vars[a_start + t - 1];

      AD<double> f0     = coeffs_[0] 
                        + coeffs_[1] * x0 
                        + coeffs_[2] * CppAD::pow(x0, 2) 
                        + coeffs_[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs_[1] 
                                 + 2 * coeffs_[2] * x0 
                                 + 3 * coeffs_[3] * CppAD::pow(x0, 2));


      // Constraints
      fg[1 + x_start + t]     = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t]     = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t]   = psi1 - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + v_start + t]     = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t]   = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t]  = epsi1 - ((psi0 - psides0) + v0 / Lf * delta0 * dt);
    }
    return;
  }
};



//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state,const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;


  // inputs

  const double x    = state[0];
  const double y    = state[1];
  const double psi  = state[2];
  const double v    = state[3];
  const double cte  = state[4];
  const double epsi = state[5];


  /**
   * TODO: Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   * TODO: Set the number of constraints
   */
  size_t n_vars = 6 * N + 2 * (N - 1);
  size_t n_constraints = 6 * N;



  // ------------------------------------------------------
  //
  //                INITIALIZATION
  //
  // ------------------------------------------------------
  
  Dvector vars(n_vars);
  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // bounds of [x, y, psi, v, cte, epsi]_t \in [-inf, inf]
  for (int i = 0; i < delta_start; i++){
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // bounds of delta \in [-25, 25] [deg]
  for (int i = delta_start; i < a_start; i++){
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] =  0.436332*Lf;
  }
  // bounds of a \in [-1,1] [m/s^2]
  for(int i = a_start; i < n_vars; i++){
    vars_lowerbound[i] = -1;
    vars_upperbound[i] =  1;
  }

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  // bounds of [x, y, psi, v, cte, epsi]_t+1 - [x, y, psi, v, cte, epsi]_t
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]     = x;
  constraints_lowerbound[y_start]     = y;
  constraints_lowerbound[psi_start]   = psi;
  constraints_lowerbound[v_start]     = v;
  constraints_lowerbound[cte_start]   = cte;
  constraints_lowerbound[epsi_start]  = epsi;

  constraints_upperbound[x_start]     = x;
  constraints_upperbound[y_start]     = y;
  constraints_upperbound[psi_start]   = psi;
  constraints_upperbound[v_start]     = v;
  constraints_upperbound[cte_start]   = cte;
  constraints_upperbound[epsi_start]  = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs, this->weights_);




  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  /**
   * TODO: Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */

  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for (int i=0; i < N-2; i++){
    result.push_back(solution.x[x_start+i+1]);
    result.push_back(solution.x[y_start+i+1]);
  }

  return result;
}