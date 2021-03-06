#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set up our timestep and duration for our MPC controller
size_t N = 10;
double dt = 0.3;

// This is the length from the front of the car to its center of gravity
const double Lf = 2.67;

// Ideally, both our cross track error (cte) and heading error (espi) should be 0
// We also aim for a reference velocity of 100
double ref_cte = 0;
double ref_espi = 0;
double ref_v = 100;

size_t t;
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    // The fg vector contains the costs and constraints that optops will optimize for
    // We start by setting up fg[0], which is the cost function

    ///////////
    // Costs //
    ///////////
    fg[0] = 0;
    // A very large weight of 15000 was used for the cost of the
    // cross track error and the heading error, relative to the velocity.
    // This means that the solver will prioritize minimizing cte and epsi
    // even if it comes at a cost of other properties like velocity

    for (t = 0; t < N; t++) {
      fg[0] += 20000 * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
      fg[0] += 20000 * CppAD::pow(vars[epsi_start + t] - ref_espi, 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    for (t = 0; t < N - 1; t++) {
      fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
    }

    for (t = 0; t < N - 2; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /////////////////
    // Constraints //
    /////////////////
    // fg[0] has been reserved for the cost function, 
    // so everything else gets moved forward by 1
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Set up the update equations
    for (t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * CppAD::pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1]);

      // We want to constrain the following values to zero

      // x1 minus predicted_x1
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

      // y1 minus predicted_y1
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

      // psi1 (heading) minus predicted_psi1 (heading)
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);

      // v1 minus predicted_v1 (velocity)
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      // cte1 minus predicted_cte1 (cross track error)
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

      // epsi1 minus predicted_epsi1 (heading error)
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  this->previous_timestamp = std::chrono::high_resolution_clock::now();
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Calculate our dt timestamp
  std::chrono::high_resolution_clock::time_point newtime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> fg_dt = std::chrono::duration_cast<std::chrono::duration<double>> (newtime - this->previous_timestamp);

  // Depending on our latency, we need to adjust our maximum speed and rework our N and dt values
  // 100ms -> look 1 sec ahead
  // 200ms -> look 2 secs ahead
  // dt = 0.1 -> 80
  // dt = 0.2 -> 60
  // dt = 0.3 -> 40
  // dt = 0.4 -> 20
  // dt >= 0.5 -> 0  
  // 1000ms -> look 10 secs ahead, can safely move 1 meter before things get dangerous.
  
  dt = fg_dt.count();

  safe_to_drive = true;
  if (dt <= 0.1) { ref_v = 100; }
  else if (dt <= 0.2) { ref_v = 80; }
  else if (dt <= 0.3) { ref_v = 60; }
  else if (dt <= 0.4) { ref_v = 30; }
  else {
    std::cout << "\nWARNING: The latency is too high. It is not safe to drive. Blocking input to actuators.\n" << std::endl;
    safe_to_drive = false;
    ref_v = 0; 
  }

  // double estimated_v = 1000 / (dt * 1000);
  // ref_v = std::min(estimated_v, 100.0);
  std::cout<< "dt:" << dt << "  ref_v:" << ref_v << std::endl;
  this->previous_timestamp = newtime;
  

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // We have 6 state variables across N timesteps (6 * N)
  // plus 2 actuators getting us from one timestep to the other (2 * (N-1))
  size_t n_vars = N * 6 + (N - 1) * 2;
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332 * Lf;
    vars_upperbound[i] = 0.436332 * Lf;
  }

  // Acceleration/decceleration upper and lower limits.
  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
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

  // Return the first actuator values (delta and a), AND
  // the predicted points by the MPC controller (xs and ys)
  vector<double> result;

  // Only pass the actuator input if it is safe to drive!
  if (safe_to_drive) {
    result.push_back(solution.x[delta_start]);
    result.push_back(solution.x[a_start]);
  } else {
    result.push_back(0);
    result.push_back(0);
  }

  for (i = 0; i < N; i++) {
    result.push_back(solution.x[x_start + i]);
    result.push_back(solution.x[y_start + i]);
  }

  return result;
}
