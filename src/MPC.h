#ifndef MPC_H
#define MPC_H

#include <vector>
#include <chrono>
#include <ctime>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  std::chrono::high_resolution_clock::time_point previous_timestamp;
  bool safe_to_drive;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
