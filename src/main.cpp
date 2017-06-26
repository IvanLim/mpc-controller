#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {

  uWS::Hub h;
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          size_t N = 25;

          // Translate and re-center all the points around (0, 0), and around 0 degrees rotation
          for (size_t i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;

            ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
            ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
          }

          // Convert the std::vectors to  Eigen::VectorXds
          Eigen::VectorXd ptsx_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsx.data(), ptsx.size());
          Eigen::VectorXd ptsy_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptsy.data(), ptsy.size());

          // Find the coefficients of a 3rd degree polynomial that fits our transformed points
          auto coeffs = polyfit(ptsx_vec, ptsy_vec, 3);
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          // Set up our initial state (0, 0) with 0 degrees rotation
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // Feed the state and our polynomial coefficients to our MPC controller
          // The returned results contains
          auto vars = mpc.Solve(state, coeffs);

          //Display the waypoints/reference line (Yellow)
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          for (size_t i = 1; i < N; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          //Display the MPC predicted trajectory (Green)
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // If we include the first 4 elements (two x, y points), the line gets
          // drawn through the car and it looks bad. So we start at 4 to skip them.
          for (size_t i = 4; i < vars.size() - 1; i+= 2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i + 1]);
          }

          // Lf is a constant representing the distance from the
          // front of the car to its center of gravity
          const double Lf = 2.67;

          json msgJson;

          // Steeting angle needs to be normalized between -1 and 1 for the simulator
          // That means dividing by 25 degrees (since the simulator steering angle
          // goes from -25 to 25)
          msgJson["steering_angle"] = vars[0] / (deg2rad(25) * Lf);;
          msgJson["throttle"] = vars[1];

          // Include the points for the MPC line (green)
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Include the points for the waypoints / reference lines (yellow)
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Add a 100ms latency to simulate a real life delays in actuation
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
