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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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

  // MPC is initialized here!
  MPC mpc;
  bool title = false;

  h.onMessage([&mpc,&title](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];  // waypoints x, map coordinate
          vector<double> ptsy = j[1]["ptsy"];  // waypoints y, map coordinate
          double px = j[1]["x"];               // car position x, map coordinate
          double py = j[1]["y"];               // car position y, map coordinate
          double psi = j[1]["psi"];            // car orientation (angle with map's x-axis), radians
          double v = j[1]["speed"];            // speed mph
          double steer_angle = j[1]["steering_angle"];

          // change map waypoints to car coordinates
          Eigen::VectorXd xvals(ptsx.size());
          Eigen::VectorXd yvals(ptsy.size());

          for (int i=0; i< ptsx.size(); i++) {
            xvals(i) =  (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
            yvals(i) =  (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
          }

          // fit the waypoints to a polynomial. expect curvy road -> three degrees 
          Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
         
          // predict the state after expected latency 100 ms
          double latency = 0.1; // 100 ms
          double x_pred = v * latency;
          double y_pred = 0.0;
          double psi_pred = -v * steer_angle * deg2rad(25) * latency / 2.67;

          // calculate the cross track error in car coordinates
          double cte_pred = polyeval(coeffs, x_pred);
          // calculate the orientation error in car coordinates
          double epsi_pred = atan(coeffs[1] + 2*coeffs[2]*x_pred);

          // Calculate steeering angle and throttle using MPC.
          Eigen::VectorXd state(6);
          state  <<  x_pred, y_pred, psi_pred, v, cte_pred, epsi_pred;
          vector<double> actuators = mpc.Solve(state, coeffs);

          double cost           = actuators[0];
          // normalize steer value to [-1,1]. divide by deg2rad(25), which is max by controller
          double steer_value    = -actuators[1] / deg2rad(25);
          double throttle_value = actuators[2];

          if (!title) {
            std::cout 
                << std::setw(10) << "CTE"
                << std::setw(10) << "ePSI"
                << std::setw(16) << "Cost"
                << std::setw(10) << "Steer"
                << std::setw(10) << "Throttle"
                << std::endl; 
            title = true;
          }

          std::cout << std::fixed << std::setprecision(4) 
              << std::setw(10) << cte_pred
              << std::setw(10) << epsi_pred
              << std::setw(16) << cost
              << std::setw(10) << steer_value
              << std::setw(10) << throttle_value 
              << std::endl; 

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i=3; i < actuators.size(); i += 2) {
              mpc_x_vals.push_back(actuators[i]);
              mpc_y_vals.push_back(actuators[i+1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i=0; i < xvals.size(); i++) {
              next_x_vals.push_back(xvals(i));
              next_y_vals.push_back(yvals(i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(int(1000*latency)));
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

