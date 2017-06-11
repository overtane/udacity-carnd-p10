# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## The Model

This work contains a model predictive controller for a simulated car on a race track. 
It uses kinematic vehicle model consisting of state [x, y, psi, v, cte, espi] and
actuations [delta, a]. These variablse are:

  * x: x position
  * y: y position
  * psi: orientation angle
  * v: velocity
  * cte: cross track error
  * epsi: oritentation error
  * delta: steering angle
  * a: acceleration (throttle value in this model)

Given a reference trajectory (for example a center line of the lane to follow) and current 
vehicle state, the implementation tries to minimize the errors between vechicles trajectory and
the reference trajectory within the given prediction horizon.

The solution consist of the main source code modules `main.cpp` and `MPC.cpp`. The former contains 
communication with the simulator, feeding vehicle's state to the model and passing the calculated
actuation back to the simulator.

`MPC.cpp` contains the model. It uses [Ipopt non-linear optimization package](https://projects.coin-or.org/Ipopt)
for solving the optimization task (actuator values). As part of the input, we calculate vehicles predicted 
state within the horizon for different time steps (`MPC.cpp`):

    fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
    fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
    fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
    fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
    fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
    fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

Important part of the input to the solver, is the cost values. The cost values present the relative importance
of different state variables on the optimization problem. In this implementation, cost value are defined like this:

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;


    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 1   * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1   * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 1   * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Cost terms controlling magnitude of the actuators
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 500 * CppAD::pow(vars[delta_start + t], 2); // multiplier here dampens steering actions
      fg[0] += 1   * CppAD::pow(vars[a_start + t], 2);     // multiplier here cuts down the max speed
    }

    // These control the magnitude of the change of the actuators
    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 50000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10000 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

The relative importance of cross track error and error change has been increased using multipliers. 
These where found by experimenting. 


## Timestep Length and Elapsed Duration (N & dt)

*The prediction horizon* is the duration over which future predictions are made. The length of the 
horizon is the product of variables N and dt.
N is the number of timesteps in the horizon. dt is the time between actuations in seconds. 

This solution uses one second horizon, N == 20 and dt == 0.05. This was found suitable for simulation on
the hardware at hand. Increasing dt to 0.1 did not give accurate enough calculations (difficult to keep
the vehicle on track). On the other hand, decrasing dt would shorten the length of the horizon. When N was
increased over 20, calculation introduced too much overhead.

## Polynomial Fitting and MPC Preprocessing

The simulator gives a set of waypoints to controlller. These waypoints represent the 
reference trajectory the vehicle to follow. Typically, in a autonomous vehicle, the path planning
module produces the waypoints.

The waypoints are presented in a global (map's) coordinate system and must be converted to vehicle's
cordinate system. Vehicle is always at the origin (0,0) of its own coordinate system. If the point (`px`,`py`) is 
vehicle's location in map coordinates and vectors `ptsx` and `ptsy` contain the waypoints, coordinate transformation is 
done in a loop (`main.cpp`): 

    for (int i=0; i< ptsx.size(); i++) {
      xvals(i) =  (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
      yvals(i) =  (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
    }

After the transformation, we can fit the waypoints into a polynomial repesenting the reference trajectory.
Third order polynomials are usually adequate for this purpose. (`main.cpp`)

    Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
         
Once we have the reference trajectory, we can calculate cross track error (CTE) and orientation error (EPSI). 
Current CTE is the difference between the current location and the reference trajectory. In vechile's coordinate 
system this is simply:
 
    double cte = polyeval(coeffs, 0);

Orientation error is the the angle between the tangent line of the reference trajectory and vehicle's orientation.
Vehicle always points to x-axis direction (`psi==0`), so we get the error from the derivative of trajectory at zero. 
('handness' of map and vehicle coordinate systems are different, thus the minus):

    double epsi = -atan(coeffs[1]);

But in reality, we need to take into account the latency of actuation in the error calculations. This is explained next.


## Model Predictive Control with Latency

In real life, there is always a delay between control calculation and control actuation. This is called latency.
The latency can be handled by feeding into solver a predicted vehicle state. We predict the state of the vehicle after
the expected latency using to the vehicle model. (Steering angle is normalized to range [-1,1] and it is converter to radians.)
(`main,cpp`)

    double latency = 0.1; // 100 ms
    double x_pred = v * latency;
    double y_pred = 0.0;
    double psi_pred = -v * steer_angle * deg2rad(25) * latency / 2.67;

Cross track and orientation errors are then calculated at this predicted state. (`main.cpp`)

    double cte_pred = polyeval(coeffs, x_pred);
    double epsi_pred = atan(coeffs[1] + 2*coeffs[2]*x_pred);


---
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

