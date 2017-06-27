# MPC Controller Project

---

## Observations


### Model


The state model consists of the following components:
* x: x position
* y: y position
* psi: heading
* v: velocity
* cte: cross track error
* epsi: heading error


It also has the following actuators:
* delta: steering angle
* a: acceleration


Each step uses the following update equations:
(Lf is the distance from the front of the car to its center of gravity)
* x: x1 - (x0 + v0 * cos(psi0) * dt)
* y: y1 - (y0 + v0 * sin(psi0) * dt)
* psi: psi1 - (psi0 - v0 * delta0 / Lf * dt)
* v: v1 - (v0 + a0 * dt)
* cte: cte1 - ((evaluated_polynomial_with_coefficients_at_x0 - y0) + (v0 * sin(epsi0) * dt))
* epsi: epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);


### Choice of N and dt

The values N = 10 and dt = 0.1 were used as a starting point (from the [MPC project Q&A video](https://www.youtube.com/watch?v=bOQuhpz3YfU)). When N was increased to 20, the car turned erratically. Similarly, when dt was set to 0.2, or 0.05, the car could not follow the waypoints well. This is most likely due to the 100ms latency simulated by the system. The first time this project was submitted, the car drove smoothly around the track on my machine, but crashed on the reviewer's machine. To workaround the problems caused by unpredictable latency, the following changes were made:
* dt is now calculated based on time elapsed since the last MPC.solve() call. 
* ref_v (reference velocity) is now adjusted depending on how good/bad the latency is.
* If the latency is too bad (> 400ms), a safety mechanism kicks in and will not allow the car to continue driving.


### Prepocessing

The list of waypoints ptsx and ptsy were transformed so that they centered around 0. The same thing for the vehicle heading. This was done to reduce the number of transforms needed later on.

### Dealing with latency

The values of N and dt were selected so that we look one second ahead, and produce predicted values that work well with the latency of 100ms.

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
