# CarND-Controls-MPC

Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5

  * All OSes: [click here for installation instructions](https://cmake.org/install/)

* make >= 4.1(mac, linux), 3.81(Windows)

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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.

* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.

* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Write-up

The MPC is an essential system of the SDC that, similar to localization, is a realization of itself according to it's projection of the world. The problem it solves is as stated: how should the SDC move itself via physical actuation towards a given desired path? First the waypoints given by the car simulation must be converted to the car's perspective. The car's coordinate system is defined by the car's forward motion aligned with the positive x-axis. The global waypoint coordinates relative to the car's global position is rotated by the car's global psi value. The normalized waypoints are generated as a polynomial model. 

For the predictive portion of MPC, the intial state, update equations, and an optimizer with a cost function are used. The update equations for x, y, v, and psi are based off the kinematic equations. The error update equations also use kinematics to project where the error will be based on the previous state. The erorr psi update equation is different than from the lesson which defines `epsi(t)=psi(t) - psides(t)` whereas I use `epsi(t)= psides(t) - psi(t)` since the cte update is also in the form of desired - actual.

>`x(t+1) = x(t) + v(t)*cos(psi(t))*dt`

>`y(t+1) = y(t) + v(t)*sin(psi(t))*dt`

>`psi(t+1) = psi(t) + v(t)*delta(t)*dt/Lf`

>`v(t+1) = v(t) + a(t)*dt`

>`cte(t+1) = y(t) - f(x(t)) + v(t)*sin(epsi(t)*dt)`

>`epsi(t+1) = psides(t) - psi(t) + v(t)*delta(t)*dt/Lf`

To account for latency, the initial state can be used to account for the trajectory during the delay. After the coordinates are all transformed to the car's coordinate system, we first want to consider the state at the origin where px=py=psi=0. The cte and epsi can be calculated using the polynomial and polynomial derivative, although most of the terms are zeroed since px=0. From there, the car's steering angle can be used with the state at the car's origin to calculate the state after the latency using the kinematic model.

The N and dt values determine how far out the car models its motion and the resolution of the calculated motion path. If the car doesn't have enough prediction length or points, it might not have enough of the predicted motion to correct itself on curves. If a combination of a small dt or large N, the processing time will increase and extra latency would have to be added on top of actuation delay. If the car predicts too far ahead, that environment changes could render the predictions obsolete. Ideally, velocity would factor into choosing N and dt parameters since the trajectory distance is based on time and speed.  

The weights for the minimizing cost function are used to determined how the model predicts motion relative to the desired path.  Empirically, the CTE and EPSI cost have the biggest influence on conforming the prediction motion to the desired path. Tuning the velocity and throttle actuation costs change how the car accelerate and deccelerate on straightaways and turns. The steering related costs prevent the car from moving erratically. The scale of the errors should be taken into consideration when choosing weights.

Once the predicted motion is optimized to minimize the given cost function, the actuation at t+1 is used to steer and throttle the car.
