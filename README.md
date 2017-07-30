# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

In this project we will revisit the lake race track from the Behavioral Cloning Project. This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track! The simulator provides the cross track error (CTE) and the velocity (mph), we need to compute the appropriate steering angle based on the CTE and velocity.

In this project, the hyperparameters were chosen by manually tuning to get the desized maneuver on the race track. To tune the hyperparameter Kp, I shmooed from 0.01 and increased by 0.01 until I reached 0.1.  Since Kp tends to overshoot, the small Kp tends to cause the car out of track at the sharp turn.  To avoid overshoot issue in Kp, the hyperparameter Kd is used by taking the derivative of the CTE error.  However, choosing a large value of Kd may cause the car to start wiggling around.  In this project, I chose Kd=4.  The hyperparameter Ki is the implementation of the sum of all the CTE error over time. So if this Ki is too large, it causes the car to run out of track quickly.  In this project, I chose Ki = 0.005.

Here is the summary of the final hyperparameters that I finally picked:
Kp = 0.1
Kd = 4
Ki = 0.005

You can view the final video of how the car successfully passed the track without going off the track at the top speed of 32mph: https://youtu.be/vZ91Xy61kgc

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

