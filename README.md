# PID Controller Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This project implements a PID (Proportional Integral Derivative) controller to use with the Udacity car simulator.  With the PID running, the car will autonomously drive around the track, adjusting the steering based on a given CTE (Cross Track Error) value.

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

# Project Goals and [Rubric](https://review.udacity.com/#!/rubrics/824/view)

The goals of this project are the following:

* The PID controller must be implemented as was taught in the lessons.
* An implementation of the twiddle algorithm tunes the hyperparameters.
* The vehicle must successfully drive a lap around the track.

# Implementation of the PID controller

Implementing the PID controller was somewhat trivial because the it was already done in the [lab](https://github.com/justinlee007/CarND-PID-Lab/blob/master/src/python/robot.py#L132) 

# Tuning the hyperparameters

# Other lessons learned

