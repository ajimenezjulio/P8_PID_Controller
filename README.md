## PID Controller
[![C++](https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B)](http://www.cplusplus.org/)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we use a PID controller to control the steering angle and maximum speed possible that allows our car to stay on the road in a virtual environment. This project involves the Udacity Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

<img src="https://github.com/ajimenezjulio/P8_PID_Controller/blob/master/docs/pid.gif">

## Goal
In this project the objective is to drive at the maximum possible speed through a road with steep curves (70 mph were reached) using a PID controller to control the steering angle that our car must turn to stay on the road using the cross track error (`cte`) as the parameter to minimize.

The simulator will provide at each point with the cross track error (`cte`), the steering angle and the speed, which will be treated by the controller to respond appropriately to the changes. 

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
    * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
    * If you install from source, checkout to commit `e94b6e1`, i.e.
      ```
      git clone https://github.com/uWebSockets/uWebSockets 
      cd uWebSockets
      git checkout e94b6e1
      ```
      Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
 * Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Another option is this guide to Windows set-up for the project [(here)](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Running the Code
A script for the process of cleaning, directory creation, build and execution was provided, so you only have to run it from the project root.
```
> ./clean_and_make.sh
```
For a manual approach the next commands should be executed.
```
> rm -r build
> mkdir build && cd build
> cmake .. && make
> ./pid
```

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## Implementation
The directory structure of this repository is as follows:

```
.
├── CMakeLists.txt
├── README.md
├── clean_and_make.sh
├── src
    ├──  json.hpp
    ├──  PID.h
    ├──  Twiddle.h
    ├──  Twiddle.cpp
    ├──  PID.cpp
    ├──  main.cpp
```

## Details
**PIDs**
1. Firstly, two PID instances (one for steering and one for speed) are initialized with the following parameters in P I D order: `steering = 0.045, 0.005, 0.9 |  speed = 0.33, 0.0027, 0.02` .
2. At each point each instance is updated using the simulator cte value.
3. Finally, the error of all the components is calculated and propagated to the control variable in the simulator.

**Twiddle**
1. An instance of the twiddle algorithm is created, the parameters to tune `p` as well as the increments `dp` must be given.
2. The flag `isTwiddleActive` must be set to use the twiddle algorithm to fine tune parameters P, I and D.
3. The algorithm will update the current parameter based on the established increment and will reset the state of the car, there is the possibility that the car may get stuck, so in addition to the state rest the error will be updated.

**Notes**
1. The algorithm was tested for the simulator in the `fastest` setting for `performance`, using a higher resolution such as `fantastic` led to unexpected behavior due to longer delay time for the update, unless you have a powerful enough computer these values may not work at higher resolutions.
1. The twiddle algorithm was used but the training process was too slow or erratic so manual trial and error intervention was used to reach the final value.
