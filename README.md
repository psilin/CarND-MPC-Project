# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Overview

Project is dedicated to using Model Predictive Controller to allow car autonomously steer in Udacity's simulator. At each
epoch telemtry information is obtained from simulator. It consists of car state (in map coordinate system), control values
(throttle and steering angle) and a number of trajectory points. Model Predictive Controller sends back actual control
values to simulator. There is a delay of `100` ms between computing actual control values and sending it back to simulator 
to further complicate the task. Implementation of a Model Predictive Controller is discussed in the following section. Video
of car running a lap controlled by this implementation is available [here](https://youtu.be/GFi-d95nyCc).

## Implementation

Implemenation will be described in order of computational flow, step by step.

### 1. Data preparation.

First of all trajectory points obtained from simulator are transformed to car inner coordinate system as all computations
are performed in this coordinate system. Transformation is based on current car state which is obtained from simulator as 
well `./src/main.cpp`, lines `109-117`. 3rd degree polynomial curve is used to fit trajectory points. This curve used as a
base trajectory car should drive as close as possible to (fitting in `./src/main.cpp`, line `120`). During lesson it was said 
that 3rd degree polynomial curves are enough to approximate most type of the roads. This curve is later used as an input to
optimization problem.

### 2. Kinematic model.

The following kinematic model (to be orecise its update equations) for car movement is used in this project:
```c
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt;
v[t+1] = v[t] + a[t] * dt;
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt;
```

where `[x,y, psi, v, cte, epsi]` is the state of a car (`x` and `y` are car's coordinate, `psi` is a car's orientation,
`v` is a car's speed, `cte` is a car cross-track error relative to reference trajectory and `epsi` is an orientaion error
to reference trajectory), `a` and `delta` are control values (`a` is car throttle and `delta` is a steering angle), `f(x)` is
an `y` coordinate of a reference trajectory for a given `x` coordinate, `psides` is a tangencial angle of a reference trajectory,
`Lf` is a distance between front of car and its center of gravity (it is a predefined constant for this project).

### 3. Dealing with latency.

There is two approaches to deal with latency that immediately come to mind. First is to use optimization procedure. Optimization
procedure produces `N-1` pairs of control values (where `N` is a number of timesteps). We use the first pair of control values
which help us to move car from state at time `t` to state at time `t+1` (or to be more precise `t + dt` where `dt` is length of
timestep). Instead we can use `n`-th pair of control values, where `n` is the pair of control values used to move car from state 
`t + delay` to `t + delay + dt`, where `delay` is equals to latency. Second approach is to use a kinematic model to predict at
what state car will be at time `t + delay` given a car state a time `t` and then use standard optimization (as it is in a system 
without delay) to move car to state at time `t + delay + dt`. It is important to mention that kinematic model update step is described
in car inner coordinate system, so it will look like:

```c
x1 = 0. + v * cos(0) * dt;
y1 = 0. + v * sin(0) * dt;
psi1 = 0. - v * steer_value * dt / Lf;
v1 = v + throttle_value * dt;
cte1 = cte + v * sin(epsi) * dt;
epsi1 = epsi - v * steer_value * dt / Lf;
```

Dealing with latency can be found at `./src/main.cpp`, lines `125-131`.

### 4. Optimization procedure horizon.

In these sections parameter tuning for solver will be described. First of all is prediction
horizon. It consists of two variables `N` and `dt`, where `N` is a number of prediction
steps and `dt` is a length of prediction step. So in ideal world the prediction procedure
should take car from state at moment `t` to state at moment `t + N * dt`. But we actually
use only first pair of produced control values so it will take car to state at time `t + dt` only.
And on the next step a new iteration of optimization procedure will be performed to state at time `t + dt`.
I tried various `N` in span of `[6, 20]` but ended with `N = 10`. I noticed that there was no significant
difference in car behaviour between `N = 10` and `N = 20` cases so I decided to avoid unnecessary computations
and chose `N = 10`. I tried to further decrease `N` to `8` and then `6` but I noticed that it was much harder
to chose correct cost function as curves produced by optimization procedure bacame really unstable at times
so I decided to return to `N = 10`. As for `dt` I ended with `dt = 0.1`. I started with `dt = 0.5` as it was in
lesson's example but I realized that it was not efficient to look to far in the future as we tend to approximate
given reference curve not so great near our car's current position which was crucial to MPC approach so I moved
to `dt = 0.1` and was able to get decent results. I decided against further decreasing `dt` as in my opinion
the approximation of reference curve was of enough quality already. So I ended up with `N = 10` and `dt = 0.1` and
prediction horizon `N * dt` of `1` second.

### 5. Optimization procedure constraints.

For all state variables constraints are chosen in `[-1.0e19, 1.0e19]` interval, so state variables can be almost
anything despite some unreasonably small or big values. Constraints for steering angle is `[-25., 25.]` degrees and
for throttle is `[-1., 1.]` which is exactly all possible values accepted by simulator. These constraints can be
found at `./src/MPC.cpp`, lines `179-197`.

### 6. Optimization procedure cost function.

Cost function is the heart of optimization procedure and is constructed to minimize the following parameters across 
all `N` optimization epochs:

```c
    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += 1000 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1000 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 1000000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 100 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

```

Function is constructed as a sum of squared parameters. First group corresponds to state parameters.
First two corresponds to minimization cross-track and orientation errors, third actually to make sure
that moving speed is not deviating much from reference speed (`50` mph in our case). This third parameter
is introduced to make sure that car optimal movement will be actually movement across reference curve and not
stopping at a point on the map. Second group of parameters is chosen to minimize usage of controll values,
the less control should be used to keep car on the reference curve the better. It ensures that car is moving 
in a relatively smooth way across the track. Third group of parameters is chosen to ensure that control values
are applied in a smooth way, and not differs much from epoch to epoch. This is an approach to make sure that control
is applied in a predictable way. Which is crucial as we only use the first pair of control values. Coefficients
before the summands are used to tune cost function and determine what parameters are more important then the others.
As it can be seen above tne most crucial parameter is difference between adjacent steering angles. During tuning
procees I found out that it was the most important parameter that was preventing car movement trajectory from divergence.
As it can be seen minimization of cross-track and orientation errors is important as well. The least important is
speed related parameters as in fact it is only needed to make sure that car is moving with something close to predefined 
speed. It is worth to mention that in the end throttle related parameters ended up to be less important than steering angle
related which is of no surprise as steering is the key to keep car on the track. Cost function implementation can be found 
at `./src/MPC.cpp`, lines `49-68`.

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
