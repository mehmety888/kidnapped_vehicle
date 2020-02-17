# Self Driving Car Nanodegree Kidnapped Vehicle Project
This repository contains an implementation of Udacity's Kidnapped Vehicle Project, using 2D Particle Filter.

## Project Introduction

Inputs:
* A map of location
* A (noisy) GPS estimate of its initial location of vehicle
* At each time step, noisy sensor(observation) and control data.

Outputs:
* **Accuracy**: particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

* **Performance**: particle filter should complete execution within the time of 100 seconds.

## Running the Code
This project involves Udacity Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh
