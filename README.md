# Kidnapped Vehicle, Particle Filter Project for Self-Driving Car Engineer Nanodegree Program

Author: David Escolme
Date: 12 July 2018

## Project Objectives

A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we will implement a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter will also get observation and control data.

## Running the Code - Copied from original Udacity README

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project - please review the commands in these for your own environment settings:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"]

["sense_y"]

["sense_theta"]

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"]

["sense_observations_y"]

OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"]

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


# Software structure
The directory structure of this repository is as follows:

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |
|   |   map_data.txt
|
|
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

# particle filter structure

The particle filter class has 4 key elements:

+ Init: Used to set the number of particles (either a default value or a value entered as a command line parameter) and set each particle's initial x, y and theta value to be the initial GPS data with added sensor noise using a gaussian distribution
+ Predict: Takes sensed velocity and yaw_rate data together with noise parameters to update each particle's expected x, y, theta value using bicycle motion models with added gaussian noise
+ Update and Data Association: Takes environment observations and transforms the coordinates with respect to each particle in turn to Map coordinates, then matches the observations to each landmark. This enables rach particle's weight to be updated which is, in essence, a probability of the particle's plausability
+ Resample: Populate a new set of particles drawn from the existing set accounting for the probability (weight) of each particle

Over time, this then leads to a 'zero-ing' in on the car's actual position with respect to its environment.

# Discussion

A default value of 100 particles allows the filter to operate within tolerance of accuracy and operation time. Using the command line parameter, some experiements were run to determine the minimum/maximum particle count for success. In this case we get:

| particle size           | status             | x error | y error | yaw error | operation time |
|-------------------------|--------------------|---------|---------|-----------|----------------|
| 1000                    | failure (time)     | 0.107   | 0.102   | 0.004     | 152            |
| 500                     | failure (time)     | 0.108   | 0.099   | 0.004     | 101.68         |
| max between 475 and 500 |                    |         |         |           |                |
| 475                     | success            | 0.110   | 0.103   | 0.004     | 98.12          |
| 50                      | success            | 0.123   | 0.116   | 0.004     | 79.54          |
| 7                       | success            | 0.161   | 0.142   | 0.006     | 68.54          |
| 6                       | failure (accuracy) | 1.131   | 0.359   | 0.042     | 70.66          |

Given that the GPS / sensor information is subject to noise and that the Map is of a certain size, I suppose that the minimum particle size is in some way correlated to both of these external parameters, whereby a minimum number of particles is needed to cover the variation in measurement.

Conversely, with more particles comes more computation time to predict and update and associate each particle at each measurement cycle.

This suggests that particle design must account for these factors to arrive at a filter that can predict location with adequate accuracy and speed for the task in hand.


