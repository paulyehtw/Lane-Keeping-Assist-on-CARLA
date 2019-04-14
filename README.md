# Lane-Keeping-Assist-on-CARLA
## Introduction
<p align="center">
  <img  src="https://github.com/paulyehtw/Lane-Keeping-Assist-on-CARLA/blob/master/controller_output/CARLA.png">
</p>

This is an module assgnment from Motion Planning for Self-Driving Cars course of Self-Driving Cars Specialization on Coursera.org.

This assginment implements Lane Keeping Assist function by applying pure pursuit method for lateral control and PID controller for longitudinal control using Python as the programming language.

The waypoints and corresponding velocities for the track are pre-defined.

To realize this function, the open sourse simulator [CARLA](http://carla.org) is introduced.

## Prerequisites
First CARLA must be installed on your machine, the CARLA loader requires **Ubuntu 16.04 or later** to run

Please go through **CARLA-Setup-Guide-_Ubuntu_.pdf** and install CARLA and all other dependencies properly.

## How to run it
First clone this repository and put it under **PythonClient** directory.

### 1. Load the simulator
Open a terminal and do `cd ~/opt/CarlaSimulator`.

Then do `./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark -quality-level=Low -fps=30
`
### 2. Run the LKA controller
Open another terminal and do `cd ~/opt/CarlaSimulator/PythonClient/Lane-Keeping-Assist-on-CARLA`.

Then do `python3 module_7.py`.

The vehicle should starting driving and following the track.

## Simulation results
The image shown below is the result of vehicle trajectory.

The green line is the track(ground truth) and the orange line is the trajectory.
<p align="center">
  <img  src="https://github.com/paulyehtw/Lane-Keeping-Assist-on-CARLA/blob/master/controller_output/trajectory_good.png">
</p>

#### TODO
1. Implement Feed Forward method for longitudinal control.
2. Implement Stanley and MPC approaches for lateral control.
