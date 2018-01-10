#PID Controller Applied to Driving Simulator
##Udacity Self-Drviing Car Nanodegree Term2, Project 4

---

##Project requirements

The purpose of this project is to code a PID controller and apply it to a 3D driving simulator in order to successfully navigate a track. In order for the project to be considered as completed, the car must drive around the circuit at least once without leaving it at any point, i.e. with no tire leaving the tarmac portion of the track.

The track contains a mixture of gentle and fairly sharp bends. There is no time requirement but maximizing the car's speed around the track introduces a fun challenge.

The project must be coded in C++. A `main.cpp` file was provided that implements the communication with the simulator; however I modified it quite extensively.


##Technical environment

For this project, I ran the simulator in Windows (based on the Unity game engine) and the Windows 10 Ubuntu Bash to edit and run the code.

The code was written in C++11 in Sublime Text and built with CMake / make. 

**Specifications:**

Intel Core i5-4440 3.10GHz x64
RAM 12 GB
Windows 10 Pro version 10.0.15063
Bash: Ubuntu 16.04 LTS


##Communication protocol with simulator

The `main.cpp` file implements communication to and from the simulator through uWebSockets:

- After initialization and at each cycle, `main.cpp` receives messages from the simulator in the shape of JSON data.
- A message starting with "telemetry" contains the following information:
  - "cte": Cross-track error of the vehicle, relative to the center of the track
  - "speed": Currently observed speed of the vehicle, units unknown (kph or mph)
  - "angle": Currently observed steering angle of vehicle, scaled from -1 to +1
- `main.cpp` can generate JSON data containing the following fields:
  - "throttle": Throttle instruction, scaled from -1 (maximum brakes / reversing) to 1 (maximum forward acceleration)
  - "steering_angle": Steering instruction, scaled from -1 to +1. 1 corresponds to 25 degrees of steering angle.
- The JSON data is then sent to the simulator through uWebSockets and the vehicle's controls are adjusted accordingly.

Our task in this project is to generate the right steering instructions to keep the car on track. Although not a strict project requirement, I found that also computing throttle instructions was extremely beneficial.


##PID class

The steering and speed PIDs are implemented through a class `PID` (see `PID.h` and `PID.cpp`) that defines the following variables and methods:

- `p_error`, `i_error`, `d_error`: Respectively, error values on position , integral of position (i.e., cumulated values of `p_error` over time) and derivative of position (i.e., difference between two successive values of `p_error`).
- `Kp`, `Ki`, `Kd`: Controller parameters. Respectively, weights of the P, I and D errors in the controller.
- `Init()`: Controller initializer. Sets the parameter values to the values of the arguments passed in the method call. Also initializes the P and I errors to 0.

Note that in the steering PID, `p_error` corresponds exactly to CTE (cross-track error), however in the speed PID, the calculation is slightly more complex (but the filter itself is simpler, see below in section "Speed PID implementation").



