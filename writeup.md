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


##PID class (`PID.h`, `PID.cpp`)

The steering and speed PIDs are implemented through a class `PID` (see `PID.h` and `PID.cpp`) that defines the following variables and methods:

- `p_error`, `i_error`, `d_error`: Respectively, error values on position , integral of position (i.e., cumulated values of `p_error` over time) and derivative of position (i.e., difference between two successive values of `p_error`).
- `Kp`, `Ki`, `Kd`: Controller parameters. Respectively, weights of the P, I and D errors in the controller.
- `Init()`: Controller initializer. Sets the parameter values to the values of the arguments passed in the method call. Also initializes the P and I errors to 0.

Note that in the steering PID, `p_error` corresponds exactly to CTE (cross-track error), however in the speed PID, the calculation is slightly more complex (but the filter itself is simpler, see below in section "Speed PID implementation").


##Speed PID implementation

Initial trials with a set throttle input were failing because the car was continuously accelerating regardless of its position on track. It therefore became quickly apparent that speed would need to be controlled in some way.

The ideal controller needs to accelerate the vehicle while it is driving in a straight line and near the center of the track, but stop throttling or even start braking when the car gets close to an edge, or when the track turns sharply. It should also set some target speed to prevent the car from accelerating to speeds where things will happen too fast for the steering controller to be able to effectively navigate the track.

From these requirements, it appears that CTE only is not enough information to build such a controller. Applying some logic and testing, I eventually selected a combination of three variables as my controller input error:

- Difference between current and target speeds
- Steering angle in absolute value
- CTE in absolute value

After some trial and error, I defined the input value of the speed controller as a linear combination between these three:

$ throttle_error = (speed - target_speed) + 10 \abs{steer_value} + 5 \abs{cte} $

On the other hand, there is no real reason to include the integral error in the controller: It doesn't really matter what the car has been doing speed-wise since it started driving. we are only interested in
