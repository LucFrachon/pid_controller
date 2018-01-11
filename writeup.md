# PID Controller Applied to Driving Simulator
## Udacity Self-Drviing Car Nanodegree Term2, Project 4

---

## Project requirements

The purpose of this project is to code a PID controller and apply it to a 3D driving simulator in order to successfully navigate a track. In order for the project to be considered as completed, the car must drive around the circuit at least once without leaving it at any point, i.e. with no tire leaving the tarmac portion of the track.

The track contains a mixture of gentle and fairly sharp bends. There is no time requirement but maximizing the car's speed around the track introduces a fun challenge.

The project must be coded in C++. A `main.cpp` file was provided that implements the communication with the simulator; however I modified it quite extensively.


## Technical environment

For this project, I ran the simulator in Windows (based on the Unity game engine) and the Windows 10 Ubuntu Bash to edit and run the code.

The code was written in C++11 in Sublime Text and built with CMake / make. 

**Specifications:**

Intel Core i5-4440 3.10GHz x64
RAM 12 GB
Windows 10 Pro version 10.0.15063
Bash: Ubuntu 16.04 LTS


## Communication protocol with simulator

The `main.cpp` file implements communication to and from the simulator through uWebSockets:

- After initialization and at each cycle, `main.cpp` receives messages from the simulator in the shape of JSON data.
- A message starting with "telemetry" contains the following information:
  - "cte": Cross-track error of the vehicle, relative to the center of the track
  - "speed": Currently observed speed of the vehicle, in mph
  - "angle": Currently observed steering angle of vehicle, scaled from -1 to +1
- `main.cpp` can generate JSON data containing the following fields:
  - "throttle": Throttle instruction, scaled from -1 (maximum brakes / reversing) to 1 (maximum forward acceleration)
  - "steering_angle": Steering instruction, scaled from -1 to +1. 1 corresponds to 25 degrees of steering angle.
- The JSON data is then sent to the simulator through uWebSockets and the vehicle's controls are adjusted accordingly.

Our task in this project is to generate the right steering instructions to keep the car on track. Although not a strict project requirement, I found that also computing throttle instructions was extremely beneficial.


## PID class (`PID.h`, `PID.cpp`)

The steering and speed PIDs are implemented through a class `PID` (see `PID.h` and `PID.cpp`) that defines the following variables and methods:

- `p_error`, `i_error`, `d_error`: Respectively, error values on position , integral of position (i.e., cumulated values of `p_error` over time) and derivative of position (i.e., difference between two successive values of `p_error`).
- `Kp`, `Ki`, `Kd`: Controller parameters. Respectively, weights of the P, I and D errors in the controller.
- `Init()`: Controller initializer. Sets the parameter values to the values of the arguments passed in the method call. Also initializes the P and I errors to 0.

Note that in the steering PID, `p_error` corresponds exactly to CTE (cross-track error), however in the speed PID, the calculation is slightly more complex (but the filter itself is simpler, see below in section "Speed PID implementation").


## Speed controller implementation

Initial trials with a set throttle input were failing because the car was continuously accelerating regardless of its position on track. It therefore became quickly apparent that speed would need to be controlled in some way.

The ideal controller needs to accelerate the vehicle while it is driving in a straight line and near the center of the track, but stop throttling or even start braking when the car gets close to an edge, or when the track turns sharply. It should also set some target speed to prevent the car from accelerating to speeds where things will happen too fast for the steering controller to be able to effectively navigate the track.

From these requirements, it appears that CTE only is not enough information to build such a controller. Applying some logic and testing, I eventually selected a combination of three variables as my controller controller input error:

- Difference between current and target speeds
- Steering angle in absolute value
- CTE in absolute value

After some trial and error, I defined the input value of the speed controller as a linear combination between these three:
`throttle_error = (speed - target_speed) + 10 * |steer_value| + 5 * |cte|`
The linear coefficients 1, 10 and 5 are a result of experimenting with different values and are not highly optimized.

In terms of the controller itself, I wanted to keep it as simple as possible so although I used the `PID` class, I tried with only the P-parameter at a non-zero value (i.e., no integration nor derivation terms) and found that it worked well. I set the P parameter at 0.25 as it seemed to work but again, it has not been thoroughly optimized. The I and D parameters are set to 0.

My final P-controller for throttle input is therefore:
`throttle_value = -0.25 * [(speed - target_speed) + 10 * |steer_value| + 5 * |cte|]`
bounded to the [-1, +1] interval (i.e. if `|throttle_value|` > 1, it is capped to 1).

## Steering controller implementation

For steering, I am using a regular PID controller from the `PID` class. The input variable is CTE (provided by the simulator). The output is a steering value that is then normalized to [-1, +1], like throttle input previously.

The steering PID-controller can therefore be written as:
`steering_value = -(Kp * p_error + Ki * i_error + Kd * d_error)`
bounded to the [-1, +1] interval (i.e. if `|steering_value|` > 1, it is capped to 1).

## Considerations on the role of each parameter

As a reminder:
- `Kp` is the position parameter. It multiplies the cross-track error relative to the center of the track
- `Ki` is the integral parameter. It multiplies the cumulative cross-track error. This is because an increasing cumulated CTE indicates systemic bias in the controller (e.g. constant error).
- `Kd` is the derivative parameter. It multiplies the difference between two successive values of CTE. The idea is to dampen any large variations to avoid overshooting the center of the "road" when correcting for CTE.

It is interesting to see the effects of running the steering controller with some parameters set to zero.

### P-controller (Ki and Kd set to zero):
With Kd and Ki set to zero, the only component to the controller is positional. As can be seen in the clip below, each time the car deviates from center, a steering correction is applied, but the car then overshoots the middle of the road and an opposite correction needs to be applied. Over time, this effect tends to amplify and only the speed controller keeps the car on the road by braking before it runs out. This is only true at low speeds however. Above 40-45mph, thinks happen too fast for either controllers to be able to stop the car from going cross-country.

<iframe src="https://player.vimeo.com/video/250462701" width="640" height="480" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

The higher the Kd parameter, the sharper the steering response. This improves reactiveness to bends and corners but also increases the amplitude of oscillations and might make car leave the car prematurely.

### PD-controller (Ki set to zero):
By adding the Kd parameter, we introduce a dampening effect on the oscillations. As a result, the vehicle's trajectory is much smoother and the controller is able to keep the car centered for most of the lap (see video clip below).

<iframe src="https://player.vimeo.com/video/250460247" width="640" height="480" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

This clip is recorded with a target speed of 30. At that speed, `Kd = 2.0` works well. With higher speeds, higher values of Kd are necessary, as we will see later (see section "Target speed").

A higher value of Kd increases the dampening but reduces responsiveness. This results in a trajectory that seems made up of straight line segments with sharp corrections between them, because the controller lets the car drift more before applying a correction. This reduces the overall speed of the car because sharp corrections cause the speed controller to apply brakes. With a less efficient speed controller, high Kd values might even prevent the car from following the track in sharp turns.

### PID-controller (no parameter set to zero):
The effect of the Ki parameter is harder to see in this example because there is not much room for systemic bias, but in principle it should help the car to stay close to the center of the road by correcting the cumulated positional error.

<iframe src="https://player.vimeo.com/video/250462590" width="640" height="480" frameborder="0" webkitallowfullscreen mozallowfullscreen allowfullscreen></iframe>

However its value must be small compared to the other two parameters because its effect accumulates rapidly as time goes.
Too small a value has no effect, but too high a value will cause the car to over-compensate and quickly leave the track.


The initial tuning was done manually according to the following process:
1. Set some value for `Kp` so that the car manages to follow the track for some time. In practice with the speed controller I designed, the car achieve a lap around the track but oscillates from left to right with a large amplitude that is only curbed by the speed controller's ability to brake the car when it deviates too much from the center or steers hard. That is as long as the target speed is not set at too high a value (up 40-45mph).

