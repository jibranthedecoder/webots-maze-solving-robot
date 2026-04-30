# Webots Maze Solving Robot

A Webots robotics case study where a maze robot uses three distance sensors, heading information, PID wall following, encoder-based turns, and a finite-state machine to navigate through a labyrinth.

## Portfolio case study

Portfolio page will be added here after the project is connected to the main portfolio:

- Portfolio page: `https://www.jibranhussain.com/projects/maze-solving-robot/`
- Finnish README: [`README.fi.md`](README.fi.md)

## Project summary

The task was to write a controller that allows a Thymio-style robot to travel through a maze. The robot has distance sensing on the left, front, and right sides, plus heading information for orientation awareness.

The implemented controller combines wall following and state-machine navigation. The robot drives forward while correcting its distance to a wall, detects intersections and blocked paths from distance sensor readings, and performs calibrated turns using wheel encoder feedback.

## System

- Simulation environment: Webots
- Robot type: Thymio-style maze robot
- Controller language: Python
- Sensors: left, front, and right distance sensors
- Heading input: compass / inertial heading device
- Actuators: left and right wheel motors
- Control methods: PID wall following, encoder-based turning, state-machine navigation

## Engineering problem

A maze-solving robot must make local decisions from limited sensor data. It needs to keep a stable distance from the walls, detect when the front path is blocked, choose a turning direction, handle dead ends, and stop when the maze exit/end condition is reached.

## Control idea

The controller uses a finite-state machine:

```text
move_fwd_max_vel → turn_left / turn_right → dead_end → end
```

While moving forward, the robot uses a PID correction term to keep a suitable wall distance:

```text
u = Kp * error + integral - Kd * measurement_rate
```

Turns are performed using wheel encoder feedback. The controller estimates the turned angle from the left and right wheel travel:

```text
theta = (right_distance - left_distance) / wheel_base
```

## Evidence files

| Evidence | File |
|---|---|
| Python controller | [`maze_solver.py`](maze_solver.py) |
| Original controller upload | [`my_controller.py`](my_controller.py) |
| Simulation video | `maze_simulation.mp4` |

The simulation video should be uploaded as `maze_simulation.mp4` if it is not already present.

## What this project demonstrates

- Distance-sensor based navigation
- PID wall following
- Encoder-based turn control
- Finite-state machine design
- Maze decision logic
- Webots robot simulation
- Python control programming

## Implementation note

The controller reads heading information using `getRollPitchYaw()` from the device named `compass`. This works when the Webots world exposes that device as an inertial heading unit. If the world uses a pure Webots `Compass` device instead, the heading readout should be adapted to `getValues()`.

## License note

The source code is licensed under the MIT License. Report and media files are included as portfolio evidence and educational demonstration material.
