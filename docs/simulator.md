# Unicycle 3-Segment Turn Simulator

## Overview

This program simulates a **3-segment maneuver for a unicycle robot** using the standard unicycle kinematics model.

The robot:

- moves with **constant linear speed magnitude**
- cannot perform **pure rotation in place**
- can only change heading while moving
- is visualized as a **rectangle** rather than a point mass
- reads its geometry and maneuver parameters from a `config.toml` file

The script produces:

- a **static plot** of the full trajectory
- an **animated visualization** showing the robot body moving along that trajectory

---

## Kinematic Model

The robot uses the standard unicycle equations:

\[
\dot{x} = v \cos(\theta)
\]

\[
\dot{y} = v \sin(\theta)
\]

\[
\dot{\theta} = \omega
\]

where:

- `x, y` are the robot position
- `theta` is the heading angle
- `v` is the linear velocity
- `omega` is the angular velocity

Since pure rotation is not allowed, the robot must always move while changing orientation.

---

## Maneuver Structure

The motion is divided into 3 parameterized segments:

1. **Forward + Left** by `alpha` degrees over distance `d1`
2. **Reverse + Right** by `beta` degrees over distance `d2`
3. **Forward + Left** by `gamma` degrees over distance `d3`

Each segment is modeled using:

- a constant linear velocity `v`
- a constant angular velocity `omega`
- a computed duration based on the requested segment distance

This means each segment has a constant curvature.

---

## Configuration File

The program reads parameters from `config.toml`.

### Supported parameters

#### Robot dimensions
- `ROBOT_WIDTH`
- `ROBOT_LENGTH`

These define the rectangular body used in the visualization.

#### Segment angles
- `alpha`
- `beta`
- `gamma`

These are specified in **degrees**.

#### Segment distances
- `d1`
- `d2`
- `d3`

These are path lengths for the 3 segments.

#### Simulation parameters
- `v` — constant speed magnitude
- `dt` — integration timestep

---

## Default Values

If a parameter is missing from `config.toml`, the code uses these defaults:

```toml
ROBOT_WIDTH = 0.8
ROBOT_LENGTH = 1.2

alpha = 90
beta = 1
gamma = 90

d1 = 1.25
d2 = 1.75
d3 = 1.25

v = 1.0
dt = 0.02
```
