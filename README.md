# Robot2018
Code for our 2018 robot. This year we decided to make the code simpler than normal while providing all the functionality we need.

## Hardware
- 6 CIM drive train - 6 x Victor SPX
- Roller Arms:
  - arm elevator: raise/lower arm - Talon
  - arm rollers: intake/shoot cube - 2 x Spark
  - arm grabbing: grab/let go of cube - double solenoid
- Dumper: useful for auto - double solenoid
- Sensors:
  - gyro
  - builtin accelerometer
  - usb camera

# What We Want to Show Off

## Automatic Driving
- `utils::driveStraight(gyro, drivetrain, time, speed)`: Uses gyro to drive straight for a set period of time
- `utils::turnDeg(gyro, drivetrain, angle)`: turns robot set number of degrees, speed decreases linearly upon aproach

## Controller Code
We have been improving our driving code for the past 3 seasons. We find that using these functions improves efficiency during driving and responds to input more naturally.
- We've made our library's (`utils.hpp`) simpler and easier to follow.
- `utils::expReduceBrownout(double, double&)`: improves efficiency by preventing rapid changes in direction/speed, motor controllers still break instantly when sticks are released.
- `utils::linReduceBrownout(double, double, double&)`: limits changes in input
- Because both sides of this years robot have functionality, we added a reverse button
- we added a slowmode button to allow for precise movements in between going fast.

# Autonomous
- **left side:** drives forward, turns 90 degrees right, drives forward, dumps cube into switch if correct color
- **right side:** drives forward, dumps cube into switch if correct color
- **drive straight; do nothing:** self explanitory
- **experiment:** not implemented yet, will need testing, expect this at state, start at middle
   - drives forward, turns toward correct side, drives forward, turns back toward switch, drive forward, dump
     - pros: easy to code since doesn't need to use arms
     - cons: 2 turns, could get unaligned
   - drives forward, turns toward correct side of switch, angles shooter, launches cube into switch

# Controls
Because this robot has lots of things to control, we need two drivers. As usual we work closely with drive team in order to make the controls as natural as possible. These are what they have given us.
