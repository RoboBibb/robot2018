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
As a result of not having encoders on this robot, autonomous driving code is somewhat of a challenge to implement, despite this we have created a drive straight function wich works and a turning function which is accurate to within 1/5th of a degree. Both of these functions return a value which can be read to determine if there was an error, and have debugging info in case they don't work perfectly. We have done quite a lot with a gyro as our only sensor.
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
All of the functions in  `utils.hpp` which control robot driving return values to tell us if they completed successfully
All of our autonomous modes have a "do nothing" failsafe, so if there are any field errors or the driver forgets to change the auto the robot won't move. 
- **left hook:** drives forward, turns 90 degrees right, drives forward, dumps cube into switch if correct color
- **right hook:** drives forward, turns 90 degrees left, drives forward, dumps cube into switch if correct color
- **right side:** drives forward, dumps cube into switch if correct color
- **drive straight; do nothing:** self explanitory, only for use when other teams need space
- **center:** drives forward, gets field data, turns to correct side of switch, drives forward, turns toward switch, dumps

# Controls
Because this robot has lots of things to control, we need two drivers. As usual we work closely with drive team in order to make the controls as natural as possible. These are what they have given us.
