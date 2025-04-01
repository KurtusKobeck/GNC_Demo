The goal of this simulation tool is to produce an algorithm for a fixed thruster 6 Degree of Freedom craft to rely on orientation mechanisms to navigate toward targets by IMU/accelerometer and gyroscope readings alone.
A prime example would be a fixed thruster rocket using internal fly wheels for orientation control.

I used Euler's Angle: General 3D Rotation Matrix to convert the "on board" IMU readings to the absolute cartesian, i.e. "observed from the ground" acceleration values to allow the craft to approximate it's current position 
using the internal clock and IMU values.

The guidance given thrust, change of orientation and top speed limitations algorithm is forthcoming. (WIP)
