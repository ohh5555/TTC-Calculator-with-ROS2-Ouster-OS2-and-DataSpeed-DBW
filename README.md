This ROS2 program calculates the TTC to the closest object in front.
LiDAR is used to detect the object in front and the horizontal field of view is narrowed to 10 degrees.
The velocity data is received from the Dataspeed's /vehicle/twist topic's X linear velocity subtopic.

The laser topic and velocity topic can be modified to fit your needs. 
