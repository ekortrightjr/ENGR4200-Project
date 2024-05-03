# ENGR4200-Project
# Autonomous Guided Patrol Vehicle (AGPV)
## Enrique V. Kortright, Jr.
## Gabriel O'Regan

This project entails implementing an autonomous guided patrol vehicle (AGPV) based on the ROS2 robot operating system and the Nav2 navigation framework.

The vehicle will patrol an enclosed environment like a warehouse. The vehicle will visit each station, check and report the station status, and then return to the base at the end of the patrol.

To accomplish this objective, the following tasks will be completed. A physical environment will be built for the robot to navigate. Using GMapping, a SLAM algorithm for map generation, and LiDAR, a map of the environment will be created. The poses of the bases and each station will be determined for use in navigation. The following patrol vehicle behaviors will be implemented: routine patrol navigation, station status detection and reporting, station error status handling, and traffic light detection and obeying. The vehicle will provide continuous feedback as it navigates from station to station and report successful task completion or errors encountered. The operation of the robot will be monitored using RViz, the ROS visualization application.

