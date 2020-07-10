# ROS_GlobalPlanner

Implemented a path planner from scratch using C++ and ROS. Use the 2D Nav goal to generate a start and end point in Rviz, and an optimal path avoiding any obstacles on the way will be generated and overlaid on the map. 

![](/Images_GlobalPlanner/global_planner.png)
Example of a path being generated after selecting a start and end goal

To use this on any map, simply run the map_server
`rosrun map_server map_server path_to_map_file.yaml`
I provided an example map that I generated using the turtlebot3 under maps/map.yaml

To launch rviz to view the map simply use the command
`rviz`

Both of the above steps are necessary before using this node, which can be launched with 
`rosrun astar astar`

After configuring the fixed frame to "map", one should be able to view the map. After which use the 2D nav goal option to generate navigation targets on the map. The "simulated robot" will generate a path to its new goal with each new nav goal, except the first which is used to decide it's initial position.

