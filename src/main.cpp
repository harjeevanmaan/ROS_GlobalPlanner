#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <stdexcept>
#include <AStarSearch.h>
using std::vector;



int main(int argc, char** argv){// destroy vector each time
    ros::init(argc, argv, "nav_demo");
    auto route = AStarSearch("/static_map");
    ros::Rate loop_rate(2);



    int x1, y1, x2, y2;    
    while(ros::ok()){
        ros::spinOnce();
        // std::cout << "Enter x1, y1, x2, y2\n";
        // std::cin >> x1 >> y1 >> x2 >> y2;
        // route.pub_path(path_pub, x1, y1, x2, y2);

        loop_rate.sleep();
    }
    return 0;
}