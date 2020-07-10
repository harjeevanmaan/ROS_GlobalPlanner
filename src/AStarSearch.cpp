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

    AStarSearch::AStarSearch(const std::string map_name) : start_node(nullptr), end_node(nullptr) {
    pub = n.advertise<nav_msgs::Path>("path", 1000);
    sub = n.subscribe("move_base_simple/goal", 1000, &AStarSearch::goal_callback, this);
    ros::Rate loop_rate(2);
    load_map(map_name);
    }

    void AStarSearch::goal_callback(const geometry_msgs::PoseStamped goal){
        if(start_node == nullptr){
            start_node = new Node(to_map_coordinates(goal.pose.position.x), to_map_coordinates(goal.pose.position.y));
            // set_goal(to_map_coordinates(goal.pose.position.x), to_map_coordinates(goal.pose.position.y), 0 ,0);
        }
        else{
            if(end_node != nullptr){
                delete start_node;
                start_node = end_node;
            }
            set_goal(to_map_coordinates(goal.pose.position.x), to_map_coordinates(goal.pose.position.y));
            start_node->h = computeHValue(start_node);
            auto path = search_path();
            deallocate_path();
            pub.publish(path);
        }
    }

    void AStarSearch::load_map(const std::string map_name){
        ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>(map_name);
        nav_msgs::GetMap srv;
        client.call(srv);
        map = srv.response.map;
        width = map.info.width;
        height = map.info.height;
    }

    bool const AStarSearch::isValid(int x, int y){
        if(x <= width && y <= height && (map.data[y*width + x] < free_thresh) && (static_cast<int16_t>(map.data[y*width + x]) >=-0.1)) return true;
        return false;
    }

    void AStarSearch::deallocate_path(){
        for(auto i: open_nodes) delete i;
        open_nodes.clear();
    }

    bool AStarSearch::is_equal(Node* N1, Node* N2){
        if((N1->x == N2->x) && (N1->y == N2->y)) return true;
        return false;
    }

    float AStarSearch::computeHValue(Node* N1){
        return N1->get_distance(end_node);
    }

    void AStarSearch::addNeighbors(Node* N1){
        for(int _x = N1->x-1; _x <= N1->x+1; _x++){
            for(int _y = N1->y-1; _y <= N1->y+1; _y++){
                if(isValid(_x, _y)) {
                    auto node = new Node(_x, _y);
                    node->parent = N1;
                    node->g = N1->g + 0.5;
                    node->h = computeHValue(node);
                    open_nodes.push_back(node);
                }
            }
        }
    }

    bool AStarSearch::compare(Node* N1, Node* N2){
        float f1 = N1->h + N1->g;
        float f2 = N2->h + N2->g;
        return f1>f2;
    }            

    AStarSearch::Node* AStarSearch::getNextNeighbor(){
        std::sort(open_nodes.begin(), open_nodes.end(), compare);
        // std::cout << open_nodes.size() << std::endl;
        if(open_nodes.size() > 0 && open_nodes.size()<10000){
            auto next = open_nodes.back();
            open_nodes.pop_back();
            return next;
        }
        else throw std::runtime_error("No valid path\n");
    }

int AStarSearch::to_map_coordinates(double val){
    return round((val  - map.info.origin.position.x)/map.info.resolution);
}

void AStarSearch::set_goal(float x2, float y2){
    this->end_node = new Node(x2, y2);
}


geometry_msgs::PoseStamped AStarSearch::get_pstamp(Node* current_node){
    geometry_msgs::PoseStamped pstamp;

    std_msgs::Header h;
    h.seq = 0;
    h.stamp = ros::Time::now();
    h.frame_id = "map";

    geometry_msgs::Point p;
    p.x = current_node->x * map.info.resolution + map.info.origin.position.x;
    p.y = current_node->y * map.info.resolution + map.info.origin.position.y;
    p.z = 0;
    
    geometry_msgs::Quaternion q;
    q.x = 0; q.y=0; q.z=0; q.w=1;

    pstamp.header = h;
    pstamp.pose.position = p;
    pstamp.pose.orientation = q;

    return pstamp;
}

vector<geometry_msgs::PoseStamped> AStarSearch::backpropagate(AStarSearch::Node* end_node){//, ros::Publisher &pub

    vector<geometry_msgs::PoseStamped> path;
    auto current_node = end_node;

    while(!is_equal(current_node, start_node)){
        auto pstamp = get_pstamp(current_node);
        path.push_back(pstamp);
        current_node = current_node->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

nav_msgs::Path AStarSearch::make_path(vector<geometry_msgs::PoseStamped> &V){
    nav_msgs::Path path;
    path.header = V[0].header;
    path.poses = V;
    for(auto &i: V) std::cout << i.pose << std::endl;
    return path;
}

nav_msgs::Path AStarSearch::search_path(){
    auto current_node = this->start_node;
    current_node->visited = true;
    while(current_node != nullptr){
        std::cout << "processing\t" << current_node->x << " " << current_node->y << " " << current_node->g << " " << current_node->h << "\n";
        addNeighbors(current_node);
        current_node = getNextNeighbor();
        if(is_equal(current_node, end_node)){
            std::cout << "End Node Reached\n";
            auto path_v = backpropagate(current_node);
            auto path = make_path(path_v);

            return path;
        }
    }
}
