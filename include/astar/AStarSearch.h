#ifndef AStarSearch_H
#define AStarSearch_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include <vector>

using std::vector;

class AStarSearch{
    public:

    class Node{
    public:
        int x;
        int y;
        float h;
        float g;
        static int width;
        static int height;
        bool visited;
        Node* parent;
        vector<Node *> neighbors;

            Node(int _x, int _y) : x(_x), y(_y), visited(false), parent(nullptr), h(0), g(0) {}

            float get_distance(Node* end){
                return sqrt(
                    (x-end->x)*(x-end->x) + 
                    (y-end->y)*(y-end->y) );
            }

    };

    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle n;
    int width;
    int height;
    float free_thresh = 0.2;
    float occupied_thresh = 0.65;
    nav_msgs::OccupancyGrid map;
    vector<Node *> open_nodes; 
    Node* start_node;
    Node* end_node; 

    AStarSearch(const std::string map_name);
    void goal_callback(const geometry_msgs::PoseStamped goal);
    void load_map(const std::string map_name);
    bool const isValid(int x, int y);
    void deallocate_path();
    bool is_equal(Node* N1, Node* N2);
    float computeHValue(Node* N1);
    void addNeighbors(Node* N1);
    bool static compare(Node* N1, Node* N2); 
    Node* getNextNeighbor();
    int to_map_coordinates(double val);
    void set_goal(float x2, float y2);
    geometry_msgs::PoseStamped get_pstamp(Node* current_node);
    vector<geometry_msgs::PoseStamped> backpropagate(AStarSearch::Node* end_node);
    nav_msgs::Path make_path(vector<geometry_msgs::PoseStamped> &V);
    nav_msgs::Path search_path();
};

#endif