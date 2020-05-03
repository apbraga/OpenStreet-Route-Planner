#include "route_planner.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    
    //initialize start and finish node
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    //return the distance from a node to the end node
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    //populate the neighbor list for the current node
    current_node->FindNeighbors();
    //loop through all neighbors from the current node
    for(RouteModel::Node *neighbor : current_node->neighbors){
        //update neighbor attributes
        //set current node as parent of the neighbor node
        neighbor->parent = current_node;
        //set h value with its heuristc distance to the end node
        neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
        //set g value incrementing the g value from parent with the distance from parent to neighbor
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        //add the neighbor to the open list
        open_list.push_back(neighbor);
        //set flag to visited
        neighbor->visited = true;
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

//helper for sort method, compare two nodes and return True if the fisrt node has lower cost, or False in case the second has lower cost
bool Compare(const auto &node1, const auto &node2){
    return ((node1->h_value + node1->g_value) < (node2->h_value + node2->g_value));
}

RouteModel::Node *RoutePlanner::NextNode() {
    //use c++ sort function to update open_list ascendingly with g+f cost
    std::sort(open_list.begin(), open_list.end(), Compare;
    });
    //pop first element (lowest cost) from the open list
    RouteModel::Node *nextnode = open_list.front();
    open_list.erase(open_list.begin());
    //return first item from the list as next node
    return nextnode;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found{};

    // TODO: Implement your solution here.
    
    //loop from end node to start node, stop when parent is not find, or in other words, we reached the start node
    while(current_node->parent != nullptr){
        //add current node to path list
        path_found.push_back(*current_node);
        //increment the distance with the distance from the current node to its path
        distance += current_node->distance(*(current_node->parent));
        //set current node to its parent
        current_node = current_node->parent;
    }
    //add the start node to the path
    path_found.push_back(*current_node);

    //reverse the path vector from end node >> start node to start node >> end node
    std::reverse(path_found.begin(),path_found.end());
    // scale the distance
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    //update the start node flag as visited
    start_node->visited = true;
    //add start node to open_list
    open_list.push_back(start_node);

    //continue search until there are nodes to explore
    while(open_list.size()>0){
        //get node with lowest cost from open list
        current_node = NextNode();
        //check if the node is the goal
        if(current_node->distance(*end_node)== 0){
            // create path from start to end
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // add new nodes to the open list
        AddNeighbors(current_node);
    }
    return;
}