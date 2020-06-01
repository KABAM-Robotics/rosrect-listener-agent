#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <iostream>
#include <rosrect-listener-agent/state_manager.h>

class cs_listener{

    std::string agent_type; // DB or ROS agent
    std::string robot_code; // UUID supplied during setup
    StateManager state_manager_instance; // State manager object that processes all incoming messages

    public:
    cs_listener(); // Constructor to set up listener object
    ~cs_listener(); // Destructor
    void log_callback(const rosgraph_msgs::Log::ConstPtr&); // Listener callback that hands over the rosout message to state manager for processing

};