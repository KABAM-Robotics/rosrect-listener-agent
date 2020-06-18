#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <signal.h>
#include <iostream>
#include <rosrect-listener-agent/state_manager.h>

class cs_listener
{

    std::string agent_type;              // DB or ROS agent
    std::string robot_code;              // UUID supplied during setup
    ros::WallTimer heartbeat_timer;      // ROS timer that is configured to with heartbeat_start as callback
    ros::WallDuration heartrate;         // Period duration for heartbeat_timer
    StateManager state_manager_instance; // State manager object that processes all incoming messages

public:
    cs_listener();                                           // Constructor to set up listener object
    ~cs_listener();                                          // Destructor
    void log_callback(const rosgraph_msgs::Log::ConstPtr &); // Listener callback that hands over the rosout message to state manager for processing
    void heartbeat_start(ros::NodeHandle);                   // Utility method to setup the heartbeat_timer
    void heartbeat_log(const ros::WallTimerEvent &);             // Timer callback that logs heartbeat online
    void heartbeat_stop();                                   // Method that is called when node is shut down to log heartbeat offline
};