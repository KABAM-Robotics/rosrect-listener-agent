#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <iostream>
#include <vector>
#include <error_resolution_diagnoser/state_manager.h>

class cs_listener
{
    // This class provides the ROS node interface for the agent.

    std::string agent_type;                // DB or ROS agent
    std::string robot_code;                // UUID supplied during setup
    ros::WallTimer heartbeat_timer;        // ROS timer that is configured to with heartbeat_start as callback
    ros::WallDuration heartrate;           // Period duration for heartbeat_timer
    StateManager state_manager_instance;   // State manager object that processes all incoming messages
    ros::Subscriber odom_sub;              // Subscriber for Odometry telemetry info
    ros::Subscriber pose_sub;              // Subscriber for Pose telemetry info
    ros::Subscriber diag_sub;              // Subscriber for Diagnostics info
    web::json::value telemetry;            // JSON value that will be pushed as a part of event/status
    bool telemetry_ok;                     // Used to check if telemetry subs have been setup
    int num_diag_samples;                  // Down sample factor for diagnostics
    int curr_diag_sample;                  // Keeps track of sample index received
    std::vector<std::string> node_list;    // List of nodes to include messages by
    std::vector<std::string> node_ex_list; // List of nodes to exclude messages by
    std::string diag_setting;              // Keeps track of the diagnostics setting on or off

public:
    cs_listener();                                                                             // Constructor to set up listener object
    ~cs_listener();                                                                            // Destructor
    void setup_telemetry(ros::NodeHandle);                                                     // Sets up additional subscribers dynamically to populate telemetry
    void setup_diagnostics(ros::NodeHandle);                                                   // Sets up additional diagnostics subscriber
    void log_callback(const rosgraph_msgs::Log::ConstPtr &);                                   // Listener callback that hands over the rosout message to state manager for processing
    void odom_callback(const nav_msgs::Odometry::ConstPtr &);                                  // Odometry callback for telemetry info
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);            // Pose callback for telemetry info
    void diag_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr &);                    // DiagnosticArray callback for diagnostics info
    void heartbeat_start(ros::NodeHandle);                                                     // Utility method to setup the heartbeat_timer
    void heartbeat_log(const ros::WallTimerEvent &);                                           // Timer callback that logs heartbeat online
    void heartbeat_stop();                                                                     // Method that is called when node is shut down to log heartbeat offline
    web::json::value odom_to_json(const nav_msgs::Odometry::ConstPtr &);                       // Utility function to convert Odometry message to JSON
    web::json::value pose_to_json(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &); // Utility function to convert Pose message to JSON
};