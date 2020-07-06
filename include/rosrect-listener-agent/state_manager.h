#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <ctime>
#include <rosgraph_msgs/Log.h>
#include <cpprest/json.h>
#include <rosrect-listener-agent/backend_api.h>
#include <rosrect-listener-agent/robot_event.h>

class StateManager
{

    // This class provides access to State Manager which manages the current state/suppression of the agent.

    bool suppress_flag;                             // Boolean flag to decide whether to suppress a message or not
    float alert_timeout_limit;                      // Timeout parameter in minutes for alert timeout
    std::vector<std::vector<std::string>> msg_data; // Accumulated message for current event stored as vector of vector of strings
    BackendApi api_instance;                        // Back end API instance
    RobotEvent event_instance;                      // Robot event instance

public:
    StateManager();
    // ~StateManager();
    std::vector<std::string> does_exist(std::string, std::string);                                        // Check if message already logged with this robot
    void check_message(std::string, std::string, const rosgraph_msgs::Log::ConstPtr &, web::json::value); // Entry point to state management that calls the correct variant of check_message*
    void check_message_ecs(std::string, const rosgraph_msgs::Log::ConstPtr &, web::json::value);          // State management in case of ECS feedback
    void check_message_ert(std::string, const rosgraph_msgs::Log::ConstPtr &, web::json::value);          // State management in case of ERT feedback
    void check_message_ros(std::string, const rosgraph_msgs::Log::ConstPtr &, web::json::value);          // State management in case of a ROS direct feed
    void check_error(std::string, std::string);                                                           // Check error suppression
    void check_warning(std::string, std::string);                                                         // Check warning suppression
    void check_info(std::string, std::string);                                                            // Check info suppression
    void check_heartbeat(bool, web::json::value);                                                         // Performs heartbeat check and pushes appropriate data
    void clear();                                                                                         // Clearing all states
};