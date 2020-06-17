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

    // This class provides access to State Manager for Cognicept's Listener Agent.

    // Boolean flag to decide whether to suppress a message or not
    bool suppress_flag;

    // Timeout parameter in minutes for alert timeout
    float alert_timeout_limit;

    // Dataframes to keep track of message suppression
    // self.msg_data = pd.DataFrame(columns=['robot_code', 'msg_text', 'time'])
    std::vector<std::vector<std::string>> msg_data;

    // Error classification API
    BackendApi api_instance;

    // Event management
    RobotEvent event_instance;

public:
    StateManager();
    // ~StateManager();
    std::vector<std::string> does_exist(std::string, std::string);                      // Check if message already logged with this robot
    void check_message(std::string, std::string, const rosgraph_msgs::Log::ConstPtr &); // Entry point to state management that calls the correct variant of check_message*
    void check_message_ecs(std::string, const rosgraph_msgs::Log::ConstPtr &);          // State management in case of ECS feedback
    void check_message_ert(std::string, const rosgraph_msgs::Log::ConstPtr &);          // State management in case of ERT feedback
    void check_message_ros(std::string, const rosgraph_msgs::Log::ConstPtr &);          // State management in case of a ROS direct feed
    void check_error(std::string, std::string);                                         // Check error suppression
    void check_warning(std::string, std::string);                                       // Check warning suppression
    void check_info(std::string, std::string);                                          // Check info suppression
    void clear();                                                                       // Clearing all states
};