#include <string>
#include <vector>
#include <ctime>
#include <uuid/uuid.h>
#include <cpprest/json.h>
#include <rosgraph_msgs/Log.h>

using namespace web::json;                  // JSON features

class RobotEvent {

    // This class provides access to Robot Event for Cognicept's Listener Agent.

    // Each message has a queue ID
    int queue_id;
    
    // Instead of using a DataFrame like python, opting to maintain several vectors instead
    // Could look into implementing this with other libraries such as C++ DataFrame or SQLite
    std::vector<std::vector<std::string>> event_log;

    // Event ID
    std::string event_id_str;

    public:
    RobotEvent();
    void update_log(const rosgraph_msgs::Log::ConstPtr&, web::json::value); // Append to event log
    void update_event_id(); // Update event id, create and udpate if necessary
    std::vector<std::vector<std::string>> get_log(); // Return event log
    void clear_log(); // Clear only event log
    void clear(); // Clearing all events
};