#define _CRT_SECURE_NO_WARNINGS
#include <cpprest/json.h>
#undef U
#include <string>
#include <vector>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>         // streaming operators etc.
#include <boost/date_time.hpp>
#include <rosgraph_msgs/Log.h>
#include <locale>
#include <codecvt>

class RobotEvent
{

    // This class provides access to Robot Event to manage event logs for the Agent.

    int queue_id;                                    // Each message has a queue ID
    std::vector<std::vector<std::string>> event_log; // Event log stored as a vector of vector of strings
    std::string event_id_str;                        // UUID that identifies an event

public:
    RobotEvent();
    void update_log(const rosgraph_msgs::Log::ConstPtr &, web::json::value, web::json::value, std::string); // Append to event log
    void update_event_id();                                                                                 // Update event id, create and udpate if necessary
    std::vector<std::vector<std::string>> get_log();                                                        // Return event log
    void clear_log();                                                                                       // Clear only event log
    void clear();                                                                                           // Clearing all events
};