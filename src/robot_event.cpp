#include <error_resolution_diagnoser/robot_event.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

RobotEvent::RobotEvent()
{

    this->event_id_str = "";
}

void RobotEvent::update_log(const rosgraph_msgs::Log::ConstPtr &data, json::value msg_info, json::value telemetry, std::string agent_type)
{
    // std::cout << "Event log updating..." << std::endl;
    // Each message has a queue id
    this->queue_id += 1;

    // Get current time
    boost::posix_time::ptime utcTime = boost::posix_time::microsec_clock::universal_time();
    std::string time_str = to_iso_extended_string(utcTime);

    // Event log order
    // 'QID', 'Date/Time', 'Level', 'Compounding',
    // 'Module', 'Source', 'Message', 'Description',
    // 'Resolution', 'RobotEvent_ID', 'Telemetry'
    int level = 8;
    std::string cflag = "Null";
    std::string module = "Null";
    std::string source = "Null";
    std::string message = "Null";
    std::string description = "Null";
    std::string resolution = "Null";

    utility::stringstream_t stream;
    telemetry.serialize(stream);
    // std::cout << "Event telemetry: " << stream.str() << std::endl;
    std::string telemetry_str = stream.str();

    if (agent_type == "ECS")
    {
        // std::cout << "Populating from ECS!" << std::endl;
        // This is the ECS case
        // Get all the data from the JSON object
        level = (msg_info.at(utility::conversions::to_string_t("severity"))).as_integer();
        bool cflag_bool = (msg_info.at(utility::conversions::to_string_t("compounding_flag"))).as_bool();
        if (cflag_bool)
        {
            cflag = "true";
        }
        else
        {
            cflag = "false";
        }
        module = (msg_info.at(utility::conversions::to_string_t("error_module"))).as_string();
        source = (msg_info.at(utility::conversions::to_string_t("error_source"))).as_string();
        message = data->msg;
        // Setting description to stored error_text. Needs to be set appropriately later
        description = (msg_info.at(utility::conversions::to_string_t("error_text"))).as_string();
        // Resolution needs to be set appropriately later.
        // resolution = (msg_info.at(utility::conversions::to_string_t("error_resolution"))).as_string();
    }
    else if ((agent_type == "ERT") || (agent_type == "DB"))
    {
        // std::cout << "Populating from ERT!" << std::endl;
        // This is the ERT case
        // Get all the data from the JSON object
        level = (msg_info.at(utility::conversions::to_string_t("error_level"))).as_integer();
        bool cflag_bool = (msg_info.at(utility::conversions::to_string_t("compounding_flag"))).as_bool();
        if (cflag_bool)
        {
            cflag = "true";
        }
        else
        {
            cflag = "false";
        }
        module = (msg_info.at(utility::conversions::to_string_t("error_module"))).as_string();
        source = (msg_info.at(utility::conversions::to_string_t("error_source"))).as_string();
        message = data->msg;
        description = (msg_info.at(utility::conversions::to_string_t("error_description"))).as_string();
        resolution = (msg_info.at(utility::conversions::to_string_t("error_resolution"))).as_string();
    }
    else
    {
        // std::cout << "Populating from ROS!" << std::endl;
        // This is the direct ROS feed case
        // Assign message
        message = data->msg;
        // Assign source
        source = data->name;

        // Assign level
        level = data->level;
    }

    // Update event id
    this->update_event_id();

    // Construct record
    std::vector<std::string> event_details;
    event_details.push_back(time_str);
    event_details.push_back(std::to_string(level));
    event_details.push_back(cflag);
    event_details.push_back(module);
    event_details.push_back(source);
    event_details.push_back(message);
    event_details.push_back(description);
    event_details.push_back(resolution);
    event_details.push_back(telemetry_str);
    event_details.push_back(this->event_id_str);

    // Push to log
    this->event_log.push_back(event_details);
    // std::cout << "Event log updated!" << std::endl;
}

void RobotEvent::update_event_id()
{
    // Update event_id if necessary

    if (!(this->event_id_str.compare("")))
    {
        // Generate UUID
        boost::uuids::uuid uuid = boost::uuids::random_generator()();

        // Stream
        std::stringstream uuid_str;
        uuid_str << uuid;

        // Assign
        this->event_id_str = uuid_str.str();

        // Clear stream
        uuid_str.clear();
    }
}

std::vector<std::vector<std::string>> RobotEvent::get_log()
{
    // Returns event log

    return this->event_log;
}

void RobotEvent::clear_log()
{
    // Clears event log

    this->event_log.clear();
}

void RobotEvent::clear()
{
    // Clear log and event_id

    // Clear log
    this->clear_log();

    // Clear event_id
    this->event_id_str = "";
}