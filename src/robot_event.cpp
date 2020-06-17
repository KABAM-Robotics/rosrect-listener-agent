#include <rosrect-listener-agent/robot_event.h>

using namespace web::json;                  // JSON features
using namespace web;                        // Common features like URIs.

RobotEvent::RobotEvent(){

    this->event_id_str = "";
}

void RobotEvent::update_log(const rosgraph_msgs::Log::ConstPtr& data, json::value msg_info){
    // std::cout << "Event log updating..." << std::endl;
    // Each message has a queue id
    this->queue_id += 1;

    // Get current time
    boost::posix_time::ptime utcTime = boost::posix_time::microsec_clock::universal_time();
    std::string time_str = to_iso_extended_string(utcTime);

    // Event log order
    // 'QID', 'Date/Time', 'Level', 'Compounding',
    // 'Module', 'Source', 'Message', 'Description',
    // 'Resolution', 'RobotEvent_ID'
    int level = 8;
    std::string cflag = "Null";
    std::string module = "Null";
    std::string source = "Null";
    std::string message = "Null";
    std::string description = "Null";
    std::string resolution = "Null"; 

    if(msg_info.is_null()){
        // std::cout << "Populating from ROS!" << std::endl;
        // This is the direct ROS feed case
        // Assign message
        message = data->msg;
        // Assign source
        source = data->name;

        // Assign level
        level = data->level;
    }
    else{
        // std::cout << "Populating from DB!" << std::endl;
        // This is the DB case
        // Get all the data from the JSON object
        level = (msg_info.at(U("error_level"))).as_integer();
        bool cflag_bool = (msg_info.at(U("compounding_flag"))).as_bool();
        if (cflag_bool){
            cflag = "true";
        }
        else{
            cflag = "false";
        }
        module = (msg_info.at(U("error_module"))).as_string();
        source = (msg_info.at(U("error_source"))).as_string();
        message = data->msg;
        description = (msg_info.at(U("error_description"))).as_string();
        resolution = (msg_info.at(U("error_resolution"))).as_string();
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
    event_details.push_back(this->event_id_str);
    
    // Push to log
    this->event_log.push_back(event_details);
    // std::cout << "Event log updated!" << std::endl;
}

void RobotEvent::update_event_id(){
    // Update event_id if necessary
    
    if (!(this->event_id_str.compare(""))){
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

std::vector<std::vector<std::string>> RobotEvent::get_log(){
    // Returns event log

    return this->event_log;
}

void RobotEvent::clear_log(){
    // Clears event log

    this->event_log.clear();
}

void RobotEvent::clear(){
    // Clear log and event_id

    // Clear log
    this->clear_log();

    // Clear event_id
    this->event_id_str="";
}