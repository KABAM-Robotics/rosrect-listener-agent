#include <error_resolution_diagnoser/state_manager.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

StateManager::StateManager()
{

    // Boolean flag to decide whether to suppress a message or not
    this->suppress_flag = false;

    // Timeout parameter in minutes for alert timeout
    this->alert_timeout_limit = 5.0;
}

std::vector<std::string> StateManager::does_exist(std::string robot_code, std::string msg_text)
{

    // Find if msg is already recorded for the given robot code
    std::vector<std::vector<std::string>>::const_iterator row;

    for (row = this->msg_data.begin(); row != this->msg_data.end(); row++)
    {
        if ((find(row->begin(), row->end(), msg_text) != row->end()) &&
            (find(row->begin(), row->end(), robot_code) != row->end()))
            return *(row);
    }

    std::vector<std::string> emptyString;
    emptyString.push_back("");

    return emptyString;
}

void StateManager::check_message(std::string agent_type, std::string robot_code, const rosgraph_msgs::Log::ConstPtr &data, json::value telemetry)
{

    if (agent_type == "ECS")
    {
        // std::cout << "Checking with ECS..." << std::endl;
        this->check_message_ecs(robot_code, data, telemetry);
    }
    else if ((agent_type == "ERT") || (agent_type == "DB"))
    {
        // std::cout << "Checking with ERT..." << std::endl;
        this->check_message_ert(robot_code, data, telemetry);
    }
    else
    {
        // std::cout << "Checking with ROS..." << std::endl;
        this->check_message_ros(robot_code, data, telemetry);
    }
}

void StateManager::check_message_ecs(std::string robot_code, const rosgraph_msgs::Log::ConstPtr &data, json::value telemetry)
{

    // Parse message to query-able format
    std::string msg_text = data->msg;
    // std::replace(msg_text.begin(), msg_text.end(), '/', ' ');
    // std::cout << "Querying: " << msg_text << std::endl;

    // Check error classification, ECS
    json::value msg_info = this->api_instance.check_error_classification(msg_text);

    bool ecs_hit = !(msg_info.is_null());
    // std::cout << "ECS Hit: " << ecs_hit << std::endl;

    if (ecs_hit)
    {
        // ECS has a hit, follow the message cycle
        // std::cout << "JSON parsed";
        // msg_info = msg_info[0];

        int error_level = (msg_info.at(utility::conversions::to_string_t("severity"))).as_integer();
        // std::cout << "Level: " << error_level << std::endl;
        std::string error_msg = (msg_info.at(utility::conversions::to_string_t("error_text"))).as_string();
        // std::cout << "Text: " << error_msg << std::endl;

        if ((error_level == 8) || (error_level == 16))
        {
            // std::cout << "Error... " << data->msg << std::endl;
            // Check for suppression
            this->check_error(robot_code, error_msg);
        }
        else if (error_level == 4)
        {
            // std::cout << "Warning... " << data->msg << std::endl;
            // Check for suppression
            this->check_warning(robot_code, error_msg);
        }
        else
        {
            // std::cout << "Info... " << data->msg << std::endl;
            // Check for suppression
            this->check_info(robot_code, error_msg);
        }

        // Process result of event
        if (this->suppress_flag)
        {
            // If suppressed, do nothing
            // std::cout << "Suppressed!" << std::endl;
        }
        else
        {
            // std::cout << "Not suppressed!" << std::endl;
            // If not suppressed, send it to event to update
            this->event_instance.update_log(data, msg_info, telemetry, "ECS");

            // Push to stream
            this->api_instance.push_event_log(this->event_instance.get_log());

            // Get compounding flag
            bool cflag = (msg_info.at(utility::conversions::to_string_t("compounding_flag"))).as_bool();

            if (cflag == true)
            {
                // Nothing to do here unless it is a compounding error
                if ((error_level == 8) || (error_level == 16))
                {
                    // Push on ALL errors / One named Info msg
                    // Clear only event log since this is compounding
                    this->event_instance.clear_log();
                }
                else
                {
                    // Nothing to do
                }
            }
            else
            {
                // This is a compounding log, Clear everything
                this->clear();
            }
        }
    }
    else
    {
        // ECS does not have a hit, normal operation resumes
    }
}

void StateManager::check_message_ert(std::string robot_code, const rosgraph_msgs::Log::ConstPtr &data, json::value telemetry)
{

    // Parse message to query-able format
    std::string msg_text = data->msg;
    // std::replace(msg_text.begin(), msg_text.end(), '/', ' ');
    // std::cout << "Querying: " << msg_text << std::endl;

    // Check error classification, ECS
    json::value msg_info = this->api_instance.check_error_classification(msg_text);

    bool ecs_hit = !(msg_info.is_null());
    // std::cout << "ECS Hit: " << ecs_hit << std::endl;

    if (ecs_hit)
    {
        // ECS has a hit, follow the message cycle
        // std::cout << "JSON parsed";
        // msg_info = msg_info[0];

        int error_level = (msg_info.at(utility::conversions::to_string_t("error_level"))).as_integer();
        // std::cout << "Level: " << error_level << std::endl;
        std::string error_msg = (msg_info.at(utility::conversions::to_string_t("error_text"))).as_string();
        // std::cout << "Text: " << error_msg << std::endl;

        if (error_level == 8)
        {
            // std::cout << "Error... " << data->msg << std::endl;
            // Check for suppression
            this->check_error(robot_code, error_msg);
        }
        else if (error_level == 4)
        {
            // std::cout << "Warning... " << data->msg << std::endl;
            // Check for suppression
            this->check_warning(robot_code, error_msg);
        }
        else
        {
            // std::cout << "Info... " << data->msg << std::endl;
            // Check for suppression
            this->check_info(robot_code, error_msg);
        }

        // Process result of event
        if (this->suppress_flag)
        {
            // If suppressed, do nothing
            // std::cout << "Suppressed!" << std::endl;
        }
        else
        {
            // std::cout << "Not suppressed!" << std::endl;
            // If not suppressed, send it to event to update
            this->event_instance.update_log(data, msg_info, telemetry, "ERT");

            // Push to stream
            this->api_instance.push_event_log(this->event_instance.get_log());

            // Get compounding flag
            bool cflag = (msg_info.at(utility::conversions::to_string_t("compounding_flag"))).as_bool();

            if (cflag == true)
            {
                // Nothing to do here unless it is a compounding error
                if (error_level == 8)
                {
                    // Push on ALL errors / One named Info msg
                    // Clear only event log since this is compounding
                    this->event_instance.clear_log();
                }
                else
                {
                    // Nothing to do
                }
            }
            else
            {
                // This is a compounding log, Clear everything
                this->clear();
            }
        }
    }
    else
    {
        // ECS does not have a hit, normal operation resumes
    }
}

void StateManager::check_message_ros(std::string robot_code, const rosgraph_msgs::Log::ConstPtr &data, json::value telemetry)
{

    if (data->level == 8)
    {
        // std::cout << "Error... " << data->msg << std::endl;
        // Check for suppression
        this->check_error(robot_code, data->msg);
    }
    else if (data->level == 4)
    {
        // std::cout << "Warning... " << data->msg << std::endl;
        // Check for suppression
        this->check_warning(robot_code, data->msg);
    }
    else
    {
        // std::cout << "Info... " << data->msg << std::endl;
        // Check for suppression
        this->check_info(robot_code, data->msg);
    }

    // Process result of event
    if (this->suppress_flag)
    {
        // If suppressed, do nothing
        // std::cout << "Suppressed!" << std::endl;
    }
    else
    {
        // std::cout << "Not suppressed!" << std::endl;
        // If not suppressed, send it to event to update
        this->event_instance.update_log(data, json::value::null(), telemetry, "ROS");

        // Push log
        this->api_instance.push_event_log(this->event_instance.get_log());

        if ((data->level == 8) || (data->msg == "Goal reached"))
        {
            // Clear everything, end of event
            this->clear();
        }
        else
        {
            // Clear only log
            this->event_instance.clear_log();
        }
    }
}

void StateManager::check_error(std::string robot_code, std::string msg_text)
{

    std::vector<std::string> found = this->does_exist(robot_code, msg_text);
    bool exist;

    if (found[0] == "")
    {
        exist = false;
    }
    else
    {
        exist = true;
    }

    if (exist)
    {
        // Found, suppress
        this->suppress_flag = true;
    }
    else
    {
        // Not found, add to data

        std::vector<std::string> msg_details;

        // Get current time
        time_t now;
        time(&now);
        char buf[sizeof "2011-10-08T07:07:09Z"];
        strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
        std::string time_str = std::string(buf);

        // Push details to data
        msg_details.push_back(robot_code);
        msg_details.push_back(msg_text);
        msg_details.push_back(time_str);
        this->msg_data.push_back(msg_details);

        // Do not suppress
        this->suppress_flag = false;
    }
}

void StateManager::check_warning(std::string robot_code, std::string msg_text)
{

    std::vector<std::string> found = this->does_exist(robot_code, msg_text);
    bool exist;

    if (found[0] == "")
    {
        exist = false;
    }
    else
    {
        exist = true;
    }

    if (exist)
    {
        // Found, check timeout limit - not implemented yet
        this->suppress_flag = true;
    }
    else
    {
        // Not found, add to data

        std::vector<std::string> msg_details;

        // Get current time
        time_t now;
        time(&now);
        char buf[sizeof "2011-10-08T07:07:09Z"];
        strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
        std::string time_str = std::string(buf);

        // Push details to data
        msg_details.push_back(robot_code);
        msg_details.push_back(msg_text);
        msg_details.push_back(time_str);
        this->msg_data.push_back(msg_details);

        // Do not suppress
        this->suppress_flag = false;
    }
}

void StateManager::check_info(std::string robot_code, std::string msg_text)
{

    std::vector<std::string> found = this->does_exist(robot_code, msg_text);
    bool exist;

    if (found[0] == "")
    {
        exist = false;
        // std::cout << "Msg found status: False" << std::endl;
    }
    else
    {
        exist = true;
        // std::cout << "Msg found status: True" << std::endl;
    }

    if (exist)
    {
        // Found, suppress
        this->suppress_flag = true;
    }
    else
    {
        // Not found, add to data

        std::vector<std::string> msg_details;

        // Get current time
        time_t now;
        time(&now);
        char buf[sizeof "2011-10-08T07:07:09Z"];
        strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
        std::string time_str = std::string(buf);

        // Push details to data
        msg_details.push_back(robot_code);
        msg_details.push_back(msg_text);
        msg_details.push_back(time_str);
        this->msg_data.push_back(msg_details);

        // Do not suppress
        this->suppress_flag = false;
    }
}

void StateManager::check_heartbeat(bool status, json::value telemetry)
{
    // Pass data to backend to push appropriate status
    this->api_instance.push_status(status, telemetry);
}

void StateManager::clear()
{
    // Clears the state manager data for a new session
    this->suppress_flag = false;
    this->msg_data.clear();
    this->event_instance.clear();
}