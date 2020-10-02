#include <rosrect-listener-agent/state_manager.h>

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

void StateManager::check_diagnostic(std::string agent_type, std::string robot_code, std::vector<diagnostic_msgs::DiagnosticStatus> current_diag, json::value telemetry)
{
    this->check_diagnostic_ros(robot_code, current_diag, telemetry);
    // if (agent_type == "ECS")
    // {
    //     // std::cout << "Checking with ECS..." << std::endl;
    //     this->check_diagnostic_ecs(robot_code, current_diag, telemetry);
    // }
    // else if ((agent_type == "ERT") || (agent_type == "DB"))
    // {
    //     // std::cout << "Checking with ERT..." << std::endl;
    //     this->check_diagnostic_ert(robot_code, current_diag, telemetry);
    // }
    // else
    // {
    //     // std::cout << "Checking with ROS..." << std::endl;
    //     this->check_diagnostic_ros(robot_code, current_diag, telemetry);
    // }
}

void StateManager::check_diagnostic_ros(std::string robot_code, std::vector<diagnostic_msgs::DiagnosticStatus> current_diag, json::value telemetry)
{
    // Check diagnostic data and if not suppressed, push it to the event

    // Variables to store diagnostic info for state management
    std::string diag_str;
    int diag_level;
    std::string diag_ident;

    for (unsigned int idx = 0; idx < current_diag.size(); idx++)
    {
        // Store diagnostics name+hardware_id in a single string for quick search
        diag_str = current_diag[idx].name + "_" + current_diag[idx].hardware_id;
        // Diagnostics level. Main determination for state suppression
        diag_level = static_cast<int>(current_diag[idx].level);
        // Check if diagnostic needs to be suppressed. All diagnostics with the same str
        // and no change in level are suppressed. We process only when there is change in levels.
        this->check_diag_data(robot_code, diag_str, std::to_string(diag_level));

        if (this->suppress_flag)
        {
            // If suppressed, do nothing
            // std::cout << "Suppressed!" << std::endl;
        }
        else
        {
            std::string diag_name = current_diag[idx].name;
            std::string diag_hwid = current_diag[idx].hardware_id;

            if (!diag_name.empty())
            {
                diag_ident = diag_name;
            }
            else if (!diag_hwid.empty())
            {
                diag_ident = diag_hwid;
            }

            std::cout << "Diagnostic Message State Change! Name: " << diag_str << ", Message: " << current_diag[idx].message << std::endl;
            // If not suppressed, send it to event to update

            // Construct ROS log equivalent of diag
            rosgraph_msgs::Log rosmsg;
            rosmsg.name = diag_str;
            if (!diag_ident.empty())
            {
                rosmsg.msg = diag_ident + "-->" + current_diag[idx].message;
            }
            else
            {
                rosmsg.msg = current_diag[idx].message;
            }

            if (diag_level == 2)
            {
                rosmsg.level = rosmsg.ERROR;
            }
            else if ((diag_level == 1) || (diag_level == 3))
            {
                rosmsg.level = rosmsg.WARN;
            }
            else
            {
                rosmsg.level = rosmsg.INFO;
            }

            rosgraph_msgs::Log::ConstPtr data(new rosgraph_msgs::Log(rosmsg));

            this->event_instance.update_log(data, json::value::null(), telemetry, "ROS");

            // Push log
            this->api_instance.push_event_log(this->event_instance.get_log());

            // if (data->level == 8)
            // {
            //     // Clear everything, end of event
            //     this->clear();
            // }
            // else
            // {
            //     // Clear only log
            //     this->event_instance.clear_log();
            // }
        }
    }
}

void StateManager::check_diagnostic_ert(std::string robot_code, std::vector<diagnostic_msgs::DiagnosticStatus> current_diag, json::value telemetry)
{
    // Check diagnostic data and if not suppressed, push it to the event

    // Variables to store diagnostic info for state management
    std::string diag_str;
    int diag_level;

    for (unsigned int idx = 0; idx < current_diag.size(); idx++)
    {
        // Store diagnostics name+hardware_id in a single string for quick search
        diag_str = current_diag[idx].name + "_" + current_diag[idx].hardware_id;
        // Diagnostics level. Main determination for state suppression
        diag_level = static_cast<int>(current_diag[idx].level);
        // Check if diagnostic needs to be suppressed. All diagnostics with the same str
        // and no change in level are suppressed. We process only when there is change in levels.
        this->check_diag_data(robot_code, diag_str, std::to_string(diag_level));

        if (this->suppress_flag)
        {
            // If suppressed, do nothing
            // std::cout << "Suppressed!" << std::endl;
        }
        else
        {
            std::cout << "Diagnostic Message State Change! Name: " << diag_str << ", Message: " << current_diag[idx].message << std::endl;
            // If not suppressed, send it to event to update

            // Parse message to query-able format
            std::string msg_text = current_diag[idx].message;
            // std::cout << "Querying: " << msg_text << std::endl;

            // Check error classification, ECS
            json::value msg_info = this->api_instance.check_error_classification(msg_text);

            bool ecs_hit = !(msg_info.is_null());
            // std::cout << "ECS Hit: " << ecs_hit << std::endl;

            if (ecs_hit)
            {
                // Construct ROS log equivalent of diag
                rosgraph_msgs::Log rosmsg;
                rosmsg.name = diag_str;
                rosmsg.msg = current_diag[idx].message;

                if (diag_level == 2)
                {
                    rosmsg.level = rosmsg.ERROR;
                }
                else if ((diag_level == 1) || (diag_level == 3))
                {
                    rosmsg.level = rosmsg.WARN;
                }
                else
                {
                    rosmsg.level = rosmsg.INFO;
                }

                rosgraph_msgs::Log::ConstPtr data(new rosgraph_msgs::Log(rosmsg));

                this->event_instance.update_log(data, msg_info, telemetry, "ERT");

                // Push log
                this->api_instance.push_event_log(this->event_instance.get_log());
            }
            else
            {
                // ECS does not have a hit, normal operation resumes
            }

            // if (data->level == 8)
            // {
            //     // Clear everything, end of event
            //     this->clear();
            // }
            // else
            // {
            //     // Clear only log
            //     this->event_instance.clear_log();
            // }
        }
    }
}

void StateManager::check_diag_data(std::string robot_code, std::string diag_str, std::string level)
{
    // Check if diagnostic already reported
    std::vector<std::string> found = this->does_diag_exist(robot_code, diag_str, level);
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

        std::vector<std::string> diag_details;

        // Get current time
        time_t now;
        time(&now);
        char buf[sizeof "2011-10-08T07:07:09Z"];
        strftime(buf, sizeof buf, "%FT%TZ", gmtime(&now));
        std::string time_str = std::string(buf);

        // Push details to data
        diag_details.push_back(robot_code);
        diag_details.push_back(diag_str);
        diag_details.push_back(level);
        diag_details.push_back(time_str);
        this->diag_data.push_back(diag_details);

        // Do not suppress
        this->suppress_flag = false;
    }
}

std::vector<std::string> StateManager::does_diag_exist(std::string robot_code, std::string diag_str, std::string level)
{

    // Find if diagnostic is already recorded for the given robot code at the given level
    std::vector<std::vector<std::string>>::const_iterator row;
    std::vector<std::vector<std::vector<std::string>>::const_iterator> erase_list;

    for (row = this->diag_data.begin(); row != this->diag_data.end(); row++)
    {
        auto found_name = find(row->begin(), row->end(), diag_str);

        if (found_name != row->end())
        {
            // Found name, check for other parameters
            if ((find(row->begin(), row->end(), level) != row->end()) &&
                (find(row->begin(), row->end(), robot_code) != row->end()))
            {
                // Found level as well, just return the row since it is already reported
                return *(row);
            }
            else
            {
                // Level not found but name is. This means state has changed.
                // Add row to erase list.
                // Will add a new row with this name downstream.
                erase_list.push_back(row);
            }
        }
    }

    // Erase elements
    for (auto element : erase_list)
    {
        this->diag_data.erase(element);
    }

    // Return empty string if no match
    std::vector<std::string> emptyString;
    emptyString.push_back("");

    return emptyString;
}

void StateManager::clear()
{
    // Clears the state manager data for a new session
    this->suppress_flag = false;
    this->msg_data.clear();
    this->event_instance.clear();
}