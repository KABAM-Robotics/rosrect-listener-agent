#define _CRT_SECURE_NO_WARNINGS
#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
#include <cpprest/json.h>
#include <cpprest/containerstream.h>
#include <cpprest/producerconsumerstream.h>
#undef U
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <locale>
#include <codecvt>
#include <boost/algorithm/string.hpp>

class BackendApi
{
  // This class provides access to Error Classification API and event log creation through C++.

  std::string robot_id;                  // ENV variable/Undefined - identifies a robot uniquely, such as an UUID
  std::string site_id;                   // ENV variable/Undefined - identifies a site uniquely, such as an UUID
  std::string agent_id;                  // ENV variable/Undefined - identifies an agent uniquely, such as an UUID
  std::string agent_mode;                // ENV variable JSON_TEST(default), POST_TEST - determines if the agent saves local JSON logs or passes it off to a configurable POST API endpoint
  std::string agent_type;                // ENV variable ROS(default), DB - determines if the agent takes ROS messages directly or passes through Error Classification first
  std::string log_dir;                   // Stores the log directory location
  std::string log_name;                  // Stores the directory along with log name
  std::string log_ext;                   // Stores the log file extension type
  int log_id;                            // Incremental log id #
  std::string msg_resp;                  // Stores the ECS response
  std::string ecs_api_host;              // ENV variable that specifies the host for the ECS API
  std::string ecs_api_endpoint;          // Stores the endpoint for the ECS API. Based on AGENT_TYPE this is automatically configured.
  std::string ecs_robot_model;           // ENV variable that specifies the type of robot. Currently use Turtlebot3 for any /move_base navigation stack.
  std::string agent_post_api;            // Any valid POST API endpoint which the agent can directly submit data to in addition to creating local logs.
  std::vector<std::string> node_list;    // List of nodes to include messages by
  std::vector<std::string> node_ex_list; // List of nodes to exclude messages by
  std::string diag_setting;              // Keeps track of the diagnostics setting on or off

public:
  BackendApi();
  ~BackendApi();
  void check_environment();                                                 // Utility method to pull environment variables and set defaults
  pplx::task<void> post_event_log(web::json::value);                        // A configurable downstream push method
  void push_status(bool, web::json::value);                                 // Pushes appropriate status data
  void push_event_log(std::vector<std::vector<std::string>>);               // Create and push single JSON record payload data for downstream consumption
  web::json::value create_event_log(std::vector<std::vector<std::string>>); // Create JSON "multiple record" payload data for downstream consumption
  pplx::task<void> query_error_classification(std::string);                 // Query error classification database table
  web::json::value check_error_classification(std::string);                 // Entry point for error classification
};