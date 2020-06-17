#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
#include <cpprest/json.h>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <cpprest/containerstream.h>
#include <cpprest/producerconsumerstream.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>

class BackendApi {
  // This class provides access to Error Classification API and event log creation through C++.

  std::string robot_id;
  std::string site_id;
  std::string agent_id;
  std::string agent_mode;
  std::string log_name;
  std::string log_ext;
  int log_id;
  std::string msg_resp;
  /* Error classification features in development below */
  std::string ecs_api_host;
  std::string ecs_api_endpoint;
  std::string ecs_robot_model;
  /* Downstream push/post variables below */
  std::string agent_post_api;

  public:
  BackendApi();
  ~BackendApi();
  pplx::task<void> post_event_log(web::json::value); // A configurable downstream push method
  void push_event_log(std::vector<std::vector<std::string>>); // Create and push single JSON record payload data for downstream consumption
  web::json::value create_event_log(std::vector<std::vector<std::string>>); // Create JSON "multiple record" payload data for downstream consumption
  /* Error classification features in development below */
  pplx::task<void> query_error_classification(std::string); // Query error classification database table
  web::json::value check_error_classification(std::string); // Entry point for error classification
  
};