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
#include <ros/package.h>

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams
using namespace ::pplx;                     // PPLX for tasks
using namespace web::json;                  // JSON features

class BackendApi {
  // This class provides access to Error Classification API and event log creation through C++.

  std::string robot_id;
  std::string site_id;
  std::string agent_id;
  std::string agent_mode;
  std::string log_name;
  std::string log_ext;
  int log_id;
  std::string error_api_host;
  std::string error_api_endpoint;
  std::string msg_resp;
  
  public:
  BackendApi();
  ~BackendApi();
  void push_event_log(std::vector<std::vector<std::string>>);
  json::value create_event_log(std::vector<std::vector<std::string>>); // Create JSON payload data for downstream consumption
  pplx::task<void> query_error_classification(std::string); // Query error classification database table
  json::value check_error_classification(std::string); // Entry point for error classification
};