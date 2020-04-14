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

using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams
using namespace ::pplx;                     // PPLX for tasks
using namespace web::json;                  // JSON features

struct ticketDetails {
    std::string module_name;
    std::string error_text;
    std::string agent_id;
    std::string robot_id;
    std::string prop_id;
};

class BackendApi {
  // This class provides access to Cognicept Ticket creation API through C++.

  std::string username;
  std::string password;
  std::string hostname;
  std::string headers;
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
  pplx::task<void> login(); // Login to register access token
  json::value create_login_json(); // Create JSON payload data to feed login POST request
  pplx::task<void> create_ticket(ticketDetails); // Main create ticket post request
  json::value create_ticket_json(ticketDetails); // Create JSON payload data to feed create ticket POST request
  void push_kinesis(std::vector<std::vector<std::string>>);
  json::value create_event_log(std::vector<std::vector<std::string>>); // Create JSON payload data for downstream consumption
  pplx::task<void> query_error_classification(std::string); // Query error classification database table
  json::value check_error_classification(std::string); // Entry point for error classification
};