#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
#include <cpprest/json.h>
#include <string>
#include <iostream>
#include <sstream>
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

  std::string username, password, hostname, headers;

  public:
  BackendApi();
  ~BackendApi();
  pplx::task<void> login(); // Login to register access token
  json::value create_login_json(); // Create JSON payload data to feed login POST request
  pplx::task<void> create_ticket(ticketDetails); // Main create ticket post request
  json::value create_ticket_json(ticketDetails); // Create JSON payload data to feed create ticket POST request

};