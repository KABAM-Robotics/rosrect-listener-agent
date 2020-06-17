#include <rosrect-listener-agent/backend_api.h>

using namespace utility;              // Common utilities like string conversions
using namespace web;                  // Common features like URIs.
using namespace web::http;            // Common HTTP functionality
using namespace web::http::client;    // HTTP client features
using namespace concurrency::streams; // Asynchronous streams
using namespace ::pplx;               // PPLX for tasks
using namespace web::json;            // JSON features

BackendApi::BackendApi()
{

  // std::cout << "Creating API instance..." << std::endl;

  // Other environment variables
  this->robot_id = std::getenv("ROBOT_CODE");
  this->site_id = std::getenv("SITE_CODE");
  this->agent_id = std::getenv("AGENT_ID");
  this->agent_mode = std::getenv("AGENT_MODE");
  this->agent_post_api = std::getenv("AGENT_POST_API");

  // File variables
  std::string run_id;
  bool check_id = ros::param::has("run_id");
  std::string parent_dir = std::getenv("HOME");
  if (check_id)
  {
    ros::param::get("run_id", run_id);
    parent_dir.append("/.cognicept/agent/logs/" + run_id);
    std::cout << "ROS session detected" << std::endl;
  }
  else
  {
    parent_dir.append("/.cognicept/agent/logs/unittest_logs");
    std::cout << "NO ROS session detected" << std::endl;
  }

  boost::filesystem::path dir2(parent_dir);
  if (boost::filesystem::exists(dir2))
  {
    std::cout << "Agent log directory already exists: " << parent_dir << std::endl;
  }
  else
  {
    if (boost::filesystem::create_directories(dir2))
    {
      std::cout << "Agent log directory created: " << parent_dir << std::endl;
    }
  }

  this->log_name = parent_dir + "/logData";
  this->log_ext = ".json";
  this->log_id = 0;

  if (this->agent_mode == "TEST")
  {
    std::cout << "TEST mode is ON. JSON Logs will be saved here: " << parent_dir << std::endl;
  }

  /* Error classification features in development below */
  // Error classification API variables
  this->ecs_api_host = std::getenv("ECS_API");
  this->ecs_api_endpoint = "/api/ert/getErrorData/";
  this->ecs_robot_model = std::getenv("ECS_ROBOT_MODEL");
}

BackendApi::~BackendApi()
{

  // std::cout << "Logged out of API..." << std::endl;
}

pplx::task<void> BackendApi::post_event_log(json::value payload)
{

  return pplx::create_task([this, payload] {
           // Create HTTP client configuration
           http_client_config config;
           config.set_validate_certificates(false);

           // Create HTTP client
           http_client client(this->agent_post_api, config);

           // // Write the current JSON value to a stream with the native platform character width
           // utility::stringstream_t stream;
           // postData.serialize(stream);

           // // Display the string stream
           // std::cout << "Post data: " << stream.str() << std::endl;

           // Build request
           http_request req(methods::POST);
           // req.headers().add("Authorization", this->headers);
           req.set_request_uri("/api/agentstream/putRecord");
           req.set_body(payload);

           // Request ticket creation
           std::cout << "Pushing downstream..." << std::endl;
           return client.request(req);
         })
      .then([this](http_response response) {
        // If successful, print ticket details
        if (response.status_code() == status_codes::OK)
        {
          auto body = response.extract_string();
          std::wcout << "Response: " << body.get().c_str() << std::endl;
        }
        // If not, request failed
        else
        {
          std::cout << "Request failed" << std::endl;
        }
      });
}

void BackendApi::push_event_log(std::vector<std::vector<std::string>> log)
{
  // Create JSON payload and push to kinesis
  auto last_log = log.back();
  int idx = 0;

  // Get values for JSON
  std::string timestr = last_log[idx++];
  std::string level = last_log[idx++];
  std::string cflag = last_log[idx++];
  std::string module = last_log[idx++];
  std::string source = last_log[idx++];
  std::string message = last_log[idx++];
  std::string description = last_log[idx++];
  std::string resolution = last_log[idx++];
  std::string event_id = last_log[idx++];

  bool ticketBool = false;
  if ((level == "8") && ((cflag == "false") || (cflag == "Null")))
  {
    ticketBool = true;
  }
  else
  {
    ticketBool = false;
  }

  // Create JSON object
  json::value payload = json::value::object();

  // Create keys
  utility::string_t agentKey(U("agent_id"));
  utility::string_t roboKey(U("robot_id"));
  utility::string_t propKey(U("property_id"));
  utility::string_t eventidKey(U("event_id"));
  utility::string_t timeKey(U("timestamp"));
  utility::string_t msgKey(U("message"));
  utility::string_t lvlKey(U("level"));
  utility::string_t modKey(U("module"));
  utility::string_t srcKey(U("source"));
  utility::string_t cKey(U("compounding"));
  utility::string_t ticketKey(U("create_ticket"));
  utility::string_t descKey(U("description"));
  utility::string_t resKey(U("resolution"));

  // Assign key-value
  payload[agentKey] = json::value::string(U(this->agent_id));
  payload[roboKey] = json::value::string(U(this->robot_id));
  payload[propKey] = json::value::string(U(this->site_id));
  payload[eventidKey] = json::value::string(U(event_id));
  payload[timeKey] = json::value::string(U(timestr));
  payload[msgKey] = json::value::string(U(message));
  payload[lvlKey] = json::value::string(U(level));
  payload[modKey] = json::value::string(U(module));
  payload[srcKey] = json::value::string(U(source));
  payload[cKey] = json::value::string(U(cflag));
  payload[ticketKey] = json::value::boolean(U(ticketBool));
  payload[descKey] = json::value::string(U(description));
  payload[resKey] = json::value::string(U(resolution));

  if (this->agent_mode == "TEST")
  {
    // Write the current JSON value to a stream with the native platform character width
    utility::stringstream_t stream;
    payload.serialize(stream);

    // Display the string stream
    // std::cout << stream.str() << std::endl;
    std::cout << level << " level event logged with id: " << event_id << std::endl;

    // Write to file
    std::ofstream outfile;
    this->log_id++;
    std::string filename = this->log_name + std::to_string(this->log_id) + this->log_ext;
    std::cout << filename << std::endl;
    outfile.open(filename);
    outfile << std::setw(4) << stream.str() << std::endl;
    outfile.close();
  }

  // Post downstream
  this->post_event_log(payload);
}

json::value BackendApi::create_event_log(std::vector<std::vector<std::string>> log)
{

  // Create JSON object
  json::value event_log = json::value::array();

  // Create keys
  utility::string_t cKey(U("Compounding"));
  utility::string_t timeKey(U("Date/Time"));
  utility::string_t descKey(U("Description"));
  utility::string_t lvlKey(U("Level"));
  utility::string_t msgKey(U("Message"));
  utility::string_t modKey(U("Module"));
  utility::string_t qidKey(U("QID"));
  utility::string_t resKey(U("Resolution"));
  utility::string_t eidKey(U("RobotEvent_ID"));
  utility::string_t srcKey(U("Source"));

  // std::cout << "Creating JSON log" << std::endl;
  for (int queue_id = 0; queue_id < log.size(); queue_id++)
  {

    // Get row
    auto current_row = log[queue_id];
    int idx = 0;

    // Retrieve data
    std::string timestr = current_row[idx++];
    std::string level = current_row[idx++];
    std::string cflag = current_row[idx++];
    std::string module = current_row[idx++];
    std::string source = current_row[idx++];
    std::string message = current_row[idx++];
    std::string description = current_row[idx++];
    std::string resolution = current_row[idx++];
    std::string event_id = current_row[idx++];
    std::string qidstr = std::to_string(queue_id);

    // Assign key-value
    event_log[queue_id][timeKey] = json::value::string(U(timestr));
    event_log[queue_id][lvlKey] = json::value::string(U(level));
    event_log[queue_id][cKey] = json::value::string(U(cflag));
    event_log[queue_id][modKey] = json::value::string(U(module));
    event_log[queue_id][srcKey] = json::value::string(U(source));
    event_log[queue_id][msgKey] = json::value::string(U(message));
    event_log[queue_id][descKey] = json::value::string(U(description));
    event_log[queue_id][resKey] = json::value::string(U(resolution));
    event_log[queue_id][eidKey] = json::value::string(U(event_id));
    event_log[queue_id][qidKey] = json::value::string(U(qidstr));
  }

  return (event_log);
}

/* Error Classification Features in development below */

pplx::task<void> BackendApi::query_error_classification(std::string msg_text)
{

  return pplx::create_task([this, msg_text] {
           // Create HTTP client configuration
           http_client_config config;
           config.set_validate_certificates(false);

           // Create HTTP client
           http_client client(this->ecs_api_host, config);

           // Build request
           http_request req(methods::GET);

           // Build request URI.
           uri_builder builder(this->ecs_api_endpoint);
           builder.append_query("RobotModel", this->ecs_robot_model);
           builder.append_query("ErrorText", msg_text);
           req.set_request_uri(builder.to_string());

           return client.request(req);
         })
      .then([this](http_response response) {
        // If successful, return JSON query
        if (response.status_code() == status_codes::OK)
        {
          auto body = response.extract_string();
          std::string body_str = body.get().c_str();
          this->msg_resp = body_str;
        }
        // If not, request failed
        else
        {
          std::cout << "Request failed" << std::endl;
        }
      });
}

json::value BackendApi::check_error_classification(std::string msg_text)
{

  this->query_error_classification(msg_text).wait();
  std::string temp_msg = this->msg_resp;
  json::value response = json::value::parse(temp_msg);
  json::value response_data;

  try
  {
    // std::cout << "Trying to get data..." << std::endl;
    response_data = response.at(U("data"));

    // // Write the current JSON value to a stream with the native platform character width
    // utility::stringstream_t stream;
    // response_data.serialize(stream);

    // Display the string stream
    // std::cout << "Get data: " << stream.str() << std::endl;
    return response_data[0];
  }
  catch (const json::json_exception &e)
  {
    // const std::exception& e
    // std::cout << " Can't get data, returning null" << std::endl;
    response_data = json::value::null();
    return response_data;
  }
}
