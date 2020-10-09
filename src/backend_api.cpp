#include <error_resolution_diagnoser/backend_api.h>

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
  // Check and set environment variables
  this->check_environment();

  // File variables
  this->log_dir = std::getenv("HOME");
  std::string run_id;
  bool check_id = ros::param::has("run_id");
  this->log_dir.append("/.cognicept/agent/logs/");
  std::string latest_log = this->log_dir + "latest_log_loc.txt";
  std::string disp_dir = "/$HOME/.cognicept/agent/logs/";

  if (check_id)
  {
    ros::param::get("run_id", run_id);
    this->log_dir.append(run_id);
    disp_dir.append(run_id);
    std::cout << "ROS session detected" << std::endl;
  }
  else
  {
    this->log_dir.append("unittest_logs");
    std::cout << "NO ROS session detected" << std::endl;
  }

  boost::filesystem::path dir2(this->log_dir);
  if (boost::filesystem::exists(dir2))
  {
    std::cout << "Agent log directory already exists: " << disp_dir << std::endl;
  }
  else
  {
    if (boost::filesystem::create_directories(dir2))
    {
      std::cout << "Agent log directory created: " << disp_dir << std::endl;
    }
  }

  // Write to file
  std::ofstream outfile;
  outfile.open(latest_log);
  outfile << std::setw(4) << disp_dir << std::endl;
  outfile.close();
  std::cout << "Updated latest log location in: "
            << "/$HOME/.cognicept/agent/logs/latest_log_loc.txt" << std::endl;

  this->log_name = this->log_dir + "/logData";
  this->log_ext = ".json";
  this->log_id = 0;

  if (this->agent_mode != "PROD")
  {
    std::cout << "TEST mode is ON. JSON Logs will be saved here: " << disp_dir << std::endl;
  }

  // Error classification API variables

  if ((this->agent_type == "ERT") || (this->agent_type == "DB"))
  {
    // This configures the endpoint to ERT queries. Must be used only for testing. Undocumented.
    this->ecs_api_endpoint = "/ert/error-data/";
  }
  else if (this->agent_type == "ECS")
  {
    // This configures the endpoint to ECS queries. Production version.
    this->ecs_api_endpoint = "/ecs/error-data/";
  }
  else
  {
    // This means the agent is running in ROS mode. So no need to configure endpoint.
    this->ecs_api_endpoint = "/api/ert/getErrorData/";
  }
}

BackendApi::~BackendApi()
{

  // std::cout << "Logged out of API..." << std::endl;
}

void BackendApi::check_environment()
{
  // Other environment variables
  std::cout << "=======================Environment variables setup======================" << std::endl;
  // AGENT_TYPE
  if (std::getenv("AGENT_TYPE"))
  {
    // Success case
    this->agent_type = std::getenv("AGENT_TYPE");
    std::cout << "Environment variable AGENT_TYPE set to: " << this->agent_type << std::endl;
    // ECS_API, ECS_ROBOT_MODEL
    if ((this->agent_type == "DB") || (this->agent_type == "ERT") || (this->agent_type == "ECS"))
    {
      if (std::getenv("ECS_API"))
      {
        // Success case
        this->ecs_api_host = std::getenv("ECS_API");
        std::cout << "Environment variable ECS_API set to: " << this->ecs_api_host << std::endl;
        if (std::getenv("ECS_ROBOT_MODEL"))
        {
          // Success case
          this->ecs_robot_model = std::getenv("ECS_ROBOT_MODEL");
          std::cout << "Environment variable ECS_ROBOT_MODEL set to: " << this->ecs_robot_model << std::endl;
        }
        else
        {
          // Failure case - Default
          std::cerr << "Agent configured in " << this->agent_type << " mode but ECS_ROBOT_MODEL environment variable is not configured. Defaulting back to ROS mode instead..." << std::endl;
          this->agent_type = "ROS";
          this->agent_type = "ROS";
        }
      }
      else
      {
        // Failure case - Default
        std::cerr << "Agent configured in " << this->agent_type << " mode but ECS_API environment variable is not configured. Defaulting back to ROS mode instead..." << std::endl;
        this->agent_type = "ROS";
      }
    }
  }
  else
  {
    // Failure case - Default
    this->agent_type = "ROS";
    std::cerr << "Environment variable AGENT_TYPE unspecified. Defaulting to ROS mode..." << std::endl;
  }

  // ROBOT_CODE
  if (std::getenv("ROBOT_CODE"))
  {
    // Success case
    this->robot_id = std::getenv("ROBOT_CODE");
    std::cout << "Environment variable ROBOT_CODE set to: " << this->robot_id << std::endl;
  }
  else
  {
    // Failure case - Default
    this->robot_id = "Undefined";
    std::cerr << "Environment variable ROBOT_CODE unspecified. Defaulting to 'Undefined'..." << std::endl;
  }

  // SITE_CODE
  if (std::getenv("SITE_CODE"))
  {
    // Success case
    this->site_id = std::getenv("SITE_CODE");
    std::cout << "Environment variable SITE_CODE set to: " << this->site_id << std::endl;
  }
  else
  {
    // Failure case - Default
    this->site_id = "Undefined";
    std::cerr << "Environment variable SITE_CODE unspecified. Defaulting to 'Undefined'..." << std::endl;
  }

  // AGENT_ID
  if (std::getenv("AGENT_ID"))
  {
    // Success case
    this->agent_id = std::getenv("AGENT_ID");
    std::cout << "Environment variable AGENT_ID set to: " << this->agent_id << std::endl;
  }
  else
  {
    // Failure case - Default
    this->agent_id = "Undefined";
    std::cerr << "Environment variable AGENT_ID unspecified. Defaulting to 'Undefined'..." << std::endl;
  }

  // AGENT_MODE, AGENT_POST_API
  if (std::getenv("AGENT_MODE"))
  {
    // Success case
    this->agent_mode = std::getenv("AGENT_MODE");
    std::cout << "Environment variable AGENT_MODE set to: " << this->agent_mode << std::endl;
    // Specially handle POST_TEST case
    if (this->agent_mode == "POST_TEST")
    {
      if (std::getenv("AGENT_POST_API"))
      {
        // Success case
        this->agent_post_api = std::getenv("AGENT_POST_API");
        std::cout << "Environment variable AGENT_POST_API set to: " << this->agent_post_api << std::endl;
      }
      else
      {
        // Failure case - Default
        this->agent_mode = "JSON_TEST";
        std::cerr << "Agent configured in POST_TEST mode but AGENT_POST_API environment variable is not configured. Defaulting back to JSON_TEST mode instead..." << std::endl;
      }
    }
  }
  else
  {
    // Failure case - Default
    this->agent_mode = "JSON_TEST";
    std::cerr << "Environment variable AGENT_MODE unspecified. Defaulting to 'JSON_TEST'..." << std::endl;
  }

  std::cout << "=========================================================================" << std::endl;
}

pplx::task<void> BackendApi::post_event_log(json::value payload)
{
  std::cout << "Posting" << std::endl;

  return pplx::create_task([this, payload] {
           // Create HTTP client configuration
           http_client_config config;
           config.set_validate_certificates(false);

           // Create HTTP client
           http_client client(this->agent_post_api, config);

           // // Write the current JSON value to a stream with the native platform character width
           // utility::stringstream_t stream;
           // payload.serialize(stream);

           // // Display the string stream
           // std::cout << "Post data: " << stream.str() << std::endl;

           // Build request
           http_request req(methods::POST);
           // req.headers().add("Authorization", this->headers);
           if (this->agent_post_api == "https://postman-echo.com")
           {
             req.set_request_uri("/post");
           }
           else
           {
             req.set_request_uri("/agentstream/put-record");
           }
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

void BackendApi::push_status(bool status, json::value telemetry)
{
  // Set all required info
  boost::posix_time::ptime utcTime = boost::posix_time::microsec_clock::universal_time();
  std::string timestr = to_iso_extended_string(utcTime);
  std::string level = "Heartbeat";
  std::string cflag = "Null";
  std::string module = "Status";
  std::string source = "Null";
  std::string message = "Null";
  std::string description = "Null";
  std::string resolution = "Null";
  std::string event_id = "Null";
  bool ticketBool = false;

  if (status)
  {
    message = "Online";
    ticketBool = false;
  }
  else
  {
    message = "Offline";
    ticketBool = true;
  }

  // Create JSON object
  json::value payload = json::value::object();

  // Create keys
  utility::string_t agentKey(utility::conversions::to_string_t("agent_id"));
  utility::string_t roboKey(utility::conversions::to_string_t("robot_id"));
  utility::string_t propKey(utility::conversions::to_string_t("property_id"));
  utility::string_t eventidKey(utility::conversions::to_string_t("event_id"));
  utility::string_t timeKey(utility::conversions::to_string_t("timestamp"));
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  utility::string_t lvlKey(utility::conversions::to_string_t("level"));
  utility::string_t modKey(utility::conversions::to_string_t("module"));
  utility::string_t srcKey(utility::conversions::to_string_t("source"));
  utility::string_t cKey(utility::conversions::to_string_t("compounding"));
  utility::string_t ticketKey(utility::conversions::to_string_t("create_ticket"));
  utility::string_t descKey(utility::conversions::to_string_t("description"));
  utility::string_t resKey(utility::conversions::to_string_t("resolution"));
  utility::string_t telKey(utility::conversions::to_string_t("telemetry"));

  // Assign key-value
  payload[agentKey] = json::value::string(utility::conversions::to_string_t(this->agent_id));
  payload[roboKey] = json::value::string(utility::conversions::to_string_t(this->robot_id));
  payload[propKey] = json::value::string(utility::conversions::to_string_t(this->site_id));
  payload[eventidKey] = json::value::string(utility::conversions::to_string_t(event_id));
  payload[timeKey] = json::value::string(utility::conversions::to_string_t(timestr));
  payload[msgKey] = json::value::string(utility::conversions::to_string_t(message));
  payload[lvlKey] = json::value::string(utility::conversions::to_string_t(level));
  payload[modKey] = json::value::string(utility::conversions::to_string_t(module));
  payload[srcKey] = json::value::string(utility::conversions::to_string_t(source));
  payload[cKey] = json::value::string(utility::conversions::to_string_t(cflag));
  payload[ticketKey] = json::value::boolean(ticketBool);
  payload[descKey] = json::value::string(utility::conversions::to_string_t(description));
  payload[resKey] = json::value::string(utility::conversions::to_string_t(resolution));
  payload[telKey] = telemetry;

  if (this->agent_mode == "JSON_TEST")
  {
    // Write the current JSON value to a stream with the native platform character width
    utility::stringstream_t stream;
    payload.serialize(stream);

    // Display the string stream
    // std::cout << stream.str() << std::endl;
    std::cout << "Status Logged: " << message << std::endl;

    // Write to file
    std::ofstream outfile;
    std::string filename = this->log_name + "Status" + this->log_ext;
    // std::cout << filename << std::endl;
    outfile.open(filename);
    outfile << std::setw(4) << stream.str() << std::endl;
    outfile.close();
  }
  else if (this->agent_mode == "POST_TEST")
  {
    // Write the current JSON value to a stream with the native platform character width
    utility::stringstream_t stream;
    payload.serialize(stream);

    // Display the string stream
    // std::cout << stream.str() << std::endl;
    std::cout << "Status Logged: " << message << std::endl;

    // Write to file
    std::ofstream outfile;
    std::string filename = this->log_name + "Status" + this->log_ext;
    // std::cout << filename << std::endl;
    outfile.open(filename);
    outfile << std::setw(4) << stream.str() << std::endl;
    outfile.close();

    // Post downstream
    try
    {
      this->post_event_log(payload).wait();
    }
    catch (const http::http_exception &e)
    {
      std::cerr << "POST API error: " << e.what() << ". Agent will retry API connection at: " << this->agent_post_api  << std::endl;
    }
  }
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
  std::string telemetry_str = last_log[idx++];
  std::string event_id = last_log[idx++];

  bool ticketBool = false;
  if (((level == "8") || (level == "16")) && ((cflag == "false") || (cflag == "Null")))
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
  utility::string_t agentKey(utility::conversions::to_string_t("agent_id"));
  utility::string_t roboKey(utility::conversions::to_string_t("robot_id"));
  utility::string_t propKey(utility::conversions::to_string_t("property_id"));
  utility::string_t eventidKey(utility::conversions::to_string_t("event_id"));
  utility::string_t timeKey(utility::conversions::to_string_t("timestamp"));
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  utility::string_t lvlKey(utility::conversions::to_string_t("level"));
  utility::string_t modKey(utility::conversions::to_string_t("module"));
  utility::string_t srcKey(utility::conversions::to_string_t("source"));
  utility::string_t cKey(utility::conversions::to_string_t("compounding"));
  utility::string_t ticketKey(utility::conversions::to_string_t("create_ticket"));
  utility::string_t descKey(utility::conversions::to_string_t("description"));
  utility::string_t resKey(utility::conversions::to_string_t("resolution"));
  utility::string_t telKey(utility::conversions::to_string_t("telemetry"));

  // Assign key-value
  payload[agentKey] = json::value::string(utility::conversions::to_string_t(this->agent_id));
  payload[roboKey] = json::value::string(utility::conversions::to_string_t(this->robot_id));
  payload[propKey] = json::value::string(utility::conversions::to_string_t(this->site_id));
  payload[eventidKey] = json::value::string(utility::conversions::to_string_t(event_id));
  payload[timeKey] = json::value::string(utility::conversions::to_string_t(timestr));
  payload[msgKey] = json::value::string(utility::conversions::to_string_t(message));
  payload[lvlKey] = json::value::string(utility::conversions::to_string_t(level));
  payload[modKey] = json::value::string(utility::conversions::to_string_t(module));
  payload[srcKey] = json::value::string(utility::conversions::to_string_t(source));
  if (cflag == "false")
  {
    payload[cKey] = json::value::boolean(false);
  }
  else if (cflag == "true")
  {
    payload[cKey] = json::value::boolean(true);
  }
  else
  {
    payload[cKey] = json::value::string(utility::conversions::to_string_t("Null"));
  }
  payload[ticketKey] = json::value::boolean(ticketBool);
  payload[descKey] = json::value::string(utility::conversions::to_string_t(description));
  payload[resKey] = json::value::string(utility::conversions::to_string_t(resolution));
  payload[telKey] = json::value::parse(utility::conversions::to_string_t(telemetry_str));

  if (this->agent_mode == "JSON_TEST")
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
  else if (this->agent_mode == "POST_TEST")
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

    // Post downstream
    try
    {
      this->post_event_log(payload).wait();
    }
    catch (const http::http_exception &e)
    {
      std::cerr << "POST API error: " << e.what() << ". Agent will retry API connection at: " << this->agent_post_api << std::endl;
    }
  }
}

json::value BackendApi::create_event_log(std::vector<std::vector<std::string>> log)
{

  // Create JSON object
  json::value event_log = json::value::array();

  // Create keys
  utility::string_t cKey(utility::conversions::to_string_t("Compounding"));
  utility::string_t timeKey(utility::conversions::to_string_t("Date/Time"));
  utility::string_t descKey(utility::conversions::to_string_t("Description"));
  utility::string_t lvlKey(utility::conversions::to_string_t("Level"));
  utility::string_t msgKey(utility::conversions::to_string_t("Message"));
  utility::string_t modKey(utility::conversions::to_string_t("Module"));
  utility::string_t qidKey(utility::conversions::to_string_t("QID"));
  utility::string_t resKey(utility::conversions::to_string_t("Resolution"));
  utility::string_t eidKey(utility::conversions::to_string_t("RobotEvent_ID"));
  utility::string_t srcKey(utility::conversions::to_string_t("Source"));

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
    event_log[queue_id][timeKey] = json::value::string(utility::conversions::to_string_t(timestr));
    event_log[queue_id][lvlKey] = json::value::string(utility::conversions::to_string_t(level));
    event_log[queue_id][cKey] = json::value::string(utility::conversions::to_string_t(cflag));
    event_log[queue_id][modKey] = json::value::string(utility::conversions::to_string_t(module));
    event_log[queue_id][srcKey] = json::value::string(utility::conversions::to_string_t(source));
    event_log[queue_id][msgKey] = json::value::string(utility::conversions::to_string_t(message));
    event_log[queue_id][descKey] = json::value::string(utility::conversions::to_string_t(description));
    event_log[queue_id][resKey] = json::value::string(utility::conversions::to_string_t(resolution));
    event_log[queue_id][eidKey] = json::value::string(utility::conversions::to_string_t(event_id));
    event_log[queue_id][qidKey] = json::value::string(utility::conversions::to_string_t(qidstr));
  }

  return (event_log);
}

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
  json::value response = json::value::null();
  json::value response_data = json::value::null();

  try
  {
    this->query_error_classification(msg_text).wait();
    std::string temp_msg = this->msg_resp;
    response = json::value::parse(temp_msg);
  }
  catch (const http::http_exception &e)
  {
    std::cerr << "ECS API error: " << e.what() << ". Agent will retry API connection at: " << this->ecs_api_host + this->ecs_api_endpoint << std::endl;
  }

  try
  {
    // std::cout << "Trying to get data..." << std::endl;
    response_data = response.at(utility::conversions::to_string_t("data"));

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
    // response_data = json::value::null();
    return response_data;
  }
}
