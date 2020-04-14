#include <rosrect-listener-agent/backend_api.h>

BackendApi::BackendApi() {

  std::cout << "Creating API instance..." << std::endl;

  // Assign member variables that configure the API
  this->username = std::getenv("API_USERNAME");
  this->password = std::getenv("API_PASSWORD");
  this->hostname = std::getenv("API_URL");
  
  // Other environment variables
  this->robot_id = std::getenv("ROBOT_CODE");
  this->site_id = std::getenv("SITE_CODE");
  this->agent_id = std::getenv("AGENT_ID");
  this->agent_mode = std::getenv("AGENT_MODE");

  // File variables
  this->log_name = "/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/tests/logs/logData";
  this->log_ext = ".json";
  this->log_id = 0;

  // Error classification API variables
  this->error_api_host = std::getenv("ECS_API");
  this->error_api_endpoint = "/api/getErrorData/";

  // Login to register access token
  this->login().wait();
}

BackendApi::~BackendApi() {

  std::cout << "Logged out of API..." << std::endl;
}

pplx::task<void> BackendApi::login () {

  return pplx::create_task([this]
  {
    // Create HTTP client configuration
    http_client_config config;
    config.set_validate_certificates(false);
    
    // Create HTTP client
    http_client client(this->hostname, config);

    // Get login details
    json::value postData = create_login_json();

    // // Write the current JSON value to a stream with the native platform character width
    // utility::stringstream_t stream;
    // postData.serialize(stream);

    // // Display the string stream
    // std::cout << "Post data: " << stream.str() << std::endl;

    // Build request
    http_request req(methods::POST);
    req.set_request_uri("/user/login");
    req.set_body(postData);
    
    // Request login
    std::cout << "Logging into API..." << std::endl;
    return client.request(req);

  })
  .then([this](http_response response)
  {
    // If request is successful, print message
    if(response.status_code() == status_codes::OK)
    {
      auto body = response.extract_string();
      std::string token = body.get().c_str();
      std::size_t pos_start = token.find(":") + 3;  // start of token
      std::size_t token_length = token.rfind("}") - 1 - pos_start; // token length
      token = token.substr(pos_start, token_length);
      this->headers = std::string("Bearer ") + token;
      std::cout << "Token registered..." << std::endl;
    }
    // If not, report failure
    else
    {
      std::cout << "Unable to register token..." << std::endl;
    }
    
  });
}

json::value BackendApi::create_login_json() {

  // Create Login JSON payload

  //  Sample JSON
  // { 'username': self.username,
  //   'password': self.password }
  
  // Create JSON objects
  json::value payload = json::value::object();

  // Create keys
  utility::string_t usrKey(U("username"));
  utility::string_t pwdKey(U("password"));

  // Create values
  std::string usrStr = this->username;
  std::string pwdStr = this->password;
  
  // Assign key-value
  payload[usrKey] = json::value::string(U(usrStr));
  payload[pwdKey] = json::value::string(U(pwdStr));

  // Return payload
  return payload;
}

pplx::task<void> BackendApi::create_ticket (ticketDetails ticket_info) {

  return pplx::create_task([this, ticket_info]
  {
    // Create HTTP client configuration
    http_client_config config;
    config.set_validate_certificates(false);
    
    // Create HTTP client
    http_client client(this->hostname, config);

    // Get ticket details
    json::value postData = create_ticket_json(ticket_info);

    // // Write the current JSON value to a stream with the native platform character width
    // utility::stringstream_t stream;
    // postData.serialize(stream);

    // // Display the string stream
    // std::cout << "Post data: " << stream.str() << std::endl;

    // Build request
    http_request req(methods::POST);
    req.headers().add("Authorization", this->headers);
    req.set_request_uri("/tickets/create");
    req.set_body(postData);
    
    // Request ticket creation
    std::cout << "Creating ticket..." << std::endl;
    return client.request(req);

  })
  .then([this](http_response response)
  {
    // If successful, print ticket details
    if(response.status_code() == status_codes::OK)
    {
        auto body = response.extract_string();
        // std::wcout << "Response: " << body.get().c_str() << std::endl;
        std::string ticket_details = body.get().c_str();
        std::size_t pos_start = ticket_details.find("ticket_id") + 12;
        std::size_t id_length = 6;
        std::string ticket_id = ticket_details.substr(pos_start, id_length);
        std::cout << "Ticket: " << ticket_id << " created..." << std::endl;
    }
    // If not, request failed
    else
    {
      std::cout << "Request failed" << std::endl;
    }
  });
}

json::value BackendApi::create_ticket_json(ticketDetails ticket_info) {

  // Create Ticket details JSON payload

  //  Sample JSON
  // "data" : {
  //             "subject": subjectStr,
  //             "description": descStr,
  //             "property": "KM",
  //             "robot": self.mir_status['robot_code']
  // }
  
  // Create JSON objects
  json::value payload = json::value::object();
  json::value data = json::value::object();

  // Create keys
  utility::string_t subKey(U("subject"));
  utility::string_t descKey(U("description"));
  utility::string_t propKey(U("property"));
  utility::string_t roboKey(U("robot"));
  utility::string_t dataKey(U("data"));

  // Create values
  std::string subStr = "Error in Module: " + ticket_info.module_name;
  std::string descStr = "Error Text: " + ticket_info.error_text + "\n Generated by Agent: " + ticket_info.agent_id;
  std::string propStr = ticket_info.prop_id;
  std::string roboStr = ticket_info.robot_id;

  // Assign key-value
  data[subKey] = json::value::string(U(subStr));
  data[descKey] = json::value::string(U(descStr));
  data[propKey] = json::value::string(U(propStr));
  data[roboKey] = json::value::string(U(roboStr));

  payload[dataKey] = data;

  // Return payload
  return payload;
}

void BackendApi::push_kinesis(std::vector<std::vector<std::string>> log){
  // Create JSON payload and push to kinesis
  auto last_log = log.back();
  int i = 0;

  // Get values for JSON
  std::string stop_time = last_log[i++];
  std::string event_category = last_log[i++];
  std::string cflag = last_log[i++];
  std::string module_name = last_log[i++];
  std::string event_id = last_log.back();
  json::value event_log = this->create_event_log(log);
  // std::string event_log = "Log";
  bool ticketBool = false;
  if((event_category == "Error") && ((cflag == "false") || (cflag == "Null"))){
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
  utility::string_t logKey(U("event_log"));
  utility::string_t catKey(U("event_category"));
  utility::string_t modKey(U("module_name"));
  utility::string_t ticketKey(U("create_ticket"));

  // Assign key-value
  payload[agentKey] = json::value::string(U(this->agent_id));
  payload[roboKey] = json::value::string(U(this->robot_id));
  payload[propKey] = json::value::string(U(this->site_id));
  payload[eventidKey] = json::value::string(U(event_id));
  payload[timeKey] = json::value::string(U(stop_time));
  payload[logKey] = event_log;
  // payload[logKey] = json::value::string(U(event_log));
  payload[catKey] = json::value::string(U(event_category));
  payload[modKey] = json::value::string(U(module_name));
  payload[ticketKey] = json::value::boolean(U(ticketBool));
  
  if(this->agent_mode == "TEST"){
    // Write the current JSON value to a stream with the native platform character width
    utility::stringstream_t stream;
    payload.serialize(stream);

    // Display the string stream
    // std::cout << stream.str() << std::endl;
    std::cout << "Event logged: " << event_id << std::endl;
    
    // Write to file
    std::ofstream outfile;
    this->log_id++;
    std::string filename = this->log_name + std::to_string(this->log_id) + this->log_ext;
    std::cout << filename << std::endl;
    outfile.open(filename);
    outfile << std::setw(4) << stream.str() << std::endl;
    outfile.close();
  }
  
}

json::value BackendApi::create_event_log(std::vector<std::vector<std::string>> log){
  
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
  for(int queue_id = 0; queue_id < log.size(); queue_id++)  {

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

  return(event_log);

}

pplx::task<void> BackendApi::check_error_classification (std::string msg_text) {
  
  
  return pplx::create_task([this, msg_text]
  {
    // Create HTTP client configuration
    http_client_config config;
    config.set_validate_certificates(false);
    
    // Create HTTP client
    http_client client(this->error_api_host, config);

    // Build request
    http_request req(methods::GET);

    // Build request URI.
    uri_builder builder(this->error_api_endpoint);
    builder.append_path(msg_text, true);
    req.set_request_uri(builder.to_string());
        
    return client.request(req);

  })
  .then([this](http_response response)
  {
    
    // If successful, return JSON query
    if(response.status_code() == status_codes::OK)
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