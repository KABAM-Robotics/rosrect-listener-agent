#include <rosrect-listener-agent/backend_api.h>

BackendApi::BackendApi() {

  std::cout << "Creating API instance..." << std::endl;

  // Assign member variables that configure the API
  this->username = std::getenv("API_USERNAME");
  this->password = std::getenv("API_PASSWORD");
  this->hostname = std::getenv("API_URL");

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