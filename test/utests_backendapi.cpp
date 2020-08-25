#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>
#include <rosrect-listener-agent/backend_api.h>

using namespace utility;              // Common utilities like string conversions
using namespace web;                  // Common features like URIs.
using namespace web::http;            // Common HTTP functionality
using namespace web::http::client;    // HTTP client features
using namespace concurrency::streams; // Asynchronous streams
using namespace ::pplx;               // PPLX for tasks
using namespace web::json;            // JSON features

// Log file settings
std::string run_id;
std::string parent_dir = std::getenv("HOME");
std::string log_name = parent_dir.append("/.cognicept/agent/logs/unittest_logs") + "/logData";
std::string log_ext = ".json";
int log_id = 0;

// Create test object
BackendApi api_instance;

// Create sample log
std::vector<std::vector<std::string>> sample_log;
std::string time_str = "2020-07-03T05:31:40.131383";
std::string level = "8";
std::string cflag = "false";
std::string module = "Null";
std::string source = "/move_base";
std::string message = "Aborting because a valid plan is not found";
std::string description = "Null";
std::string resolution = "Null";
std::string event_id_str = "Sample id";
std::string telemetry = "{ \"pose\" : 42 }";

// Hold the record
std::vector<std::string> event_details;

TEST(BackEndApiTestSuite, pushTest)
{
  // Push to details
  event_details.push_back(time_str);
  event_details.push_back(level);
  event_details.push_back(cflag);
  event_details.push_back(module);
  event_details.push_back(source);
  event_details.push_back(message);
  event_details.push_back(description);
  event_details.push_back(resolution);
  event_details.push_back(telemetry);
  event_details.push_back(event_id_str);

  // Push to sample_log
  sample_log.push_back(event_details);

  // Push event
  api_instance.push_event_log(sample_log);

  // Get log file
  int log_id = 1;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile(filename);
  bool fileflag = infile.good();
  ASSERT_TRUE(fileflag);
}

TEST(BackEndApiTestSuite, jsonTest)
{
  json::value eventJson;
  eventJson = api_instance.create_event_log(sample_log);

  // Check if returned value is an array
  bool arrayFlag = eventJson.is_array();
  ASSERT_TRUE(arrayFlag);

  // Check if it has the right fields according to schema
  bool fieldFlag = true;
  // List all field names
  std::vector<std::string> fieldNames;
  fieldNames.push_back("Compounding");
  fieldNames.push_back("Date/Time");
  fieldNames.push_back("Description");
  fieldNames.push_back("Level");
  fieldNames.push_back("Message");
  fieldNames.push_back("Module");
  fieldNames.push_back("QID");
  fieldNames.push_back("Resolution");
  fieldNames.push_back("RobotEvent_ID");
  fieldNames.push_back("Source");

  // Loop through field names to see if they exist with the correct data type
  for (int idx = 0; idx < fieldNames.size(); idx++)
  {
    fieldFlag = eventJson[0].has_string_field(fieldNames[idx]);
    if (fieldFlag == false)
    {
      break;
    }
  }
  ASSERT_TRUE(fieldFlag);
}

TEST(BackEndApiTestSuite, statusTrueTest)
{
  // Push status
  api_instance.push_status(true, json::value::string(telemetry));

  // Get log file
  std::string filename = log_name + "Status" + log_ext;
  std::cout << "Checking: " << filename << std::endl;

  // Check if file exists
  std::ifstream infile(filename);
  bool fileflag = infile.good();
  ASSERT_TRUE(fileflag);

  // Stream to store read JSON
  utility::stringstream_t stream;
  // String to hold single line of read file
  std::string status_line;

  // Read file
  while (getline(infile, status_line))
  {
    // Output the text from the file
    stream << status_line;
  }

  // Parse JSON string
  json::value status_log = json::value::parse(stream);

  // Create key and get value
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  std::string message_value;
  message_value = status_log[msgKey].as_string();

  // Check if message is correct
  ASSERT_EQ(message_value, "Online");

  // Close file
  infile.close();
}

TEST(BackEndApiTestSuite, statusFalseTest)
{
  // Push status
  api_instance.push_status(false, json::value::string(telemetry));

  // Get log file
  std::string filename = log_name + "Status" + log_ext;
  std::cout << "Checking: " << filename << std::endl;

  // Check if file exists
  std::ifstream infile(filename);
  bool fileflag = infile.good();
  ASSERT_TRUE(fileflag);

  // Stream to store read JSON
  utility::stringstream_t stream;
  // String to hold single line of read file
  std::string status_line;

  // Read file
  while (getline(infile, status_line))
  {
    // Output the text from the file
    stream << status_line;
  }

  // Parse JSON string
  json::value status_log = json::value::parse(stream);

  // Create key and get value
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  std::string message_value;
  message_value = status_log[msgKey].as_string();

  // Check if message is correct
  ASSERT_EQ(message_value, "Offline");

  // Close file
  infile.close();
}

TEST(BackEndApiTestSuite, ecsHitTest)
{
  // Sample message
  std::string msgText = "Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00";

  // Check with ecs
  json::value msgInfo;
  msgInfo = api_instance.check_error_classification(msgText);

  // Check if it has the right fields according to schema
  bool fieldFlag = true;
  // List all field names
  std::vector<std::string> fieldNames;
  fieldNames.push_back("error_code");
  fieldNames.push_back("error_level");
  fieldNames.push_back("compounding_flag");
  fieldNames.push_back("error_module");
  fieldNames.push_back("error_source");
  fieldNames.push_back("error_text");
  fieldNames.push_back("error_description");
  fieldNames.push_back("error_resolution");

  // Loop through field names to see if they exist with the correct data type
  for (int idx = 0; idx < fieldNames.size(); idx++)
  {
    if (fieldNames[idx] == "compounding_flag")
    {
      fieldFlag = msgInfo.has_boolean_field(fieldNames[idx]);
      // std::cout << idx << ", " << fieldNames[idx] << ", " << fieldFlag << std::endl;
    }
    else if (fieldNames[idx] == "error_level")
    {
      fieldFlag = msgInfo.has_integer_field(fieldNames[idx]);
      // std::cout << idx << ", " << fieldNames[idx] << ", " << fieldFlag << std::endl;
    }
    else
    {
      fieldFlag = msgInfo.has_string_field(fieldNames[idx]);
      // std::cout << idx << ", " << fieldNames[idx] << ", " << fieldFlag << std::endl;
    }

    if (fieldFlag == false)
    {
      break;
    }
  }
  ASSERT_TRUE(fieldFlag);
}

TEST(BackEndApiTestSuite, ecsMissTest)
{
  // Sample message
  std::string msgText = "Dummy message. Won't be available in ECS.";

  // Check with ecs
  json::value msgInfo;
  msgInfo = api_instance.check_error_classification(msgText);

  // Check if null value is returned
  bool nullFlag = msgInfo.is_null();
  ASSERT_TRUE(nullFlag);
}

int main(int argc, char **argv)
{

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}