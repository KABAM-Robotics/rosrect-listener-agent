#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>
#include <error_resolution_diagnoser/backend_api.h>

using namespace utility;              // Common utilities like string conversions
using namespace web;                  // Common features like URIs.
using namespace web::http;            // Common HTTP functionality
using namespace web::http::client;    // HTTP client features
using namespace concurrency::streams; // Asynchronous streams
using namespace ::pplx;               // PPLX for tasks
using namespace web::json;            // JSON features

// Log file settings
std::string run_id;
std::string parent_dir = std::getenv("USERPROFILE");
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

// Converter for wstring <> string
std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

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
  std::vector<std::wstring> fieldNames;
  fieldNames.push_back(L"Compounding");
  fieldNames.push_back(L"Date/Time");
  fieldNames.push_back(L"Description");
  fieldNames.push_back(L"Level");
  fieldNames.push_back(L"Message");
  fieldNames.push_back(L"Module");
  fieldNames.push_back(L"QID");
  fieldNames.push_back(L"Resolution");
  fieldNames.push_back(L"RobotEvent_ID");
  fieldNames.push_back(L"Source");

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
  api_instance.push_status(true, json::value::string(converter.from_bytes(telemetry)));

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
    stream << converter.from_bytes(status_line);
  }

  // Parse JSON string
  json::value status_log = json::value::parse(stream);

  // Create key and get value
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  std::wstring message_value;
  message_value = status_log[msgKey].as_string();

  // Check if message is correct
  ASSERT_EQ(message_value, L"Online");

  // Close file
  infile.close();
}

TEST(BackEndApiTestSuite, statusFalseTest)
{
  // Push status
  api_instance.push_status(false, json::value::string(converter.from_bytes(telemetry)));

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
    stream << converter.from_bytes(status_line);
  }

  // Parse JSON string
  json::value status_log = json::value::parse(stream);

  // Create key and get value
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  std::wstring message_value;
  message_value = status_log[msgKey].as_string();

  // Check if message is correct
  ASSERT_EQ(message_value, L"Offline");

  // Close file
  infile.close();
}

int main(int argc, char **argv)
{

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}