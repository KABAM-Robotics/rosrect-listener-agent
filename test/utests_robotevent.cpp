#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <locale>
#include <codecvt>
#include <error_resolution_diagnoser/robot_event.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

// Create test object
RobotEvent event_instance;

// Create sample log
std::vector<std::vector<std::string>> sample_log;
std::wstring level = L"8";
std::wstring cflag = L"Null";
std::wstring module = L"Null";
std::wstring source = L"/move_base";
std::wstring message = L"Aborting because a valid control could not be found. Even after executing all recovery behaviors";
std::wstring description = L"Null";
std::wstring resolution = L"Null";
std::wstring telemetry_str = L"{\"pose\":42}";

// Hold the record
std::vector<std::string> event_details;

// Converter for wstring <> string
std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

TEST(RobotEventTestSuite, getLogTest)
{
  // Sample message
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = "/move_base";
  data.msg = converter.to_bytes(message);
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // For simple get test, no need for ECS, telemetry info
  json::value msgInfo = json::value::null();
  json::value telemetry = json::value::null();

  // Declare log variable
  std::vector<std::vector<std::string>> updatedLog;
  std::vector<std::string> currentRow;

  // Update log
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ROS");

  // Check number of elements in event log
  // Get log
  updatedLog = event_instance.get_log();

  // Check if there is only one row
  ASSERT_EQ(updatedLog.size(), 1);

  // Check if there are 10 elements in that one row
  int expElements = 10;
  currentRow = updatedLog[0];
  ASSERT_EQ(currentRow.size(), expElements);

  // Clear event
  event_instance.clear();
}

TEST(RobotEventTestSuite, updateLogROSTest)
{
  // Sample message
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = "/move_base";
  data.msg = converter.to_bytes(message);
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // For ROS test, no need for ECS
  json::value msgInfo = json::value::null();

  // Declare log variable
  std::vector<std::vector<std::string>> updatedLog;
  std::vector<std::string> currentRow;

  // For testing, telemetry is set to a constant
  json::value telemetry = json::value::parse(telemetry_str);

  // Update log
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ROS");

  // Expected log
  // For ROS, cflag is Null
  cflag = L"Null";
  event_details.push_back(converter.to_bytes(level));
  event_details.push_back(converter.to_bytes(cflag));
  event_details.push_back(converter.to_bytes(module));
  event_details.push_back(converter.to_bytes(source));
  event_details.push_back(converter.to_bytes(message));
  event_details.push_back(converter.to_bytes(description));
  event_details.push_back(converter.to_bytes(resolution));
  event_details.push_back(converter.to_bytes(telemetry_str));

  // Get log
  updatedLog = event_instance.get_log();
  currentRow = updatedLog[0];

  // Check if content is equal
  for (int idx = 1; idx < currentRow.size() - 1; idx++)
  {
    ASSERT_EQ(event_details[idx - 1], currentRow[idx]);
  }

  // Clear event
  event_instance.clear();
  // Clear expected log
  event_details.clear();
}

TEST(RobotEventTestSuite, updateLogDBTest)
{
  // Sample message
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = converter.to_bytes(source);
  data.msg = converter.to_bytes(message);
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // For DB test, need for ECS, manually construct an ECS response so we don't rely on ECS connection
  json::value msgInfo = json::value::object();
  // For DB, cflag is NOT Null
  cflag = L"true";
  event_details.push_back(std::to_string(8));
  event_details.push_back(converter.to_bytes(cflag));
  event_details.push_back("Navigation");
  event_details.push_back(converter.to_bytes(source));
  event_details.push_back(converter.to_bytes(message));
  event_details.push_back("The robot is unable to move around. This usually means the robot is mislocalized or there is an obstacle.");
  event_details.push_back("Relocalize the robot using intervention, assign a sample goal. If that does not work, use teleoperation to nudge the robot from the impossible position. If that does not work, escalate to property.");
  event_details.push_back(converter.to_bytes(telemetry_str));

  // Declare log variable
  std::vector<std::vector<std::string>> updatedLog;
  std::vector<std::string> currentRow;

  // Create keys
  utility::string_t codeKey(utility::conversions::to_string_t("error_code"));
  utility::string_t lvlKey(utility::conversions::to_string_t("error_level"));
  utility::string_t cfKey(utility::conversions::to_string_t("compounding_flag"));
  utility::string_t modKey(utility::conversions::to_string_t("error_module"));
  utility::string_t srcKey(utility::conversions::to_string_t("error_source"));
  utility::string_t txtKey(utility::conversions::to_string_t("error_text"));
  utility::string_t descKey(utility::conversions::to_string_t("error_description"));
  utility::string_t resKey(utility::conversions::to_string_t("error_resolution"));

  // Assign key-value
  msgInfo[codeKey] = json::value::string(L"Null");
  msgInfo[lvlKey] = json::value::number(8);
  msgInfo[cfKey] = json::value::boolean(true);
  msgInfo[modKey] = json::value::string(L"Navigation");
  msgInfo[srcKey] = json::value::string(L"/move_base");
  msgInfo[txtKey] = json::value::string(message);
  msgInfo[descKey] = json::value::string(L"The robot is unable to move around. This usually means the robot is mislocalized or there is an obstacle.");
  msgInfo[resKey] = json::value::string(L"Relocalize the robot using intervention, assign a sample goal. If that does not work, use teleoperation to nudge the robot from the impossible position. If that does not work, escalate to property.");

  // For testing, telemetry is set to a constant
  json::value telemetry = json::value::string(telemetry_str);

  // Update log
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ERT");

  // Get log
  updatedLog = event_instance.get_log();
  currentRow = updatedLog[0];

  // Check if content is equal
  for (int idx = 1; idx < currentRow.size() - 2; idx++)
  {
    ASSERT_EQ(event_details[idx - 1], currentRow[idx]);
  }

  // Clear event
  event_instance.clear();
  // Clear expected log
  event_details.clear();
}

TEST(RobotEventTestSuite, updateLogECSTest)
{
  // Sample message
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = converter.to_bytes(source);
  data.msg = converter.to_bytes(message);
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // For DB test, need for ECS, manually construct an ECS response so we don't rely on ECS connection
  json::value msgInfo = json::value::object();
  // For DB, cflag is NOT Null
  cflag = L"false";
  event_details.push_back(std::to_string(16));
  event_details.push_back(converter.to_bytes(cflag));
  event_details.push_back("Navigation");
  event_details.push_back(converter.to_bytes(source));
  event_details.push_back(converter.to_bytes(message));
  event_details.push_back(converter.to_bytes(message));
  event_details.push_back("Null");
  event_details.push_back(converter.to_bytes(telemetry_str));

  // Declare log variable
  std::vector<std::vector<std::string>> updatedLog;
  std::vector<std::string> currentRow;

  // Create keys
  utility::string_t codeKey(utility::conversions::to_string_t("cognicept_error_code"));
  utility::string_t lvlKey(utility::conversions::to_string_t("severity"));
  utility::string_t cfKey(utility::conversions::to_string_t("compounding_flag"));
  utility::string_t modKey(utility::conversions::to_string_t("error_module"));
  utility::string_t srcKey(utility::conversions::to_string_t("error_source"));
  utility::string_t txtKey(utility::conversions::to_string_t("error_text"));

  // Assign key-value
  msgInfo[codeKey] = json::value::string(L"NAV-SW-16-R-2");
  msgInfo[lvlKey] = json::value::number(16);
  msgInfo[cfKey] = json::value::boolean(false);
  msgInfo[modKey] = json::value::string(L"Navigation");
  msgInfo[srcKey] = json::value::string(L"/move_base");
  msgInfo[txtKey] = json::value::string(message);
  
  // For testing, telemetry is set to a constant
  json::value telemetry = json::value::string(telemetry_str);

  // Update log
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ECS");

  // Get log
  updatedLog = event_instance.get_log();
  currentRow = updatedLog[0];

  // Check if content is equal
  for (int idx = 1; idx < currentRow.size() - 2; idx++)
  {
    ASSERT_EQ(event_details[idx-1], currentRow[idx]);
  }

  // Clear event
  event_instance.clear();
  // Clear expected log
  event_details.clear();
}

TEST(RobotEventTestSuite, updateEventIdTest)
{
  // Sample message
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = converter.to_bytes(source);
  data.msg = converter.to_bytes(message);
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // For simple get test, no need for ECS
  json::value msgInfo = json::value::null();

  // For testing, telemetry can be null
  json::value telemetry = json::value::null();

  // Declare log variable
  std::vector<std::vector<std::string>> updatedLog;
  std::vector<std::string> currentRow;

  // Update log
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ROS");

  // Get log
  updatedLog = event_instance.get_log();

  // Get event id value
  currentRow = updatedLog[0];
  std::string eventId1 = currentRow.back();

  // Clear LOG and Update log again, this should result in same event id
  event_instance.clear_log();
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ROS");

  // Get log
  updatedLog = event_instance.get_log();

  // Get event id value
  currentRow = updatedLog[0];
  std::string eventId2 = currentRow.back();

  // Clear EVENT and Update log again, this should result in a different event id
  event_instance.clear();
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ROS");

  // Get log
  updatedLog = event_instance.get_log();

  // Get event id value
  currentRow = updatedLog[0];
  std::string eventId3 = currentRow.back();

  // Check if even ids 1 and 2 are equal
  ASSERT_EQ(eventId1, eventId2);

  // Check if even ids 2 and 3 are NOT equal
  ASSERT_NE(eventId2, eventId3);

  // Clear event
  event_instance.clear();
}

TEST(RobotEventTestSuite, clearTest)
{
  // Sample message
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = "/move_base";
  data.msg = converter.to_bytes(message);
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // For simple get test, no need for ECS
  json::value msgInfo = json::value::null();

  // Declare log variable
  std::vector<std::vector<std::string>> updatedLog;
  std::vector<std::string> currentRow;

  // For testing, telemetry can be null
  json::value telemetry = json::value::null();

  // Update log
  event_instance.update_log(rosmsg, msgInfo, telemetry, "ROS");

  // Get log
  updatedLog = event_instance.get_log();

  // Check log emptiness
  bool beforeClearEmptyFlag = updatedLog.empty();

  // Clear LOG
  event_instance.clear_log();

  // Get log
  updatedLog = event_instance.get_log();

  // Check log emptiness
  bool afterClearEmptyFlag = updatedLog.empty();

  // Check if before clearing, log is NOT empty
  ASSERT_FALSE(beforeClearEmptyFlag);

  // Check if after clearing, log is empty
  ASSERT_TRUE(afterClearEmptyFlag);

  // Clear event
  event_instance.clear();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}