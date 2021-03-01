#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>
#include <error_resolution_diagnoser/state_manager.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

// Log file settings
std::string run_id;
std::string parent_dir = std::getenv("USERPROFILE");
std::string log_name = parent_dir.append("/.cognicept/agent/logs/unittest_logs") + "/logData";
std::string log_ext = ".json";
int log_id = 0;

// Create test object
StateManager state_manager_instance;

// Sample message
std::string errorMessage = "Aborting because a valid control could not be found. Even after executing all recovery behaviors";
std::string warningMessage = "DWA Planner failed to produce path.";
std::string infoMessage = "Got new plan";
std::string infoEndMessage = "Goal reached";

// Sample telemetry
json::value telemetry = json::value::parse(L"{ \"pose\" : 42 }");

// Sample log
std::vector<std::string> found;

// Utility function to clean up log files
void logCleanup()
{
  bool fileRemoveError = false;
  int file_id = 1;

  while (!fileRemoveError)
  {
    // Get filename
    std::string filename = log_name + std::to_string(file_id) + log_ext;
    // Remove file
    fileRemoveError = remove(filename.c_str());
    // std::cout << "Trying to remove file: " << filename << ": " << fileRemoveError << std::endl;
    file_id++;
  }
  log_id = 0;
}

TEST(StateManagerTestSuite, existTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is sample text";

  // Check if message exists
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkErrorTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_warning
  state_manager_instance.check_error(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleMsgText);

  // Call check_warning again to see if suppression works. This should not add a new row.
  state_manager_instance.check_error(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkWarningTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_warning
  state_manager_instance.check_warning(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleMsgText);

  // Call check_warning again to see if suppression works. This should not add a new row.
  state_manager_instance.check_warning(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkInfoTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_warning
  state_manager_instance.check_info(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleMsgText);

  // Call check_warning again to see if suppression works. This should not add a new row.
  state_manager_instance.check_info(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkMessageROSErrorTest)
{
  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = "/move_base";
  data.msg = errorMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;
  std::cout << "Checking: " << filename << std::endl;
  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Call check_message_ros again with the same message
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkMessageROSWarningTest)
{
  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 4;
  data.name = "/move_base";
  data.msg = warningMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkMessageROSInfoTest)
{
  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 2;
  data.name = "/move_base";
  data.msg = infoMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg1(new rosgraph_msgs::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg1, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Change message to an INFO end
  data.msg = infoEndMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg2(new rosgraph_msgs::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg2, telemetry);

  // Check if file exists
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, clearTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleMsgText = "This is a sample error text";

  // Call check_error
  state_manager_instance.check_error(sampleRobotCode, sampleMsgText);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);
  int sizeFirstCheck = found.size();

  // Check if size is 1
  ASSERT_EQ(sizeFirstCheck, 3);

  // Clear state manager
  state_manager_instance.clear();

  // See if NO message exists in internal data structure
  found = state_manager_instance.does_exist(sampleRobotCode, sampleMsgText);

  // Check if result is empty
  ASSERT_EQ(found[0], "");
}

TEST(StateManagerTestSuite, checkMessageERTErrorTest)
{
  // Clean up logs
  logCleanup();

  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = "/move_base";
  data.msg = errorMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // Set mode to ERT
  char agent_type[50] = "AGENT_TYPE=ERT";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Turtlebot3";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_ert;

  // Call check_message_ert
  sm_ert.check_message_ert(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Call check_message_ert again with the same message
  sm_ert.check_message_ert(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  sm_ert.clear();
}

TEST(StateManagerTestSuite, checkMessageERTWarningTest)
{
  // Clean up logs
  logCleanup();

  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 4;
  data.name = "/move_base";
  data.msg = warningMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // Set mode to ERT
  char agent_type[50] = "AGENT_TYPE=ERT";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Turtlebot3";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_ert;

  // Call check_message_ert
  sm_ert.check_message_ert(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  sm_ert.clear();
}

TEST(StateManagerTestSuite, checkMessageERTInfoTest)
{
  // Clean up logs
  logCleanup();

  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 2;
  data.name = "/move_base";
  data.msg = infoMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg1(new rosgraph_msgs::Log(data));

  // Set mode to ERT
  char agent_type[50] = "AGENT_TYPE=ERT";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Turtlebot3";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_ert;

  // Call check_message_ert
  sm_ert.check_message_ert(sampleRobotCode, rosmsg1, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Change message to an INFO end
  data.msg = infoEndMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg2(new rosgraph_msgs::Log(data));

  // Call check_message_ert
  sm_ert.check_message_ert(sampleRobotCode, rosmsg2, telemetry);

  // Check if file exists
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  sm_ert.clear();
}

TEST(StateManagerTestSuite, checkMessageECSErrorTest)
{
  // Clean up logs
  logCleanup();

  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 8;
  data.name = "/move_base";
  data.msg = errorMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // Set mode to ECS
  char agent_type[50] = "AGENT_TYPE=ECS";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Turtlebot3";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_ecs;

  // Call check_message_ecs
  sm_ecs.check_message_ecs(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Call check_message_ecs again with the same message
  sm_ecs.check_message_ecs(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  sm_ecs.clear();
}

TEST(StateManagerTestSuite, checkMessageECSWarningTest)
{
  // Clean up logs
  logCleanup();

  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 4;
  data.name = "/move_base";
  data.msg = warningMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

  // Set mode to ECS
  char agent_type[50] = "AGENT_TYPE=ECS";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Turtlebot3";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_ecs;

  // Call check_message_ecs
  sm_ecs.check_message_ecs(sampleRobotCode, rosmsg, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  sm_ecs.clear();
}

TEST(StateManagerTestSuite, checkMessageECSInfoTest)
{
  // Clean up logs
  logCleanup();

  // Sample warning message
  std::string sampleRobotCode = "SampleRobotCode";
  rosgraph_msgs::Log data;
  data.level = 2;
  data.name = "/move_base";
  data.msg = infoMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg1(new rosgraph_msgs::Log(data));

  // Set mode to ECS
  char agent_type[50] = "AGENT_TYPE=ECS";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Turtlebot3";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_ecs;

  // Call check_message_ecs
  sm_ecs.check_message_ecs(sampleRobotCode, rosmsg1, telemetry);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;

  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Change message to an INFO end
  data.msg = infoEndMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg2(new rosgraph_msgs::Log(data));

  // Call check_message_ecs
  sm_ecs.check_message_ecs(sampleRobotCode, rosmsg2, telemetry);

  // Check if file exists
  log_id++;
  filename = log_name + std::to_string(log_id) + log_ext;
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Clear state manager
  sm_ecs.clear();
}

TEST(StateManagerTestSuite, diagExistTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleDiagStr = "/test1";
  std::string sampleLvl = "2";

  // Check if message exists
  found = state_manager_instance.does_diag_exist(sampleRobotCode, sampleDiagStr, sampleLvl);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkDiagTest)
{
  // Sample message
  std::string sampleRobotCode = "SampleRobotCode";
  std::string sampleDiagStr = "/test1";
  std::string sampleLvl = "2";

  // Check if message exists
  found = state_manager_instance.does_diag_exist(sampleRobotCode, sampleDiagStr, sampleLvl);

  // Expecting a null return since no messages are seen before this
  ASSERT_TRUE(found[0].empty());

  // Call check_diag_data
  state_manager_instance.check_diag_data(sampleRobotCode, sampleDiagStr, sampleLvl);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_diag_exist(sampleRobotCode, sampleDiagStr, sampleLvl);
  int sizeFirstCheck = found.size();

  // Expecting first element of internal data structure to be robot code and next to be sample message
  ASSERT_EQ(found[0], sampleRobotCode);
  ASSERT_EQ(found[1], sampleDiagStr);
  ASSERT_EQ(found[2], sampleLvl);

  // Call check_diag_data again to see if suppression works. This should not add a new row.
  state_manager_instance.check_diag_data(sampleRobotCode, sampleDiagStr, sampleLvl);

  // See if error message exists in internal data structure
  found = state_manager_instance.does_diag_exist(sampleRobotCode, sampleDiagStr, sampleLvl);
  int sizeSecondCheck = found.size();

  // Check if size is the same
  ASSERT_EQ(sizeFirstCheck, sizeSecondCheck);

  // Clear state manager
  state_manager_instance.clear();
}

TEST(StateManagerTestSuite, checkDiagROSTest)
{
  // Clean up logs
  logCleanup();

  // Sample vector of diagnostics
  std::vector<diagnostic_msgs::DiagnosticStatus> diag_array;

  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  diagnostic_msgs::DiagnosticStatus data;
  data.level = data.ERROR;
  data.name = "/test1";
  data.message = "Test Error Message";
  diag_array.push_back(data);

  // Sample warn message
  data.level = data.WARN;
  data.name = "/test2";
  data.message = "Test Warn Message";
  diag_array.push_back(data);

  // Create new state manager instance
  StateManager sm_diag_ros;

  // Call check_diagnostic_ros
  sm_diag_ros.check_diagnostic_ros(sampleRobotCode, diag_array, telemetry);

  // Check if logs are created
  int expected_logs = 2;
  for (int idx = 0; idx < expected_logs; idx++)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;
    std::cout << "Checking: " << filename << std::endl;
    // Check if file exists
    std::ifstream infile1(filename);
    bool fileflag = infile1.good();
    ASSERT_TRUE(fileflag);
  }

  // Clear state manager
  sm_diag_ros.clear();
}

TEST(StateManagerTestSuite, checkDiagERTTest)
{
  // Clean up logs
  logCleanup();

  // Sample vector of diagnostics
  std::vector<diagnostic_msgs::DiagnosticStatus> diag_array;

  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  diagnostic_msgs::DiagnosticStatus data;
  data.level = data.ERROR;
  data.name = "/Other/rslidar_node: rslidar_packets topic status";
  data.message = "No data received";
  diag_array.push_back(data);

  // Sample warn message
  data.level = data.WARN;
  data.name = "/Other/camera realsense2_camera_manager_color: Frequency Status";
  data.message = "Frequency too low";
  diag_array.push_back(data);

  // Set mode to ERT
  char agent_type[50] = "AGENT_TYPE=ERT";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Jeff";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_diag_ert;

  // Call check_diagnostic_ros
  sm_diag_ert.check_diagnostic_ert(sampleRobotCode, diag_array, telemetry);

  // Check if logs are created
  int expected_logs = 2;
  for (int idx = 0; idx < expected_logs; idx++)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;
    std::cout << "Checking: " << filename << std::endl;
    // Check if file exists
    std::ifstream infile1(filename);
    bool fileflag = infile1.good();
    ASSERT_TRUE(fileflag);
  }

  // Clear state manager
  sm_diag_ert.clear();
}

TEST(StateManagerTestSuite, checkDiagECSTest)
{
  // Clean up logs
  logCleanup();

  // Sample vector of diagnostics
  std::vector<diagnostic_msgs::DiagnosticStatus> diag_array;

  // Sample error message
  std::string sampleRobotCode = "SampleRobotCode";
  diagnostic_msgs::DiagnosticStatus data;
  data.level = data.ERROR;
  data.name = "/Other/rslidar_node: rslidar_packets topic status";
  data.message = "No data received";
  diag_array.push_back(data);

  // Sample warn message
  data.level = data.WARN;
  data.name = "/Other/camera realsense2_camera_manager_color: Frequency Status";
  data.message = "Frequency too low";
  diag_array.push_back(data);

  // Set mode to ERT
  char agent_type[50] = "AGENT_TYPE=ECS";
  char ecs_api[200] = "ECS_API=http://localhost:8000";
  char ecs_robot_model[200] = "ECS_ROBOT_MODEL=Jeff";
  putenv(agent_type);
  putenv(ecs_api);
  putenv(ecs_robot_model);

  // Create new state manager instance
  StateManager sm_diag_ecs;

  // Call check_diagnostic_ros
  sm_diag_ecs.check_diagnostic_ecs(sampleRobotCode, diag_array, telemetry);

  // Check if logs are created
  int expected_logs = 2;
  for (int idx = 0; idx < expected_logs; idx++)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;
    std::cout << "Checking: " << filename << std::endl;
    // Check if file exists
    std::ifstream infile1(filename);
    bool fileflag = infile1.good();
    ASSERT_TRUE(fileflag);
  }

  // Clear state manager
  sm_diag_ecs.clear();
}

int main(int argc, char **argv)
{
  // Start tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}