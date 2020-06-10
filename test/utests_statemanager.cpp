#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <rosrect-listener-agent/state_manager.h>

using namespace web::json;                  // JSON features
using namespace web;                        // Common features like URIs.

// Log file settings
std::string run_id;
std::string parent_dir = std::getenv("HOME");
std::string log_name = parent_dir.append("/.cognicept/agent/logs/unittest_logs") + "/logData";
std::string log_ext = ".json";
int log_id = 0;

// Utility function to clean up log files
void logCleanup()
{
  bool fileRemoveError = false;

  while (!fileRemoveError)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;
    // std::cout << "Trying to remove file: " << filename << std::endl;
    // Remove file
    fileRemoveError = remove(filename.c_str());
  }
  log_id = 0;
}

// Create test object
StateManager state_manager_instance;

// Sample message
std::string errorMessage = "Aborting because a valid control could not be found. Even after executing all recovery behaviors";
std::string warningMessage = "DWA Planner failed to produce path.";
std::string infoMessage = "Got new plan";
std::string infoEndMessage = "Goal reached";

// Sample log
std::vector<std::string> found;

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
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;
  std::cout << "Checking: " << filename << std::endl; 
  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();  
  ASSERT_TRUE(fileflag);

  // Call check_message_ros again with the same message
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg);

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
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg);

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
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg1);

  // Check if log is created
  log_id++;
  std::string filename = log_name + std::to_string(log_id) + log_ext;
  
  // Check if file DOES NOT exist, since for info, the listener waits to push
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();  
  ASSERT_TRUE(fileflag);

  // Change message to an INFO end
  data.msg = infoEndMessage;
  rosgraph_msgs::Log::ConstPtr rosmsg2(new rosgraph_msgs::Log(data));

  // Call check_message_ros
  state_manager_instance.check_message_ros(sampleRobotCode, rosmsg2);

  // Check if file exists, since for info end, the state manager pushes
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
  ASSERT_EQ(found[0],"");
}

// TEST(StateManagerTestSuite, checkMessageDBErrorTest)
// {
//   // Sample error message
//   std::string sampleRobotCode = "SampleRobotCode";
//   rosgraph_msgs::Log data;
//   data.level = 8;
//   data.name = "/move_base";
//   data.msg = errorMessage;
//   rosgraph_msgs::Log::ConstPtr rosmsg(new rosgraph_msgs::Log(data));

//   // Call check_message_db
//   state_manager_instance.check_message_db(sampleRobotCode, rosmsg);

//   // Check if log is created
//   log_id++;
//   std::string filename = log_name + std::to_string(log_id) + log_ext;
  
//   // Check if file exists
//   std::ifstream infile1(filename);
//   bool fileflag = infile1.good();  
//   ASSERT_TRUE(fileflag);

//   // Call check_message_db again with the same message
//   state_manager_instance.check_message_db(sampleRobotCode, rosmsg);

//   // Check if log is created
//   log_id++;
//   filename = log_name + std::to_string(log_id) + log_ext;
  
//   // Check if file exists
//   std::ifstream infile2(filename);
//   fileflag = infile2.good();  
//   ASSERT_TRUE(fileflag);

//   // Clear state manager
//   state_manager_instance.clear();
// }

int main(int argc, char **argv)
{
  // Start tests
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}