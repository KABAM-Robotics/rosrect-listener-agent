#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <ros/console.h>
#include <ros/package.h>

// Log file settings
std::string package_path = ros::package::getPath("rosrect-listener-agent");
std::string log_name = package_path + "/test/logs/logData";
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
    std::cout << "Trying to remove file: " << filename << std::endl;
    // Remove file
    fileRemoveError = remove(filename.c_str());
  }
  log_id = 0;
}

// Utility function to broadcast log messages
void talk(std::vector<std::string> msg_list, std::vector<std::string> sev_list)
{

  ros::Rate looprate(2);

  for (int idx = 0; idx < msg_list.size(); idx++)
  {
    std::string msg = msg_list[idx];
    std::string sev = sev_list[idx];

    if (sev == "E")
    {
      ROS_ERROR("%s", msg.c_str());
    }
    else if (sev == "W")
    {
      ROS_WARN("%s", msg.c_str());
    }
    else
    {
      ROS_INFO("%s", msg.c_str());
    }
    ros::spinOnce();
    looprate.sleep();
  }
}

TEST(ListenerAgentTestSuite, errorSuppressionTest)
{
  // Test message list
  std::vector<std::string> msg_list =
      {
          "Got new plan",
          "Invalid Trajectory 0.000000, 0.000000, -0.400000, cost: -6.000000",
          "Rotation cmd in collision",
          "Error when rotating.",
          "Clearing both costmaps to unstuck robot (1.84m).",
          "Got new plan",
          "Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00",
          "Invalid Trajectory 0.000000, 0.000000, -0.400000, cost: -6.000000",
          "Rotation cmd in collision",
          "Error when rotating.",
          "Rotate recovery behavior started.",
          "Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00",
          "Got new plan",
          "Invalid Trajectory 0.000000, 0.000000, -0.400000, cost: -6.000000",
          "Rotation cmd in collision",
          "Error when rotating.",
          "Aborting because a valid control could not be found. Even after executing all recovery behaviors"};

  std::vector<std::string> sev_list =
      {
          "I", "W", "W", "W", "W", "I",
          "E", "W", "W", "W", "W", "E",
          "I", "W", "W", "W", "E"};

  talk(msg_list, sev_list);

  // Check if log is created
  int expected_logs = 16;
  for (int idx = 0; idx < expected_logs; idx++)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;

    // Check if file exists
    std::ifstream infile1(filename);
    bool fileflag = infile1.good();
    ASSERT_TRUE(fileflag);
  }
}

TEST(ListenerAgentTestSuite, infoSuppressionTest)
{
  // Test message list
  std::vector<std::string> msg_list =
      {
          "Got new plan",
          "Got new plan",
          "Got new plan",
          "Got new plan",
          "Got new plan",
          "Goal reached"};

  std::vector<std::string> sev_list =
      {
          "I", "I", "I", "I", "I", "I"};

  talk(msg_list, sev_list);

  // Check if log is created
  int expected_logs = 2;
  for (int idx = 0; idx < expected_logs; idx++)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;

    // Check if file exists
    std::ifstream infile1(filename);
    bool fileflag = infile1.good();
    ASSERT_TRUE(fileflag);
  }
}

TEST(ListenerAgentTestSuite, warningSuppressionTest)
{
  // Test message list
  std::vector<std::string> msg_list =
      {
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Clearing both costmaps to unstuck robot (3.00m).",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Got new plan",
          "DWA planner failed to produce path.",
          "Clearing both costmaps to unstuck robot (1.84m).",
          "Got new plan",
          "Goal reached"};

  std::vector<std::string> sev_list =
      {
          "I", "W", "I", "W", "I", "W",
          "I", "W", "I", "W", "W", "I",
          "W", "I", "W", "I", "W", "I",
          "W", "I", "W", "W", "I", "I"};

  talk(msg_list, sev_list);

  // Check if log is created
  int expected_logs = 5;
  for (int idx = 0; idx < expected_logs; idx++)
  {
    // Get filename
    log_id++;
    std::string filename = log_name + std::to_string(log_id) + log_ext;

    // Check if file exists
    std::ifstream infile1(filename);
    bool fileflag = infile1.good();
    ASSERT_TRUE(fileflag);
  }
}

int main(int argc, char **argv)
{
  // Cleanup
  logCleanup();

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  bool logger_change = ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  if (logger_change)
  {
    std::cout << "Logger level changed for testing..." << std::endl;
    ros::console::notifyLoggerLevelsChanged();
  }
  else
  {
    ROS_ERROR("Nothing happened...");
  }
  // ROS_INFO("Starting tester...");
  return RUN_ALL_TESTS();
}