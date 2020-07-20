#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <ros/console.h>
#include <rosrect-listener-agent/tester_talker.h>

// Utility function to broadcast log messages
void TesterTalker::talk(std::vector<std::string> msg_list, std::vector<std::string> sev_list)
{
  ros::NodeHandle nh;
  this->pub = nh.advertise<rosgraph_msgs::Log>("rosout_agg", 1);

  ros::Rate looprate(2);
  rosgraph_msgs::Log rosmsg;

  for (int idx = 0; idx < msg_list.size(); idx++)
  {
    std::string msg = msg_list[idx];
    rosmsg.msg = msg;

    std::string sev = sev_list[idx];
    
    if (sev == "E")
    {
      // ROS_ERROR("%s", msg.c_str());
      rosmsg.level = 8;
    }
    else if (sev == "W")
    {
      // ROS_WARN("%s", msg.c_str());
      rosmsg.level = 4;
    }
    else
    {
      // ROS_INFO("%s", msg.c_str());
      rosmsg.level = 2;
    }
    this->pub.publish(rosmsg);
    ros::spinOnce();
    looprate.sleep();
  }
}

// Create Talker object for tests
TesterTalker talker_instance;

// Log file settings
std::string run_id;
std::string parent_dir;
std::string log_name;
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

  talker_instance.talk(msg_list, sev_list);

  // Check if log is created
  int expected_logs = 16;
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

  talker_instance.talk(msg_list, sev_list);

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

  talker_instance.talk(msg_list, sev_list);

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
  // Wait for 5 seconds to run the listener test
  std::cout << "Listener test will wait 5 seconds for other tests to finish..." << std::endl;
  sleep(5);

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  
  // Set log folder 
  ros::param::get("run_id", run_id);
  parent_dir = std::getenv("HOME");
  parent_dir.append("/.cognicept/agent/logs/" + run_id);
  log_name = parent_dir + "/logData";

  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}