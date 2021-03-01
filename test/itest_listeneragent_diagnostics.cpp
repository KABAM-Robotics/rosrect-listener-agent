#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING
#define _CRT_SECURE_NO_WARNINGS
#include <cpprest/filestream.h>
#include <cpprest/json.h>
#include <cpprest/containerstream.h>
#include <cpprest/producerconsumerstream.h>
#undef U
#include <gtest/gtest.h>
#include <fstream>
#include <ros/console.h>
#include <boost/date_time.hpp>
#include <Windows.h>
#include <error_resolution_diagnoser/tester_talker.h>

using namespace web;       // Common features like URIs.
using namespace web::json; // JSON features

// Utility function to broadcast diagnostic messages
void TesterTalker::talk_diagnostics(std::vector<std::string> node_list, std::vector<std::vector<std::string>> sev_list)
{
  std::cout << "Publishing sample diagnostic information..." << std::endl;

  // Create publishers
  ros::NodeHandle nh;
  this->diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics_agg", 1);
  ros::Rate looprate(0.5);

  // Create a sample message
  diagnostic_msgs::DiagnosticArray diag_arr;
  diagnostic_msgs::DiagnosticStatus diag_msg;

  // Loop through messages
  for (int idx = 0; idx < sev_list.size(); idx++)
  {
    auto sev = sev_list[idx];
    for (int jdx = 0; jdx < node_list.size(); jdx++)
    {
      if (sev[jdx] == "E")
      {
        diag_msg.level = diag_msg.ERROR;
      }
      else if (sev[jdx] == "I")
      {
        diag_msg.level = diag_msg.OK;
      }
      else
      {
        diag_msg.level = diag_msg.WARN;
      }

      diag_msg.name = node_list[jdx];
      diag_msg.message = "Test Message";
      std::cout << "Publishing: " << diag_msg.name << " " << sev[jdx] << std::endl;
      diag_arr.status.push_back(diag_msg);
    }

    // Publish messages
    this->diag_pub.publish(diag_arr);
    diag_arr.status.clear();

    // Spin away
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

TEST(ListenerAgentTestSuite, diagnosticsNoStateChangeTest)
{
  // Test message list
  std::vector<std::string> node_list =
      {
          "/test1",
          "/test2",
          "/test3"};

  std::vector<std::vector<std::string>> sev_list;
  sev_list.push_back({"I", "I", "I"});
  sev_list.push_back({"I", "I", "I"});
  sev_list.push_back({"I", "I", "I"});
  sev_list.push_back({"I", "I", "I"});

  talker_instance.talk_diagnostics(node_list, sev_list);

  // Check if log is created
  int expected_logs = 3;
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

TEST(ListenerAgentTestSuite, diagnosticsStateChangeTest)
{
  // Test message list
  std::vector<std::string> node_list =
      {
          "/test1",
          "/test2",
          "/test3"};

  std::vector<std::vector<std::string>> sev_list;
  sev_list.push_back({"I", "I", "I"});
  sev_list.push_back({"I", "I", "I"});
  sev_list.push_back({"I", "W", "I"});
  sev_list.push_back({"I", "W", "E"});
  sev_list.push_back({"I", "W", "E"});
  sev_list.push_back({"W", "E", "E"});
  sev_list.push_back({"E", "E", "E"});
  sev_list.push_back({"I", "I", "I"});

  talker_instance.talk_diagnostics(node_list, sev_list);

  // Check if log is created
  int expected_logs = 8;
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

int main(int argc, char **argv)
{
  // Wait for a few seconds to run the listener test
  std::cout << "Listener test will wait a few seconds for other tests to finish..." << std::endl;
  Sleep(40000);

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

  // Set log folder
  ros::param::get("run_id", run_id);
  parent_dir = std::getenv("USERPROFILE");
  parent_dir.append("/.cognicept/agent/logs/" + run_id);
  log_name = parent_dir + "/logData";

  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}