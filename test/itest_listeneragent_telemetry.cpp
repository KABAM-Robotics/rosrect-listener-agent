#include <cpprest/filestream.h>
#include <cpprest/json.h>
#include <cpprest/containerstream.h>
#include <cpprest/producerconsumerstream.h>
#undef U
#include <gtest/gtest.h>
#include <fstream>
#include <ros/console.h>
#include <boost/date_time.hpp>
#include <rosrect-listener-agent/tester_talker.h>

using namespace web;       // Common features like URIs.
using namespace web::json; // JSON features

// Utility function to broadcast telemetry messages
void TesterTalker::talk_telemetry()
{
  std::cout << "Publishing sample telemetry information..." << std::endl;

  // Create publishers
  ros::NodeHandle nh;
  this->odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  this->pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);
  ros::Rate looprate(1);

  // Create a sample message
  nav_msgs::Odometry odom_msg;
  geometry_msgs::PoseWithCovarianceStamped pose_msg;

  // Set static value for messages
  odom_msg.pose.pose.position.x = 1.0;
  odom_msg.pose.pose.position.y = 1.0;
  odom_msg.pose.pose.position.z = 1.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  odom_msg.pose.pose.orientation.x = 1.0;
  odom_msg.pose.pose.orientation.y = 1.0;
  odom_msg.pose.pose.orientation.z = 1.0;

  pose_msg.pose.pose.position.x = 2.0;
  pose_msg.pose.pose.position.y = 2.0;
  pose_msg.pose.pose.position.z = 2.0;
  pose_msg.pose.pose.orientation.w = 2.0;
  pose_msg.pose.pose.orientation.x = 2.0;
  pose_msg.pose.pose.orientation.y = 2.0;
  pose_msg.pose.pose.orientation.z = 2.0;

  // Loop through messages
  for (int idx = 0; idx < 20; idx++)
  {
    // Set dynamic message values
    odom_msg.pose.pose.position.x = float(idx);
    odom_msg.pose.pose.orientation.x = float(idx);
  
    pose_msg.pose.pose.position.x = float(idx);
    pose_msg.pose.pose.orientation.x = float(idx);

    // Publish messages
    this->odom_pub.publish(odom_msg);
    this->pose_pub.publish(pose_msg);

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

TEST(ListenerAgentTestSuite, telemetryTest)
{
  // Go through the lifecycle of a heartbeat to test telemetry

  // When agent node starts, heartbeat is generated and includes no telemetry 
  // since those topics have not started publishing
  std::cout << "Starting pre-telemetry checks..." << std::endl;
  // Check if log is created
  std::string filename = log_name + log_ext;
  std::cout << "Checking: " << filename << std::endl;
  // Check if file exists
  std::ifstream infile1(filename);
  bool fileflag = infile1.good();
  ASSERT_TRUE(fileflag);

  // Stream to store read JSON
  utility::stringstream_t stream;
  // String to hold single line of read file
  std::string status_line;

  // Read file
  while (getline(infile1, status_line))
  {
    // Output the text from the file
    stream << status_line;
    std::cout << stream.str() << std::endl;
  }

  // Close file
  infile1.close();

  // Parse JSON string
  json::value status_log = json::value::parse(stream);

  // Create key and get value
  utility::string_t msgKey(utility::conversions::to_string_t("message"));
  utility::string_t tktKey(utility::conversions::to_string_t("create_ticket"));
  utility::string_t teleKey(utility::conversions::to_string_t("telemetry"));
  utility::string_t tmKey(utility::conversions::to_string_t("timestamp"));
  std::string message_value;
  bool ticket_value;
  json::value telemetry_value;
  std::string timestamp_value_str;

  message_value = status_log[msgKey].as_string();
  ticket_value = status_log[tktKey].as_bool();
  telemetry_value = status_log[teleKey];
  timestamp_value_str = status_log[tmKey].as_string();
  boost::posix_time::ptime timestamp1 = boost::posix_time::from_iso_extended_string(timestamp_value_str);

  // Check if message is correct
  ASSERT_EQ(message_value, "Online");

  // Check if create ticket is correctly set to False
  ASSERT_EQ(ticket_value, false);

  // Check if telemetry is null
  std::vector<utility::string_t> expected_keys;
  expected_keys.push_back(utility::conversions::to_string_t("nav_pose"));
  expected_keys.push_back(utility::conversions::to_string_t("odom_pose"));
  bool fieldFlag = true;
  for (const auto &value : expected_keys)
  {
    // If field exists, test fails since telemetry is not publishing yet
    if(telemetry_value.has_field(value))
    {
      fieldFlag = false;
    }
  }
  ASSERT_EQ(fieldFlag, true);
  
  // Start publishing telemetry topics and check again.
  // This is also a good time to check timestamps.
  std::cout << "Starting telemetry checks. This will take around 20 seconds..." << std::endl;

  // Send messages for at least 15 seconds so new heartbeat is guaranteed to generate.
  // This will include the telemetry info.
  talker_instance.talk_telemetry();

  // Check if log is created
  std::cout << "Checking: " << filename << std::endl;
  // Check if file exists
  std::ifstream infile2(filename);
  fileflag = infile2.good();
  ASSERT_TRUE(fileflag);

  // Stream to store read JSON
  stream.clear();

  // Read file
  while (getline(infile2, status_line))
  {
    // Output the text from the file
    stream << status_line;
    std::cout << stream.str() << std::endl;
  }

  // Close file
  infile2.close();

  // Parse JSON string
  status_log = json::value::parse(stream);

  // Get value
  message_value = status_log[msgKey].as_string();
  ticket_value = status_log[tktKey].as_bool();
  telemetry_value = status_log[teleKey];
  timestamp_value_str = status_log[tmKey].as_string();
  boost::posix_time::ptime timestamp2 = boost::posix_time::from_iso_extended_string(timestamp_value_str);

  // Check if message is correct
  ASSERT_EQ(message_value, "Online");

  // Check if create ticket is correctly set to False
  ASSERT_EQ(ticket_value, false);

  // Check if telemetry has the right keys
  fieldFlag = true;
  for (const auto &value : expected_keys)
  {
    // If field DOES NOT exist, test fails since telemetry is publishing
    if(!telemetry_value.has_field(value))
    {
      fieldFlag = false;
    }
    std::cout << value << ": " << fieldFlag << std::endl;
  }
  ASSERT_EQ(fieldFlag, true);

  // Check if timestamps are at least 15 seconds apart
  auto duration = timestamp2 - timestamp1;
  auto duration_in_s = duration.seconds();
  ASSERT_GE(duration_in_s, 14.0);
  // ASSERT_NEAR(duration_in_s, 15.0, 1.0);

}

int main(int argc, char **argv)
{
  // Wait for a few seconds to run the listener test
  std::cout << "Listener test will wait a few seconds for other tests to finish..." << std::endl;
  sleep(30);

  // Start tests
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");

  // Set log folder
  ros::param::get("run_id", run_id);
  parent_dir = std::getenv("HOME");
  parent_dir.append("/.cognicept/agent/logs/" + run_id);
  log_name = parent_dir + "/logDataStatus";

  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}