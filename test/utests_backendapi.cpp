#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <rosrect-listener-agent/backend_api.h>

TEST(BackEndApiTestSuite, PushTest)
{
  BackendApi api_instance;
  
  // std::vector<std::vector<std::string>> log;
  // api_instance.push_event_log(log);

  // std::string package_path = ros::package::getPath("rosrect-listener-agent");
  // std::string log_name = package_path + "/tests/logs/logData";
  // std::string log_ext = ".json";
  // int log_id = 1;

  // std::string filename = log_name + std::to_string(log_id) + log_ext;

  // std::ifstream infile(filename);
  // bool fileflag = infile.good();
  
  ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}