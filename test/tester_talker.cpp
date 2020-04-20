#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <string>

class TesterTalker{
  
  public:
  TesterTalker(int, char** ); // Constructor to set up listener object
  ~TesterTalker(); // Destructor
  void talk(std::vector<std::string>, std::vector<std::string>, ros::Rate); // Given a message list and a severity list, logs the message with the appropriate method
};

TesterTalker::TesterTalker(int argc, char** argv){
  // Constructor
  // Initialize node
  ros::init(argc, argv, "rosrect_listener_test_node");
}

TesterTalker::~TesterTalker(){
  // Destructor
  
}

void TesterTalker::talk(std::vector<std::string> msg_list, std::vector<std::string> sev_list, ros::Rate looprate){

  for(int idx = 0; idx < msg_list.size(); idx++){
    std::string msg = msg_list[idx];
    std::string sev = sev_list[idx];

    if(sev == "E"){
      ROS_ERROR("%s", msg);            
    }
    else if(sev == "W"){
      ROS_WARN("%s", msg);
    }
    else{
      ROS_INFO("%s", msg); 
    }
    ros::spinOnce();
    looprate.sleep();
  }  

}