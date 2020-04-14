#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <iostream>
#include <rosrect-listener-agent/state_manager.h>

class cs_listener{

    std::string agent_type; // DB or ROS agent
    std::string robot_code; // UUID supplied during setup
    StateManager state_manager_instance; // State manager object that processes all incoming messages

    public:
    cs_listener(); // Constructor to set up listener object
    ~cs_listener(); // Destructor
    void log_callback(const rosgraph_msgs::Log::ConstPtr&); // Listener callback that hands over the rosout message to state manager for processing

};

cs_listener::cs_listener(){
  // Constructor

  this->agent_type = std::getenv("AGENT_TYPE");
  this->robot_code = std::getenv("ROBOT_CODE");
  this->state_manager_instance = StateManager();

  // Depending on ENV variable, communicate to user
  if(this->agent_type == "DB"){
    std::cout << "Subscribed to Cognicept Agent with DB Access..." << std::endl;
  }
  else{
    std::cout << "Subscribed to Cognicept Agent with direct rosout..." << std::endl;
  }
}

cs_listener::~cs_listener(){
  // Destructor
  std::cout << "Unsubscribed from Cognicept Agent..." << std::endl;
}

void cs_listener::log_callback(const rosgraph_msgs::Log::ConstPtr& rosmsg){
  // Callback that hands over message to State Manager
  this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg);

}

int main(int argc, char** argv){

  // Initialize node
  ros::init(argc, argv, "rosrect_listener_agent_node");
  ros::NodeHandle nh;
  ros::Rate looprate(10);

  // Create /rosout_agg subcriber
  cs_listener cs_agent;
  ros::Subscriber sub =
      nh.subscribe("rosout_agg", 1, &cs_listener::log_callback, &cs_agent);

  while(ros::ok()){
    ros::spinOnce();
    looprate.sleep();    
  }
  
  return 0;
}