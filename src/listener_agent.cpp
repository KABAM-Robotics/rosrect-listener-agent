#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <iostream>
#include <rosrect-listener-agent/backend_api.h>

class cs_listener{

    public:

    // Instantiate BackendApi class object. This will automatically login to Cognicept's 
    // incident manangement API and store access tokens for creating tickets.
    BackendApi api_instance;

    ticketDetails ticket_info;

    void log_callback(const rosgraph_msgs::Log::ConstPtr& rosmsg){

        // Using cout intentionally to not introduce noise to log from this node
        int msg_level;
        msg_level = static_cast<int>(rosmsg->level);

        // Support only navigation errors to start similar to roslibpy agent        
        if(~(rosmsg->name.compare(std::string{"/move_base"}))){

          // Start with only ticket creation on errors. Can include notifications for warnings later
          if(msg_level == 8){
            // std::cout << rosmsg->name << " reporting error: " << rosmsg->msg << std::endl;
            // std::cout << "Creating a ticket!" << std::endl;
            
            ticket_info.module_name = rosmsg->name;
            ticket_info.error_text = rosmsg->msg;
            ticket_info.robot_id = std::getenv("ROBOT_CODE");
            ticket_info.prop_id = std::getenv("SITE_CODE");
            ticket_info.agent_id = std::getenv("AGENT_CODE");

            this->api_instance.create_ticket(ticket_info).wait();
          }

        }
    }
};

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