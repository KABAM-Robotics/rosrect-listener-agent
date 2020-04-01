#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <iostream>

class cs_listener{

    public:

    void log_callback(const rosgraph_msgs::Log::ConstPtr& rosmsg){

        // Using cout intentionally to not introduce noise to log from this node
        int msg_level;
        msg_level = static_cast<int>(rosmsg->level);

        // Support only navigation errors to start similar to roslibpy agent        
        if(~(rosmsg->name.compare(std::string{"/move_base"}))){

          // Start with only ticket creation on errors. Can include notifications for warnings later
          if(msg_level == 8){
            std::cout << rosmsg->name << " reporting error: " << rosmsg->msg << std::endl;
            std::cout << "Create a ticket!" << std::endl;
          }

        }
    }
};

int main(int argc, char** argv){

  // Initialize node
  ros::init(argc, argv, "cognicept_rosout_listener_node");
  ros::NodeHandle nh;
//   ros::Rate looprate(2);

  // Create /rosout_agg subcriber
  cs_listener cs_agent;
  ros::Subscriber sub =
      nh.subscribe("rosout_agg", 1, &cs_listener::log_callback, &cs_agent);

  ros::spin();
  
  return 0;
}