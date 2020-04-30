#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

class TesterTalker {
    
    public:
    void talk(std::vector<std::string>, std::vector<std::string>);
    ros::Publisher pub;
};