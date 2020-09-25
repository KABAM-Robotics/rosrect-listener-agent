#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class TesterTalker {
    
    public:
    void talk_log(std::vector<std::string>, std::vector<std::string>);
    void talk_telemetry();
    ros::Publisher rosout_pub;
    ros::Publisher odom_pub;
    ros::Publisher pose_pub;
};