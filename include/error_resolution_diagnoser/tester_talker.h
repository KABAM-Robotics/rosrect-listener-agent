#define _SILENCE_TR1_NAMESPACE_DEPRECATION_WARNING
#define _CRT_SECURE_NO_WARNINGS
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>

class TesterTalker {
    
    public:
    void talk_log(std::vector<std::string>, std::vector<std::string>);
    void talk_telemetry();
    void talk_diagnostics(std::vector<std::string>, std::vector<std::vector<std::string>>);
    ros::Publisher rosout_pub;
    ros::Publisher odom_pub;
    ros::Publisher pose_pub;
    ros::Publisher diag_pub;
};