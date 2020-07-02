#include <rosrect-listener-agent/listener_agent.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

cs_listener::cs_listener()
{
  // Constructor

  // Pulling environment variables
  // AGENT_TYPE
  if(std::getenv("AGENT_TYPE"))
  {
    // Success case
    this->agent_type = std::getenv("AGENT_TYPE");
    // See if configuration is correct otherwise default to ROS
    if((this->agent_type == "DB") || (this->agent_type == "ERT") || (this->agent_type == "ECS"))
    {
      if(std::getenv("ECS_API"))
      {
        // Success case
        if(std::getenv("ECS_ROBOT_MODEL"))
        {
          // Success case
        }
        else
        {
          // Failure case - Default
          this->agent_type = "ROS";
        } 
      }
      else
      {
        // Failure case - Default
        this->agent_type = "ROS";
      }     
    }
  }
  else
  {
    // Failure case - Default
    this->agent_type = "ROS";
  }

  // ROBOT_CODE
  if(std::getenv("ROBOT_CODE"))
  {
    // Success case
    this->robot_code = std::getenv("ROBOT_CODE");
  }
  else
  {
    // Failure case - Default
    this->robot_code = "Undefined";
  }

  // Depending on ENV variable, communicate to user
  if ((this->agent_type == "DB") || (this->agent_type == "ERT"))
  {
    std::cout << "Subscribed to Listener Agent with ERT Access..." << std::endl;
  }
  else if (this->agent_type == "ECS")
  {
    std::cout << "Subscribed to Listener Agent with ECS Access..." << std::endl;
  }
  else
  {
    std::cout << "Subscribed to Listener Agent with direct rosout..." << std::endl;
  }

  // Heartbeat parameters
  this->heartrate = ros::WallDuration(15.0);

  // Telemetry
  // Create JSON object
  this->telemetry = json::value::object();
}

cs_listener::~cs_listener()
{
  // Stop heartbeat
  this->heartbeat_stop();
  // Destructor
  std::cout << "Unsubscribed from Listener Agent..." << std::endl;
}

json::value cs_listener::odom_to_json(const nav_msgs::Odometry::ConstPtr &rosmsg)
{
  // Create JSON objects
  json::value odom_json = json::value::object();
  json::value orientation = json::value::object();
  json::value position = json::value::object();

  // Create keys
  utility::string_t oKey(U("orientation"));
  utility::string_t pKey(U("position"));
  utility::string_t wKey(U("w"));
  utility::string_t xKey(U("x"));
  utility::string_t yKey(U("y"));
  utility::string_t zKey(U("z"));

  // Assign orientation key-value
  orientation[wKey] = json::value::number(U(rosmsg->pose.pose.orientation.w));
  orientation[xKey] = json::value::number(U(rosmsg->pose.pose.orientation.x));
  orientation[yKey] = json::value::number(U(rosmsg->pose.pose.orientation.y));
  orientation[zKey] = json::value::number(U(rosmsg->pose.pose.orientation.z));

  // Assign position key-value
  position[xKey] = json::value::number(U(rosmsg->pose.pose.position.x));
  position[yKey] = json::value::number(U(rosmsg->pose.pose.position.y));
  position[zKey] = json::value::number(U(rosmsg->pose.pose.position.z));

  // Assign odom key-value
  odom_json[oKey] = orientation;
  odom_json[pKey] = position;

  return (odom_json);
}

json::value cs_listener::pose_to_json(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &rosmsg)
{
  // Create JSON objects
  json::value pose_json = json::value::object();
  json::value orientation = json::value::object();
  json::value position = json::value::object();

  // Create keys
  utility::string_t oKey(U("orientation"));
  utility::string_t pKey(U("position"));
  utility::string_t wKey(U("w"));
  utility::string_t xKey(U("x"));
  utility::string_t yKey(U("y"));
  utility::string_t zKey(U("z"));

  // Assign orientation key-value
  orientation[wKey] = json::value::number(U(rosmsg->pose.pose.orientation.w));
  orientation[xKey] = json::value::number(U(rosmsg->pose.pose.orientation.x));
  orientation[yKey] = json::value::number(U(rosmsg->pose.pose.orientation.y));
  orientation[zKey] = json::value::number(U(rosmsg->pose.pose.orientation.z));

  // Assign position key-value
  position[xKey] = json::value::number(U(rosmsg->pose.pose.position.x));
  position[yKey] = json::value::number(U(rosmsg->pose.pose.position.y));
  position[zKey] = json::value::number(U(rosmsg->pose.pose.position.z));

  // Assign odom key-value
  pose_json[oKey] = orientation;
  pose_json[pKey] = position;

  return (pose_json);
}

void cs_listener::setup_telemetry(ros::NodeHandle nh)
{
  // Get all topics
  ros::master::V_TopicInfo all_topics;
  ros::master::getTopics(all_topics);

  // Find topics relevant to telemetry info and dynamically subscribe
  std::string odom_msg_type = "nav_msgs/Odometry";
  std::string pose_msg_type = "geometry_msgs/PoseWithCovarianceStamped";
  std::string odom_topic;
  std::string pose_topic;

  for (ros::master::V_TopicInfo::iterator it = all_topics.begin(); it != all_topics.end(); it++)
  {
    const ros::master::TopicInfo &info = *it;
    if (info.datatype == odom_msg_type)
    {
      // If odom type is found, subscribe
      odom_topic = info.name;
      std::cout << "Odom topic found! Subscribing to " << info.name << " for telemetry." << std::endl;
      this->odom_sub =
          nh.subscribe(odom_topic, 1000, &cs_listener::odom_callback, this);
    }
    else if ((info.datatype == pose_msg_type) && (info.name != "/initialpose"))
    {
      // If pose type is found, subscribe
      pose_topic = info.name;
      std::cout << "Pose topic found! Subscribing to " << info.name << " for telemetry." << std::endl;
      this->pose_sub =
          nh.subscribe(pose_topic, 1000, &cs_listener::pose_callback, this);
    }
  }

  // If subscribers are empty, prompt appropriately
  if ((this->odom_sub.getTopic().empty()) && (this->pose_sub.getTopic().empty()))
  {
    std::cout << "No relevant topics found for telemetry. It will be null. Consider starting the robot nodes first and relaunch this agent." << std::endl;
  }
}

void cs_listener::log_callback(const rosgraph_msgs::Log::ConstPtr &rosmsg)
{

  // To debug this callback function
  std::cout << "Message received: " << rosmsg->msg << std::endl;

  // Callback that hands over message to State Manager
  this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg, this->telemetry);
}

void cs_listener::odom_callback(const nav_msgs::Odometry::ConstPtr &rosmsg)
{
  // Process odometry information for telemetry
  // std::cout << "Odom callback called" << std::endl;

  // Convert to JSON
  json::value odom_data = this->odom_to_json(rosmsg);

  // Create key
  utility::string_t odomKey(U("odom_pose"));

  // Assign key-value
  this->telemetry[odomKey] = odom_data;
}

void cs_listener::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &rosmsg)
{
  // Process pose information for telemetry
  // std::cout << "Pose callback called" << std::endl;

  // Convert to JSON
  json::value pose_data = this->pose_to_json(rosmsg);

  // Create key
  utility::string_t poseKey(U("nav_pose"));

  // Assign key-value
  this->telemetry[poseKey] = pose_data;
}

void cs_listener::heartbeat_start(ros::NodeHandle nh)
{
  // Records heartbeat online status when node is started. Future status is pushed by timer bound callback
  this->state_manager_instance.check_heartbeat(true, this->telemetry);

  // Create a Wall Timer for heartrate period
  this->heartbeat_timer = nh.createWallTimer(this->heartrate, &cs_listener::heartbeat_log, this);
}

void cs_listener::heartbeat_log(const ros::WallTimerEvent &timer_event)
{
  // A timer bound method that periodically checks the ROS connection status and passes it to the state manager.
  bool status = ros::ok;
  this->state_manager_instance.check_heartbeat(status, this->telemetry);
}

void cs_listener::heartbeat_stop()
{
  // Records heartbeat offline status when node is shutdown
  this->state_manager_instance.check_heartbeat(false, this->telemetry);
}

int main(int argc, char **argv)
{

  // Initialize node
  ros::init(argc, argv, "rosrect_listener_agent_node");
  ros::NodeHandle nh;
  ros::Rate looprate(10);

  // Create Agent
  cs_listener cs_agent;

  // Start heartbeat
  cs_agent.heartbeat_start(nh);

  // Setup telemetry
  cs_agent.setup_telemetry(nh);

  // Create /rosout_agg subscriber
  ros::Subscriber rosout_agg_sub =
      nh.subscribe("rosout_agg", 1000, &cs_listener::log_callback, &cs_agent);

  while (ros::ok())
  {
    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
}