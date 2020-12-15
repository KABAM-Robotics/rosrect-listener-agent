#include <error_resolution_diagnoser/listener_agent.h>

using namespace web::json; // JSON features
using namespace web;       // Common features like URIs.

enum class RobotStatus
{
  INIT,
  RUNNING,
  MASTER_DISCONNECTED
};

cs_listener::cs_listener()
{
  // Constructor

  // Pulling environment variables
  // AGENT_TYPE
  if (std::getenv("AGENT_TYPE"))
  {
    // Success case
    this->agent_type = std::getenv("AGENT_TYPE");
    // See if configuration is correct otherwise default to ROS
    if ((this->agent_type == "DB") || (this->agent_type == "ERT") || (this->agent_type == "ECS"))
    {
      if (std::getenv("ECS_API"))
      {
        // Success case
        if (std::getenv("ECS_ROBOT_MODEL"))
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
  if (std::getenv("ROBOT_CODE"))
  {
    // Success case
    this->robot_code = std::getenv("ROBOT_CODE");
  }
  else
  {
    // Failure case - Default
    this->robot_code = "Undefined";
  }

  // SET node list if specified
  if (std::getenv("LOG_NODE_LIST"))
  {
    // Get env variable
    std::string log_node_list_env = std::getenv("LOG_NODE_LIST");
    // Split it by ; delimiter
    boost::split(this->node_list, log_node_list_env, [](char c) { return c == ';'; });
  }
  else if (std::getenv("LOG_NODE_EX_LIST"))
  {
    // Get env variable
    std::string log_node_ex_list_env = std::getenv("LOG_NODE_EX_LIST");
    // Split it by ; delimiter
    boost::split(this->node_ex_list, log_node_ex_list_env, [](char c) { return c == ';'; });
  }

  // DIAGNOSTICS setting
  if (std::getenv("DIAGNOSTICS"))
  {
    // Success case
    this->diag_setting = std::getenv("DIAGNOSTICS");
    boost::algorithm::to_lower(this->diag_setting);
  }
  else
  {
    // Failure case - Default
    this->diag_setting = "off";
  }

  // Heartbeat parameters
  this->heartrate = ros::WallDuration(15.0);

  // Telemetry
  // Create JSON object
  this->telemetry = json::value::object();

  // Diagnostics
  this->num_diag_samples = 15;
  this->curr_diag_sample = INT_MIN;
}

cs_listener::~cs_listener()
{
  // Stop heartbeat
  this->heartbeat_stop();
  // Destructor
  std::cout << "error_resolution_diagnoser stopped..." << std::endl;
}

json::value cs_listener::odom_to_json(const nav_msgs::Odometry::ConstPtr &rosmsg)
{
  // Create JSON objects
  json::value odom_json = json::value::object();
  json::value orientation = json::value::object();
  json::value position = json::value::object();

  // Create keys
  utility::string_t oKey(utility::conversions::to_string_t("orientation"));
  utility::string_t pKey(utility::conversions::to_string_t("position"));
  utility::string_t wKey(utility::conversions::to_string_t("w"));
  utility::string_t xKey(utility::conversions::to_string_t("x"));
  utility::string_t yKey(utility::conversions::to_string_t("y"));
  utility::string_t zKey(utility::conversions::to_string_t("z"));

  // Assign orientation key-value
  orientation[wKey] = json::value::number(rosmsg->pose.pose.orientation.w);
  orientation[xKey] = json::value::number(rosmsg->pose.pose.orientation.x);
  orientation[yKey] = json::value::number(rosmsg->pose.pose.orientation.y);
  orientation[zKey] = json::value::number(rosmsg->pose.pose.orientation.z);

  // Assign position key-value
  position[xKey] = json::value::number(rosmsg->pose.pose.position.x);
  position[yKey] = json::value::number(rosmsg->pose.pose.position.y);
  position[zKey] = json::value::number(rosmsg->pose.pose.position.z);

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
  utility::string_t oKey(utility::conversions::to_string_t("orientation"));
  utility::string_t pKey(utility::conversions::to_string_t("position"));
  utility::string_t wKey(utility::conversions::to_string_t("w"));
  utility::string_t xKey(utility::conversions::to_string_t("x"));
  utility::string_t yKey(utility::conversions::to_string_t("y"));
  utility::string_t zKey(utility::conversions::to_string_t("z"));

  // Assign orientation key-value
  orientation[wKey] = json::value::number(rosmsg->pose.pose.orientation.w);
  orientation[xKey] = json::value::number(rosmsg->pose.pose.orientation.x);
  orientation[yKey] = json::value::number(rosmsg->pose.pose.orientation.y);
  orientation[zKey] = json::value::number(rosmsg->pose.pose.orientation.z);

  // Assign position key-value
  position[xKey] = json::value::number(rosmsg->pose.pose.position.x);
  position[yKey] = json::value::number(rosmsg->pose.pose.position.y);
  position[zKey] = json::value::number(rosmsg->pose.pose.position.z);

  // Assign odom key-value
  pose_json[oKey] = orientation;
  pose_json[pKey] = position;

  return (pose_json);
}

void cs_listener::setup_telemetry(ros::NodeHandle nh)
{

  // Find topics relevant to telemetry info and subscribe
  std::string odom_topic = "odom";
  std::string pose_topic = "amcl_pose";

  this->odom_sub =
      nh.subscribe(odom_topic, 1000, &cs_listener::odom_callback, this);

  this->pose_sub =
      nh.subscribe(pose_topic, 1000, &cs_listener::pose_callback, this);

  // // Example for optional subscription
  // const ros::master::TopicInfo &info = *it;
  // if (info.datatype == odom_msg_type)
  // {
  //   // If odom type is found, subscribe
  //   odom_topic = info.name;
  //   std::cout << "Odom topic found! Subscribing to " << info.name << " for telemetry." << std::endl;
  //   this->odom_sub =
  //       nh.subscribe(odom_topic, 1000, &cs_listener::odom_callback, this);
  // }
  // else if ((info.datatype == pose_msg_type) && (info.name != "/initialpose"))
  // {
  //   // If pose type is found, subscribe
  //   pose_topic = info.name;
  //   std::cout << "Pose topic found! Subscribing to " << info.name << " for telemetry." << std::endl;
  //   this->pose_sub =
  //       nh.subscribe(pose_topic, 1000, &cs_listener::pose_callback, this);
  // }
}

void cs_listener::setup_diagnostics(ros::NodeHandle nh)
{
  if (this->diag_setting == "on")
  {
    // Subscribe to diagnostics_agg topic
    this->diag_sub =
        nh.subscribe("diagnostics_agg", 1000, &cs_listener::diag_callback, this);
  }
}

void cs_listener::log_callback(const rosgraph_msgs::Log::ConstPtr &rosmsg)
{
  // If node list is not set
  if (this->node_list.empty())
  {
    // If node except list is not set, process everything
    if (this->node_ex_list.empty())
    {
      // To debug this callback function
      std::cout << "Message received: " << rosmsg->msg << std::endl;

      // Callback that hands over message to State Manager
      this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg, this->telemetry);
    }
    else
    {
      // If incoming message is NOT from the node except list, process
      if (find(this->node_ex_list.begin(), this->node_ex_list.end(), rosmsg->name) == this->node_ex_list.end())
      {
        // To debug this callback function
        std::cout << "Message received: " << rosmsg->msg << std::endl;

        // Callback that hands over message to State Manager
        this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg, this->telemetry);
      }
    }
  }
  else
  {
    // If incoming message IS from the node list, process
    if (find(this->node_list.begin(), this->node_list.end(), rosmsg->name) != this->node_list.end())
    {
      // To debug this callback function
      std::cout << "Message received: " << rosmsg->msg << std::endl;

      // Callback that hands over message to State Manager
      this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg, this->telemetry);
    }
  }
}

void cs_listener::odom_callback(const nav_msgs::Odometry::ConstPtr &rosmsg)
{
  // Process odometry information for telemetry
  // std::cout << "Odom callback called" << std::endl;

  // Convert to JSON
  json::value odom_data = this->odom_to_json(rosmsg);

  // Create key
  utility::string_t odomKey(utility::conversions::to_string_t("odom_pose"));

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
  utility::string_t poseKey(utility::conversions::to_string_t("nav_pose"));

  // Assign key-value
  this->telemetry[poseKey] = pose_data;
}

void cs_listener::diag_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr &rosmsg)
{
  // Process diagnostics information

  // Check if current diagnostic sample index is less than prescribed number
  // If yes, ignore sample until prescribed number is reached. Just a simple downsample
  if (this->curr_diag_sample < this->num_diag_samples)
  {
    // Handle special case of if the current sample index is INT_MIN then it is the first ever sample, so we process.
    if (this->curr_diag_sample == INT_MIN)
    {
      this->state_manager_instance.check_diagnostic(this->agent_type, this->robot_code, rosmsg->status, this->telemetry);
      this->curr_diag_sample = 0;
    }
    else
    {
      // General case to ignore current sample
      this->curr_diag_sample++;
    }
  }
  else
  {
    // General case to process current sample if index > prescribed number
    this->state_manager_instance.check_diagnostic(this->agent_type, this->robot_code, rosmsg->status, this->telemetry);
    // Reset index back to 0 to restart sampling loop again
    this->curr_diag_sample = 0;
  }
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
  bool status = ros::master::check();
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
  ros::init(argc, argv, "error_resolution_diagnoser");
  ros::NodeHandle nh;
  ros::Rate looprate(10);

  // Create Agent
  cs_listener cs_agent;

  // Start heartbeat
  cs_agent.heartbeat_start(nh);

  // Setup telemetry
  cs_agent.setup_telemetry(nh);

  // Setup diagnostics
  cs_agent.setup_diagnostics(nh);

  // Create /rosout_agg subscriber
  ros::Subscriber rosout_agg_sub =
      nh.subscribe("rosout_agg", 1000, &cs_listener::log_callback, &cs_agent);

  // ROS Master reconnection parameters
  RobotStatus status = RobotStatus::RUNNING;
  std::string session_id;
  nh.param<std::string>("/run_id", session_id, "unknown");
  int loop_counter;
  std::cout << "AGENT:: STATUS:: OK" << std::endl;

  while (ros::ok())
  {
    ros::spinOnce();
    looprate.sleep();

    // ROS Master Connection/Reconnection
    bool master_status = ros::master::check();
    if (!master_status && status == RobotStatus::RUNNING)
    {
      // If ROS master is unavailable and robot status was running, disconnect.
      std::cout << "AGENT:: ROS master with session id `" << session_id << "` disconnected." << std::endl;
      std::cout << "AGENT:: STATUS:: MASTER_DISCONNECTED" << std::endl;
      status = RobotStatus::MASTER_DISCONNECTED;
    }
    else if (master_status && status == RobotStatus::MASTER_DISCONNECTED)
    {
      // If ROS master is available and robot status was disconnected, connect.
      std::string new_session_id;
      nh.param<std::string>("/run_id", new_session_id, "unknown");
      std::cout << "AGENT:: ROS master is back online with session id `" << session_id << "`." << std::endl;
      status = RobotStatus::RUNNING;
      if (new_session_id != session_id)
      {
        // If ROS master is new, restart agent.
        std::cout << "AGENT:: New ROS master detected. Restarting agent." << std::endl;
        std::cout << "AGENT:: STATUS:: OFFLINE" << std::endl;
        break;
      }
    }
    else if (!master_status && status == RobotStatus::MASTER_DISCONNECTED)
    {
      // If ROS master is not available and robot was disconnected, do nothing.
    }
    else
    {
      // Normal case
      status = RobotStatus::RUNNING;
    }

    if (loop_counter % 6000 == 0)
    {
      if (status == RobotStatus::RUNNING)
      {
        std::cout << "AGENT:: STATUS:: OK" << std::endl;
      }
      else if (status == RobotStatus::MASTER_DISCONNECTED)
      {
        std::cout << "AGENT:: STATUS:: MASTER_DISCONNECTED" << std::endl;
      }
    }
  }

  return 0;
}