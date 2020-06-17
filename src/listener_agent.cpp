#include <rosrect-listener-agent/listener_agent.h>

cs_listener::cs_listener()
{
  // Constructor

  // Pulling environment variables
  this->agent_type = std::getenv("AGENT_TYPE");
  this->robot_code = std::getenv("ROBOT_CODE");

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
}

cs_listener::~cs_listener()
{
  // Destructor
  std::cout << "Unsubscribed from Listener Agent..." << std::endl;
}

void cs_listener::log_callback(const rosgraph_msgs::Log::ConstPtr &rosmsg)
{

  // To debug this callback function
  std::cout << "Message received: " << rosmsg->msg << std::endl;

  // Callback that hands over message to State Manager
  this->state_manager_instance.check_message(this->agent_type, this->robot_code, rosmsg);
}

int main(int argc, char **argv)
{

  // Initialize node
  ros::init(argc, argv, "rosrect_listener_agent_node");
  ros::NodeHandle nh;
  ros::Rate looprate(10);

  // Create /rosout_agg subscriber
  cs_listener cs_agent;
  ros::Subscriber sub =
      nh.subscribe("rosout_agg", 1000, &cs_listener::log_callback, &cs_agent);

  while (ros::ok())
  {
    ros::spinOnce();
    looprate.sleep();
  }

  return 0;
}