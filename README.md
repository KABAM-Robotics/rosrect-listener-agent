# rosrect Listener Agent Documentation

This project adheres to the Contributor Covenant [code of conduct](https://github.com/cognicept-admin/rosrect/blob/master/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [info@cognicept.systems](mailto:info@cognicept.systems). If you are interested in contributing, please refer to the guidelines [here](https://github.com/cognicept-admin/rosrect/blob/master/CONTRIBUTING.md).

- [Description](#description)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Syntax](#syntax)
- [Example-Application](#example-application)
    * [Catching Navigation Errors from /move_base](catching-navigation-errors-from-move_base)
    * [Start Simulation](#start-simulation)
    * [Start rosrect Listener Agent](#start-rosrect-listener-agent)
    * [Generate navigation errors and create tickets](#generate-navigation-errors-and-create-tickets)
- [Related-Pages](#related-pages)

## Description
This article explains how to run the `rosrect Listener Agent` ROS node.

## Overview
This article shows how to start the `rosrect Listener Agent`. By the end of this, you will be able to start the agent, run a simulation and test the listener agent to listen to navigation errors.

## Prerequisites
Some knowledge of ROS and robotics is necessary.

## Installation

**NOTE: These features have only been tested on Ubuntu 18.04 and ROS Melodic.**

You can get access to the agent by cloning this repo and building the ROS node. Steps are as follows:

1. Open a terminal window.
2. Change to your `src` folder of the catkin workspace directory. Generally it is as follows:
    ```
    $ cd ~/catkin_ws/src
    ```
3. Clone the repo:
    ```git
    $ git clone https://github.com/cognicept-admin/rosrect-listener-agent
    ```
4. Change to your `catkin_ws` folder:
   ```
    $ cd ..
    ``` 
5. Issue `catkin_make` to build the ROS node:
    ```
    $ catkin_make
    ```
6. Check if node has built correctly and registered using `rospack`:
    ```
    $ rospack list | grep rosrect
      rosrect-listener-agent /home/swaroophs/catkin_ws/src/rosrect-listener-agent
    ```
7. Finally install Microsoft's [`C++ REST SDK`][6] for establishing the backend api for incident management using `apt-get`:
    ```
    $ sudo apt-get install libcpprest-dev
    ```

That is it for the installation!

## Syntax
Now, you can run the listener agent using the provided launch file and `roslaunch`:
```
$ roslaunch rosrect-listener-agent listener-agent.launch 
```
**NOTE: Just launching the ROS node will start a new ROS master if one is not found. This is just a syntax. We will be using this to connect to a simulation to listen to errors in the next section!**

## Example Application

### Catching Navigation Errors from /move_base
In this example, we will run the `rosrect Listener Agent` along with the Turtlebot3 simulation to see how `/move_base` navigation errors are caught.

### Start Simulation
Start a Turtlebot3 Navigation demo as documented by Robotis [here][1]. Please make sure you have gone through the Turtlebot3 installation [documentation][2] if you are facing any errors with this step.

First, open a new terminal and launch the `turtlebot3_world`:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Then, open a second terminal and launch the `turtlebot3_navigation` launch file:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

At the end of this step, you would need to see something that looks like the following. Notice that the robot is mislocalized i.e. the laser scan doesn't match the map! :

![alt text](/docs/images/Mislocalized.png "Turtlebot mislocalized")

### Start rosrect Listener Agent
We are ready to start listening to robot errors. Open a new terminal window. First, let us set up some test environment variables for the robot, agent and the site. For actual deployments, these will need to be configured for the real robot/site configuration on the incident management backend.
```
export API_USERNAME=cognicept
export API_PASSWORD=coinfinium
export API_URL=https://dashboard.cognicept.systems/api/v1
export ROBOT_CODE=ROBOT11
export SITE_CODE=CS12
export AGENT_CODE=CPP
```
**Note: These values are available only in the current terminal and need to be recreated every time before running the listener. One way to get around this is to place these statements in the `bashrc` file**

Simply launch the listener ROS node using the launch file:
```
$ roslaunch rosrect-listener-agent listener-agent.launch 
... logging to /home/swaroophs/.ros/log/ba192ef0-74b6-11ea-b771-9cb6d09cab4f/roslaunch-swarooph-xps-17404.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:41935/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.5

NODES
  /
    rosrect_listener_agent_node (rosrect-listener-agent/rosrect-listener-agent)

ROS_MASTER_URI=http://localhost:11311

process[rosrect_listener_agent_node-1]: started with pid [17512]
Creating API instance...
Logging into API...
Token registered...
```

### Generate navigation errors and create tickets
Now, use `rviz` to provide a `2D Nav Goal` for the robot. 

![alt text](/docs/images/NavGoal.png "Navigation Goal in rviz")

Because the robot is mislocalized, chances are high that it will be unable to reach its goal, generating an error. When that happens, the following can be observed:

The terminal window running the navigation will emit errors as shown below. Based on the type of error it might seem different for you:
```
[ERROR] [1585813925.247390668, 36.191000000]: Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00
[ WARN] [1585813930.464774909, 41.291000000]: Clearing both costmaps to unstuck robot (1.84m).
[ WARN] [1585813935.878557026, 46.392000000]: Rotate recovery behavior started.
[ERROR] [1585813937.392862200, 47.842000000]: Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00
[ERROR] [1585813942.677359717, 52.942000000]: Aborting because a valid plan could not be found. Even after executing all recovery behaviors
```

The terminal window running the listener agent will show the following. Depending on the number of errors in the previous window, you will see the same number of tickets generated:
```
Creating ticket...
Ticket: 12487, created...
Creating ticket...
Ticket: 12488, created...
Creating ticket...
Ticket: 12489, created...
```

To see the actual tickets:
1. Go to [https://dashboard.cognicept.systems/#/tickets][5]
2. Login using your credentials
3. Navigate to `Tickets` on the sidebar

You will be able to see the tickets that were just generated similar to the screenshot shown below:

![alt text](/docs/images/ticketsPage.png "Tickets")

If you click on one of the tickets, you can see the details as shown in the screenshot below. This includes the module that errored, robot code, site code and error text.

![alt text](/docs/images/ticketDetailsPage.png "Ticket Details")

This shows that the listener agent is able to successfully capture `rosout` logs with [severity level][3] `ERROR` and origin node of `/move_base` which is the navigation stack that the Turtlebot3 is using. It is also able to connect to an incident management system to keep track of robot errors. Now, operators can monitor this incident management system to intervene robot operations to correct the errors to reduce downtime on the actual field.

## Related Pages
For more related information, refer to:

* [Virtual Navigation with Turtlebot3][1]
* [Turtlebot3 installing packages][2]
* [rosgraph_msgs documentation][3]
* [ROS logging documentation][4]
* [Cognicept Systems Tickets Page][5]
* [Microsoft C++ REST SDK][6]

[1]: http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-navigation-with-turtlebot3
[2]: http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-packages
[3]: http://docs.ros.org/api/rosgraph_msgs/html/msg/Log.html
[4]: http://wiki.ros.org/roscpp/Overview/Logging
[5]: https://dashboard.cognicept.systems/#/tickets
[6]: https://github.com/microsoft/cpprestsdk