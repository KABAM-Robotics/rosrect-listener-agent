# rosrect Listener Agent Documentation

- [Description](#description)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Syntax](#syntax)
- [Example-Application](#example-application)
    * [Catching Navigation Errors from /move_base](catching-navigation-errors-from-/move_base)
    * [Start Simulation](#start-simulation)
    * [Start rosrect Listener Agent](#start-rosrect-listener-agent)
    * [Generate a navigation error](#generate-a-navigation-error)
- [Related-Pages](#related-pages)

## Description
This article explains how to run the rosrect Listener Agent ROS node.

## Overview
This article shows how to start the rosrect Listener Agent. By the end of this, you will be able to start the agent, run a simulation and test the listener agent to listen to navigation errors.

## Prerequisites
Some knowledge of ROS and robotics is necessary.

## Installation

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
That is it for the installation!

## Syntax
Now, you can run the listener agent using the provided launch file and `roslaunch`:
```
$ roslaunch rosrect-listener-agent listener-agent.launch 
```
**NOTE: Just launching the ROS node will start a new ROS master if one is not found. This is just a syntax. We will be using this to connect to a simulation to listen to errors in the next section!

## Example Application

### Catching Navigation Errors from /move_base
In this example, we will run the rosrect Listener Agent along with the Turtlebot3 simulation to see how `/move_base` navigation errors are caught.

### Start Simulation
Start a Turtlebot3 Navigation demo as documented by Robotis [here][1]. Please make sure you have gone through the Turtlebot3 installation [documentation][2] if you are facing any errors with this step.

First, open a new terminal and launch the turtlebot3_world:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Then, open a second termainal and launch the navigation stack:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

At the end of this step, you would need to see something that looks like the following. Notice that the robot is mislocalized (i.e. the scan doesn't match the map)! :

![alt text](/docs/images/Mislocalized.png "Turtlebot mislocalized")

### Start rosrect Listener Agent
We are ready to start listening to robot errors. Simply launch the listener ROS node using the launch file:
```
$ roslaunch rosrect-listener-agent listener-agent.launch 
... logging to /home/swaroophs/.ros/log/005da8e6-73ef-11ea-b5e3-9cb6d09cab4f/roslaunch-swarooph-xps-12815.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:42941/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.5

NODES
  /
    rosrect_listener_agent_node (rosrect-listener-agent/rosrect-listener-agent)

ROS_MASTER_URI=http://localhost:11311

process[rosrect_listener_agent_node-1]: started with pid [12834]
```

### Generate a navigation error
Now, use `rviz` to provide a `2D Nav Goal` for the robot. 

![alt text](/docs/images/NavGoal.png "Navigation Goal in rviz")

Because the robot is mislocalized, chances are high that it will be unable to reach its goal, generating an error. When that happens, the terminal window running the listener agent will show something like the following:
```
/move_base reporting error: Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00
Create a ticket!
/move_base reporting error: Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00
Create a ticket!
/move_base reporting error: Aborting because a valid plan could not be found. Even after executing all recovery behaviors
Create a ticket!
```

This shows that the listener agent is able to successfully capture `rosout` logs with [severity level][3] `ERROR` and origin node of `/move_base` which is the navigation stack that the Turtlebot3 is using.


## Related Pages
For more related information, refer to:
* [Virtual Navigation with Turtlebot3][1]
* [Turtlebot3 installing packages][2]
* [rosgraph_msgs documentation][3]
* [ROS logging documentation][4]

[1]: http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-navigation-with-turtlebot3
[2]: http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-packages
[3]: http://docs.ros.org/api/rosgraph_msgs/html/msg/Log.html
[4]: http://wiki.ros.org/roscpp/Overview/Logging