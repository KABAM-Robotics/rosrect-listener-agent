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
This article shows how to start the `rosrect Listener Agent`. By the end of this, you will be able to start the agent, run a simulation and test the listener agent to listen to and create JSON logs.

## Prerequisites
Some knowledge of ROS and robotics is necessary.

## Installation

**NOTE: These features have only been tested on Ubuntu 18.04 and ROS Melodic.**

You can get access to the agent by cloning this repo and building the ROS node. Steps are as follows:

1. Open a terminal window.
2. Install Microsoft's [`C++ REST SDK`][6] for establishing the backend api for incident management using `apt-get`:
    ```
    $ sudo apt-get install libcpprest-dev
    ```
3. Change to your `src` folder of the catkin workspace directory. Generally it is as follows:
    ```
    $ cd ~/catkin_ws/src
    ```
4. Clone the repo:
    ```git
    $ git clone https://github.com/cognicept-admin/rosrect-listener-agent
    ```
5. Change to your `catkin_ws` folder:
   ```
    $ cd ..
    ``` 
6. Issue `catkin_make` to build the ROS node:
    ```
    $ catkin_make
    ```
7. Check if node has built correctly and registered using `rospack`:
    ```
    $ rospack list | grep rosrect
      rosrect-listener-agent /home/swaroophs/catkin_ws/src/rosrect-listener-agent
    ```

That is it for the installation!

## Syntax
The listener agent is expecting some environment variables to be set as follows:
* `ROBOT_CODE` - A unique code that identifies the robot the agent is listening to. Usually a UUID but any string will work.
* `SITE_CODE` - A unique code that identifies the site of the robot being listened to. Usually a UUID but any string will work.
* `AGENT_ID` - A unique code that identifies the agent that is listening. Usually a UUID but any string will work.
* `AGENT_MODE` - A variable that when set to value TEST, will save JSON logs as "outputs" of the listener in the logs folder.
* `AGENT_TYPE` - The same listener can be operated to catch *ANY* ROS log or logs that are only available as part of the Error Classification System (ECS) to enable log suppression for particular robots/sites. Set it to value ROS or DB respectively.

For example,
```
$ export ROBOT_CODE=R2D2
$ export SITE_CODE=MFALCON
$ export AGENT_ID=DROID
$ export AGENT_MODE=TEST
$ export AGENT_TYPE=ROS
```
**Note: These values are available only in the current terminal and need to be recreated every time before running the listener. One way to get around this is to place these statements in the `bashrc` file**

Now, you can run the listener agent using the provided launch file and `roslaunch`:
```
$ roslaunch rosrect-listener-agent listener-agent.launch 
```
**NOTE: Just launching the ROS node will start a new ROS master if one is not found. This is just a syntax. We will be using this to connect to a simulation to listen to errors in the next section!**

## Example Application

### Creating Event Logs from Turtlebot3
In this example, we will run the `rosrect Listener Agent` along with the Turtlebot3 simulation to see how JSON logs are created for different `/rosout` logs.

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
We are ready to start listening to robot errors. Open a new terminal window. First, let us set up some test environment variables for the robot, agent and the site. For actual deployments, these will need to be configured for the real robot/site configuration on the incident management backend. Some example values are below:
```
$ export ROBOT_CODE=R2D2
$ export SITE_CODE=MFALCON
$ export AGENT_ID=DROID
$ export AGENT_MODE=TEST
$ export AGENT_TYPE=ROS
```
**Note: These values are available only in the current terminal and need to be recreated every time before running the listener. One way to get around this is to place these statements in the `bashrc` file**

Simply launch the listener ROS node using the launch file:
```
$ roslaunch rosrect-listener-agent listener-agent.launch
... logging to /home/swaroophs/.ros/log/e8a150c4-7fa9-11ea-a48a-9cb6d09cab4f/roslaunch-swarooph-xps-26224.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://localhost:40351/

SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.5

NODES
  /
    rosrect_listener_agent_node (rosrect-listener-agent/rosrect-listener-agent)

ROS_MASTER_URI=http://localhost:11311

process[rosrect_listener_agent_node-1]: started with pid [26244]
TEST mode is ON. JSON Logs will be saved here: /home/swaroophs/catkin_ws/src/rosrect-listener-agent/tests/logs/
Subscribed to Listener Agent with direct rosout...
```

### Generate navigation errors and create tickets
Now, use `rviz` to provide a `2D Nav Goal` for the robot. 

![alt text](/docs/images/NavGoal.png "Navigation Goal in rviz")

Because the robot is mislocalized, chances are high that it will be unable to reach its goal, generating an error. When that happens, the following can be observed:

The terminal window running the navigation will emit errors as shown below. Based on the type of error it might seem different for you:
```
[ERROR] [1587018038.561199840, 141.104000000]: Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00
[ INFO] [1587018038.683621802, 141.204000000]: Got new plan
[ WARN] [1587018038.695809689, 141.211000000]: DWA planner failed to produce path.
[ WARN] [1587018038.804423369, 141.305000000]: Clearing both costmaps to unstuck robot (1.84m).
[ INFO] [1587018038.928574363, 141.405000000]: Got new plan
[ WARN] [1587018038.939311940, 141.414000000]: DWA planner failed to produce path.
[ WARN] [1587018039.040675524, 141.504000000]: Rotate recovery behavior started.
[ERROR] [1587018039.040853274, 141.504000000]: Rotate recovery can't rotate in place because there is a potential collision. Cost: -1.00
[ INFO] [1587018039.174766472, 141.604000000]: Got new plan
[ WARN] [1587018039.191863141, 141.615000000]: DWA planner failed to produce path.
[ERROR] [1587018039.299496906, 141.704000000]: Aborting because a valid control could not be found. Even after executing all recovery behaviors
```

The terminal window running the listener agent will show the following. Depending on the number of errors in the previous window, you will see the same number of JSON logs generated:
```
Error Event logged with id: a36c4273-b487-4ddc-8972-7211037ed7c5
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/tests/logs/logData1.json
Error Event logged with id: e78590eb-9168-4382-8408-bfbaf276d02c
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/tests/logs/logData2.json
Error Event logged with id: c235f134-e56c-4a61-b86f-60f4ecd49125
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/tests/logs/logData3.json
```

Let's look at an example JSON log, `logData3.json`. Note the `agent_id`, `robot_id` and `property_id` are same as the environment variables set before. Event log has information about the logs for this particular log and a unique UUID `event_id`. This also has a flag `create_ticket` that can be used by downstream systems to trigger particular actions such as creating tickets or notifications:
```JSON
{
    "agent_id": "DROID",
    "create_ticket": true,
    "event_category": "Error",
    "event_id": "c235f134-e56c-4a61-b86f-60f4ecd49125",
    "event_log": [
        {
            "Compounding": "Null",
            "Date/Time": "2020-04-16T06:20:39Z",
            "Description": "Null",
            "Level": "Info",
            "Message": "Got new plan",
            "Module": "Null",
            "QID": "0",
            "Resolution": "Null",
            "RobotEvent_ID": "c235f134-e56c-4a61-b86f-60f4ecd49125",
            "Source": "/move_base"
        },
        {
            "Compounding": "Null",
            "Date/Time": "2020-04-16T06:20:39Z",
            "Description": "Null",
            "Level": "Error",
            "Message": "Aborting because a valid control could not be found. Even after executing all recovery behaviors",
            "Module": "Null",
            "QID": "1",
            "Resolution": "Null",
            "RobotEvent_ID": "c235f134-e56c-4a61-b86f-60f4ecd49125",
            "Source": "/move_base"
        }
    ],
    "module_name": "Null",
    "property_id": "MFALCON",
    "robot_id": "R2D2",
    "timestamp": "2020-04-16T06:20:39Z"
}
```
This shows that the listener agent is able to successfully capture `rosout` logs and report it as a JSON structure. These JSON structures are created whenever there is a log with [severity level][3] `ERROR` or `Goal reached` message. These JSON logs can be consumed by REST APIs/data streams to connect to incident management/monitoring systems to keep track of robot errors. Now, operators can monitor this incident management system to intervene robot operations to correct the errors to reduce downtime on the actual field.

## Related Pages
For more related information, refer to:

* [Virtual Navigation with Turtlebot3][1]
* [Turtlebot3 installing packages][2]
* [rosgraph_msgs documentation][3]
* [ROS logging documentation][4]
* [Microsoft C++ REST SDK][6]

[1]: http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#virtual-navigation-with-turtlebot3
[2]: http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#install-dependent-ros-packages
[3]: http://docs.ros.org/api/rosgraph_msgs/html/msg/Log.html
[4]: http://wiki.ros.org/roscpp/Overview/Logging
[5]: https://dashboard.cognicept.systems/#/tickets
[6]: https://github.com/microsoft/cpprestsdk