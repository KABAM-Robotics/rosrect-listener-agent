[![Build Status](https://jenkins.cognicept.systems/job/cognicept-agent-pipeline/badge/icon)](https://jenkins.cognicept.systems/job/cognicept-agent-pipeline/)

# rosrect Listener Agent Documentation 

This project adheres to the Contributor Covenant [code of conduct](https://github.com/cognicept-admin/rosrect/blob/master/CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to [info@cognicept.systems](mailto:info@cognicept.systems). If you are interested in contributing, please refer to the guidelines [here](https://github.com/cognicept-admin/rosrect/blob/master/CONTRIBUTING.md).  

- [Description](#description)
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running tests](#running-tests)
- [Syntax](#syntax)
- [Example-Application](#example-application)
    * [Catching Navigation Errors from /move_base](catching-navigation-errors-from-/move_base)
    * [Start Simulation](#start-simulation)
    * [Start rosrect Listener Agent](#start-rosrect-listener-agent)
    * [Generate a navigation error](#generate-a-navigation-error)
- [Related-Pages](#related-pages)

## Description
This article explains how to run the `rosrect Listener Agent` ROS node.

## Overview
This article shows how to start the `rosrect Listener Agent`. By the end of this, you will be able to start the agent, run a simulation and test the listener agent to listen to navigation errors.

## Prerequisites
Some knowledge of ROS and robotics is necessary.

## Installation

You can get access to the agent by cloning this repo and building the ROS node. Steps are as follows:

1. Open a terminal window.

2. Install Microsoft's [`C++ REST SDK`][6] for establishing the backend api for incident management using `apt-get`:
    ```
    $ sudo apt-get install libcpprest-dev
    ```

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

## Running tests
Optionally, you can run the unit tests by following steps below. 

1. Open a new terminal and switch to the `catkin_ws` directory:

    ```
    $ cd ~/catkin_ws
    ```

3. Run tests using `catkin_make run_tests`. Your terminal will show test results similar to a snapshot below. Logs will be created in the `./test/logs` folder:

    ```
    $ catkin_make run_tests_rosrect-listener-agent
    .
    .
    .
    [==========] Running 5 tests from 1 test case.
    [----------] Global test environment set-up.
    [----------] 5 tests from RobotEventTestSuite
    [ RUN      ] RobotEventTestSuite.getLogTest
    [       OK ] RobotEventTestSuite.getLogTest (1 ms)
    [ RUN      ] RobotEventTestSuite.updateLogROSTest
    [       OK ] RobotEventTestSuite.updateLogROSTest (1 ms)
    [ RUN      ] RobotEventTestSuite.updateLogDBTest
    [       OK ] RobotEventTestSuite.updateLogDBTest (1 ms)
    [ RUN      ] RobotEventTestSuite.updateEventIdTest
    [       OK ] RobotEventTestSuite.updateEventIdTest (2 ms)
    [ RUN      ] RobotEventTestSuite.clearTest
    [       OK ] RobotEventTestSuite.clearTest (1 ms)
    [----------] 5 tests from RobotEventTestSuite (6 ms total)

    [----------] Global test environment tear-down
    [==========] 5 tests from 1 test case ran. (6 ms total)
    [  PASSED  ] 5 tests.
    .
    .
    .
    .
    [==========] Running 2 tests from 1 test case.
    [----------] Global test environment set-up.
    [----------] 2 tests from BackEndApiTestSuite
    [ RUN      ] BackEndApiTestSuite.pushTest
    Error Event logged with id: Sample id
    /home/swaroophs/catkin_ws/src/rosrect-listener-agent/test/logs/logData1.json
    [       OK ] BackEndApiTestSuite.pushTest (0 ms)
    [ RUN      ] BackEndApiTestSuite.jsonTest
    [       OK ] BackEndApiTestSuite.jsonTest (0 ms)
    [----------] 2 tests from BackEndApiTestSuite (0 ms total)

    [----------] Global test environment tear-down
    [==========] 2 tests from 1 test case ran. (0 ms total)
    [  PASSED  ] 2 tests.
    .
    .
    .
    .
    [==========] Running 8 tests from 1 test case.
    [----------] Global test environment set-up.
    [----------] 8 tests from StateManagerTestSuite
    [ RUN      ] StateManagerTestSuite.existTest
    [       OK ] StateManagerTestSuite.existTest (0 ms)
    [ RUN      ] StateManagerTestSuite.checkErrorTest
    [       OK ] StateManagerTestSuite.checkErrorTest (0 ms)
    [ RUN      ] StateManagerTestSuite.checkWarningTest
    [       OK ] StateManagerTestSuite.checkWarningTest (0 ms)
    [ RUN      ] StateManagerTestSuite.checkInfoTest
    [       OK ] StateManagerTestSuite.checkInfoTest (0 ms)
    [ RUN      ] StateManagerTestSuite.checkMessageROSErrorTest
    [       OK ] StateManagerTestSuite.checkMessageROSErrorTest (4 ms)
    [ RUN      ] StateManagerTestSuite.checkMessageROSWarningTest
    [100%] Built target _run_tests_rosrect-listener-agent_gtest_robotevent_test_node
    /home/swaroophs/catkin_ws/src/rosrect-listener-agent/test/logs/logData3.json
    [       OK ] StateManagerTestSuite.checkMessageROSWarningTest (3 ms)
    [ RUN      ] StateManagerTestSuite.checkMessageROSInfoTest
    [       OK ] StateManagerTestSuite.checkMessageROSInfoTest (1 ms)
    [ RUN      ] StateManagerTestSuite.clearTest
    [       OK ] StateManagerTestSuite.clearTest (0 ms)
    [----------] 8 tests from StateManagerTestSuite (8 ms total)

    [----------] Global test environment tear-down
    [==========] 8 tests from 1 test case ran. (8 ms total)
    [  PASSED  ] 8 tests.

    [Testcase: testlisteneragent_test_node] ... ok

    [ROSTEST]-----------------------------------------------------------------------

    [rosrect-listener-agent.rosunit-listeneragent_test_node/errorSuppressionTest][passed]
    [rosrect-listener-agent.rosunit-listeneragent_test_node/infoSuppressionTest][passed]
    [rosrect-listener-agent.rosunit-listeneragent_test_node/warningSuppressionTest][passed]

    SUMMARY
    * RESULT: SUCCESS
    * TESTS: 3
    * ERRORS: 0
    * FAILURES: 0
    ```

## Syntax
The listener agent is expecting some environment variables to be set as follows:

* `ROBOT_CODE` - A unique code that identifies the robot the agent is listening to. Usually a UUID but any string will work.
* `SITE_CODE` - A unique code that identifies the site of the robot being listened to. Usually a UUID but any string will work.
* `AGENT_ID` - A unique code that identifies the agent that is listening. Usually a UUID but any string will work.
* `AGENT_MODE` - A variable that when set to value TEST, will save JSON logs as "outputs" of the listener in the logs folder.
* `AGENT_TYPE` - The same listener can be operated to catch *ANY* ROS log or logs that are only available as part of the Error Classification System (ECS) to enable log suppression for particular robots/sites. Set it to value ROS or DB respectively. **NOTE:** The ECS feature is currently under development and is unavailable for use. 

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

### Catching Navigation Errors from /move_base
In this example, we will run the `rosrect Listener Agent` along with the Turtlebot3 simulation to see how `/move_base` navigation errors are caught.

### Start Simulation
Start a Turtlebot3 Navigation demo as documented by Robotis [here][1]. Please make sure you have gone through the Turtlebot3 installation [documentation][2] if you are facing any errors with this step.

First, open a new terminal and launch the `turtlebot3_world`:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Then, open a second termainal and launch the `turtlebot3_navigation` launch file:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

At the end of this step, you would need to see something that looks like the following. Notice that the robot is mislocalized (i.e. the scan doesn't match the map) :

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
[ INFO] [1587696824.491342942, 51.457000000]: Got new plan
[ INFO] [1587696824.710663399, 51.657000000]: Got new plan
[ INFO] [1587696824.922135530, 51.857000000]: Got new plan
[ WARN] [1587696830.797858600, 57.385000000]: DWA planner failed to produce path.
[ WARN] [1587696836.470445778, 62.557000000]: Clearing both costmaps to unstuck robot (3.00m).
[ WARN] [1587696841.920679359, 67.657000000]: Rotate recovery behavior started.
[ WARN] [1587696853.962929465, 79.158000000]: Clearing both costmaps to unstuck robot (1.84m).
[ WARN] [1587696859.321470229, 84.258000000]: Rotate recovery behavior started.
[ERROR] [1587696871.592459373, 95.758000000]: Aborting because a valid plan could not be found. Even after executing all recovery behaviors
```

The terminal window running the listener agent will show the following. Depending on the number of errors in the previous window, you will see the same number of JSON logs generated:
```
Info Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData1.json
Info Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData2.json
Warning Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData3.json
Warning Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData4.json
Warning Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData5.json
Warning Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData6.json
Error Event logged with id: d5f8e58b-394d-47b4-ab66-0fa0a179d9fc
/home/swaroophs/catkin_ws/src/cognicept_rosout_listener/test/logs/logData7.json
```

Let's look at an example JSON log, `logData7.json`. Note the `agent_id`, `robot_id` and `property_id` are same as the environment variables set before. `message` has information about the `rosout` actual message and a unique UUID `event_id`. This also has a flag `create_ticket` that can be used by downstream systems to trigger particular actions such as creating tickets or notifications:
```JSON
{
    "agent_id": "DROID",
    "compounding": "Null",
    "create_ticket": true,
    "description": "Null",
    "event_id": "d5f8e58b-394d-47b4-ab66-0fa0a179d9fc",
    "level": "Error",
    "message": "Aborting because a valid plan could not be found. Even after executing all recovery behaviors",
    "module": "Null",
    "property_id": "MFALCON",
    "resolution": "Null",
    "robot_id": "R2D2",
    "source": "/move_base",
    "timestamp": "2020-04-24T02:54:31Z"
}
```
This shows that the listener agent is able to successfully capture `rosout` logs and report it as a JSON structure. These JSON structures are created whenever there is `rosout` message. It associates an `event_id` for each of the messages. This unique `event_id` is reset whenever a log with [severity level][3] `ERROR` or `Goal reached` message. This means multiple JSON logs can be combined into a single **event log** using these `event_id`s. 

**NOTE:** You will not see a log for EACH of the `rosout` message seen on screen. Here are some scenarios to consider:

* Screen doesn't always show all `rosout` messages. Some nodes are publishing directly to the topic for the log and not displaying it on screen. These messages will also create logs. E.g. Setting goals/poses.
* During the same *event* duplicate messages will *NOT* create logs. This suppression logic is intentional and built into the listener via a `State Manager`. E.g. the log above shows 3 "Got new plan" messages. However only one log will be created for this. At the end of this *event*, there is an error, which will trigger the end of the *event*. So any other "Got new plan" messages in the future WILL create a log.

These JSON logs can be consumed by REST APIs/data streams to connect to incident management/monitoring systems to keep track of robot errors. Now, operators can monitor this incident management system to intervene robot operations to correct the errors to reduce downtime on the actual field.

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
[6]: https://github.com/microsoft/cpprestsdk
