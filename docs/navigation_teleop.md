Control Youbot with keyboard {#TeleopSection} 
===============

__Prerequisites:__ Familiarity with basic ROS [Simulators](@ref SimulatorsSection)

__Duration:__ 30 mins


Now that we have introduced simulation packages in ROS, it is time to do something in practice. For this, we will control the Youbot in Gazebo with our keyboard buttons.

A common approach to do real-time control with keyboard buttons in ROS is to utilize the Teleop package.

## Setting everything up

### Install Youbot navigation package

This page assumes that you have downloaded all the dependencies instructed in the simulation section, namely the Youbot 3D model files as well as the ROS control packages. If you haven't installed these yet, go back to the simulation section and install them according to the instructions there. In addition to these, you need to install the Youbot navigation package:

	cd YOUR_CATIN_WORKSPACE/src
	git clone http://github.com/youbot/youbot_navigation -b hydro-devel

Add this line to the youbot_navigation/youbot_navigation_common/CMakeLists.txt:
`include_directories(${catkin_INCLUDE_DIRS})` (somewhere between `find_package` and `catkin_package`).

You are also going to need the *sample_navigation* package from the ros101 repository. Copy the package to your catkin workspace:

    mv ~/ros101/navigation/sample_navigation ~/<NAME_OF_CATKIN_WORKSPACE>/src

Compile your catkin workspace from the root (it should compile without errors)

	cd ..
	catkin_make

### Install the Move Base package

	sudo apt-get install ros-<YOUR_ROS_DISTRO>-move-base

## Run Gazebo + teleop node

Now you should be ready to launch all your nodes required for controlling the Youbot in the Gazebo simulation.

First, launch Youbot in Gazebo

	roslaunch youbot_gazebo_robot youbot.launch

Next, launch the nodes for move base and Youbot integration

	roslaunch youbot_navigation_local move_base_local.launch

Finally, run the [teleop node](@ref packages) so that keyboard presses will activate publishing to the topics which move base listens to

	rosrun sample_navigation teleop.py

The `teleop.py` node is provided under the `sample_navigation` catkin package. Executing the node should print out the keyboard button configuration in the terminal window. Remember, in order for the node to recognize the keyboard buttons you are pressing, the terminal window of the node needs to be the active window. 

Now, you should be able to control the Youbot in the Gazebo simulation. If you have got no errors in the terminal window and you have gone through all the steps carefully, the result will look like [this](https://gitlab.com/ktiwari9/ros101/blob/master/docs/gifs/youbot_gazebo_teleop.gif).

## Recommended next step

This was the last chapter in the tutorial. Congratulations, you passed it!

You should now be able to start new ROSjects on your own. The [cheatsheet](@ref CheatSheets) you might feel useful to refresh your memory and quickly find the most commonly used commands.