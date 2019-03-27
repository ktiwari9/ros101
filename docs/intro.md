Introduction {#IntroductionSection}
===============

The goal of this page is to explain:
- [What is ROS?](@ref whatIsROS)
- [Where is ROS used?](@ref whereIsROS)
- [Why is ROS used?](@ref whyROS)

__Prerequisites:__ A curious mindset and hunger for learning robotics

__Time to read:__ 10 mins

# What is ROS? {#whatIsROS}

ROS stands for *Robot Operating System* and just like the operating system as we know for computers, is a custom built OS meant to be understood by robots. In layman terms, it is the language that the robots can understand and use to communicate with humans.

# Where is ROS used? {#whereIsROS}
A robot in general is comprised of primarily two components: 
- *Hardware*: Sensors, electronics, mechanical components, body frame, etc.
- *Software*: The brain of the robot which can control and harmonize operation of various components.

ROS helps establishing a smooth two-way bridge between the hardware and software components such that the entire robotic system works in harmony as one unified unit.

# Why to use ROS? {#whyROS}

Building the robot software from scratch can be a daunting task because of the complexity and multitude of different sub-tasks that need to be performed in parallel. Say, you want to develop software for a mobile robot like the Youbot (shown below), which needs to autonomously navigate in complex environments.

![Youbot robot](images/youbot.jpg)

It needs to read and process multiple different sensory information from LiDAR, stereo camera, accelerometer, gyroscope, and, GPS. This information is then utilized to perform Simultaneous Localization and Mapping (SLAM), make decisions on where to go, control its wheels to reach the destination and plan/execute paths.

All these functions can be split up into separate software modules, and implemented one at a time.... but it is not that easy. Before one can even start with the fun stuff (robotics), some major hurdles need to be addressed first:

- __How can these software modules be connected in unison to make them work as an integrated system to be tested?__

- __How can the development and testing of the different subsystems progress independently?__

- __How can one simulate the system to ensure safety and optimal execution before deploying on physical robot platform?__ 

- __How can the complex interconnected relationships with different software modules be managed seamlessly?__

Just to set up this project from scratch would require significant amount of resources and expertise, even before one can get started with solving the robotic problems of SLAM, decision making, preprocessing data from sensors and so on.

This is where ROS comes in. With ROS, each software component can be formulated as a module, synchronised and connected via apt communication channels that allow for flow of information amongst various modules. This modular nature helps people not only to `divide-and-conquer` their own research problems but also to recycle parts of the solutions to the most common problems. This saves time as the solutions to most common problem are available as simple opensource plug-and-play modules for commercial robots and easily customizable for custom built robots. This is where the key strength of ROS lies.

# Recommended next step

Now, you hopefully know what ROS is and why it is used. Next, let us introduce some basic ROS concepts and explain how it is used, [here](@ref JargonSection) .