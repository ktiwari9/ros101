Cheatsheets {#CheatSheets}
===============

## Basic ROS commands

### ROS environment

__Activate catkin workspace__

	source devel/setup.bash

__Build catkin workspace__

	catkin_make

__Initializing a new catkin workspace__
	
	cd desired_location
	mkdir -p catkin_workspace_name/src
	cd catkin_workspace_name/src
	catkin_init_workspace

__Creating new ROSjects__

	cd catkin_workspace_location/src
	catkin_create_pkg rosject_name dependency1 dependency2 .... dependencyn

__Checklist for python ROSjects__

- Are there `__init__.py` files where required?
- Is the `catkin_python_setup()` line uncommented in CMakeLists.txt?
- Is every python node executable? (`chmod +x node.py`)
- Does the ROSject have a [setup.py](@ref GettingStartedSection) file in src/?
- Is each node specified in CMakeLists.txt as executable script 

		install(PROGRAMS
		src/node.py
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

### Nodes

__Launch master node__

	roscore

__Launch node from ROSject__

	rosrun rosject_name rosnode_name

__Launch multiple nodes specified in launch file__

	roslaunch rosject_name launch_file.launch

__Get list of all nodes currently running__

	rosnode list

### Topics

__Get list of all topics currently running__

	rostopic list

__Print output of topic to terminal window__

	rostopic echo topic_name

__Print publishing frequency of topic to terminal window__

	rostopic hz topic_name

__Print info about topic to terminal window__

	rostopic info topic_name

## Programming nodes

__Initializing node__

	rospy.init_node('node_name') # Initialize node with name node_name

__Publishing__

	pub = rospy.Publisher('topic_name',data_type,queue_size=k) # This node publishes to the topic /topic_name with format data_type with queue size k
	pub.publish(data) # Publish data to topic /topic_name

__Subscribing__

	def callback_function(data):
		# Insert functionality here that activates when data is published to topic
	rospy.Subscriber('topic_name',data_type,callback_function) # Subscribe to /topic_name with format data_type and run callback_function each time something is published

__Loops__

	loop_rate = rospy.Rate(hz) # Create loop_rate object with frequency hz Hz
	while not rospy.is_shutdown(): # Run loop until rospy is interrupted (e.g. with ctrl+c)
		loop_rate.sleep() # Sleep for 1/hz seconds

__Printing__

	rospy.init_node('node_name',log_level=rospy.INFO) # add log_level argument in order to ensure that log messages of this node get published to /rosout
	rospy.loginfo("Debug message") # Print out string or variable of any type to /rosout

__Services__

*rosject/srv/ServiceName.srv:*

	data_type1 request_parameter1
	data_type2 request_parameter2
	---
	data_type3 response_parameter1

*Reply node:*

	from rosject.srv import ServiceName # Import service from file rosject/srv/ServiceName.srv
	from std_msgs.msg import data_type1 data_type2 ... data_typen # Import all datatypes that ServiceName.srv uses

	s = rospy.Service('name', ServiceName, send_reply) # Create service named 'name' of type ServiceName and with callback function send_reply

	def send_reply(request_parameter1,request_parameter2):
		# Specify what needs to be done each time a request has arrived through the service
		return response_parameter1

*Request node:*

	def send_request():
		rospy.wait_for_service('name') # Wait until connection to service is reached
		try:
			proxy('name',ServiceName) # Create connection to Service named 'name' of type ServiceName and save it to object proxy
			response = proxy(request_parameter1,request_parameter2) # Send request to service of object proxy and save response
		except rospy.ServiceException, e: # If service connection is unsuccessful
			# Insert exception handling functionality here

*Checklist for building services:*

- Is message_generation package added as a dependency?
- Is std_msgs added to generate_messages() in CMakeLists.txt?
- Is add_service_files udpated with your .srv files?
- Is `__init__.py` added to srv/ folder?
- Are all message types used in the .srv file added as dependencies?
