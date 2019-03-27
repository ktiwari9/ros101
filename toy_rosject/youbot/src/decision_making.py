#!/usr/bin/env python
# @brief python_file
import rospy
from std_msgs.msg import Float32MultiArray,Float32
from youbot.srv import ControlCommand # Importing service with: from <package_name>.srv import <srv_file_name>

map_ = Float32MultiArray()
loc_ =  Float32MultiArray()



def make_decision(map_data,loc_data):
  """
  Dummy function for making a decision (x- and y- direction) based on location and map of surroundings
  :param map_data: the mapping resulting from the SLAM algorithm
  :param loc_data: the localization resulting from the SLAM algorithm

  :return: decided velocities in x- and y- direction
  :rtype: Float32
  """
  x_direction = sum(map_data.data) # Replace with advanced decision making algorithms
  y_direction = sum(loc_data.data) # Replace with advanced decision making algorithms
  return x_direction, y_direction

def send_decision(x_dir,y_dir):
  """
  Function for sending a message over ControlCommand service to control_wheels
  :param x_dir: desired velocity in x direction
  :param y_dir: desired velocity in y direction
  """
  rospy.wait_for_service('control_wheels') # Do not continue before connection to service is reached
  try:
    proxy = rospy.ServiceProxy('control_wheels',ControlCommand) # Create connection to service
    success = proxy(x_dir,y_dir) # Send x and y parameters via service and store response in variable success
  except rospy.ServiceException, e: # If service connection is unsuccessful:
    print("Service call failed: %s", e)

def callback_map(data):
  """
  Function that is called each time something is published on topic /map
  :param data: data that got published
  """
  map_ = data
  x_dir, y_dir = make_decision(map_,loc_) # Compute target x- and y- directions based on mapping and localization
  send_decision(x_dir,y_dir) # Send decision to wheel_control node over service

def callback_loc(data):
  """
  Function that is called each time something is published on topic /localization
  :param data: data that got published
  """
  loc_ = data
  x_dir, y_dir = make_decision(map_,loc_) # Compute target x- and y- directions based on mapping and localization
  send_decision(x_dir,y_dir) # Send decision to wheel_control node over service

def decision_making():
  """
  Main function of node
  """
  rospy.init_node('decision_making',anonymous=False) # "Initialize node decision_making"
  rospy.Subscriber('/map',Float32MultiArray,callback_map) # "Subscribe to topic /map. Whenever something is published on that topic, callback_map is called"
  rospy.Subscriber('/localization',Float32MultiArray,callback_loc) # "Subscribe to topic /localization. Whenever something is published on that topic, callback_loc is called"
  rospy.spin()

if __name__ == '__main__':
  decision_making()