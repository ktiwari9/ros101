#!/usr/bin/env python
# @brief python_file
import rospy
from std_msgs.msg import Float32MultiArray,Float32

pub_map = rospy.Publisher('/map', Float32MultiArray, queue_size=10) # "This node publishes to the topic /map with format Int32MultiArray"
pub_loc = rospy.Publisher('/localization', Float32MultiArray, queue_size=10) # "This node publishes to the topic /localization  with format Int32MultiArray"
map_size = 10

def perform_slam(lidar_data): 
  """
  Dummy function for SLAM algorithm
  :param lidar_data: data from lidar sensor
  """
  # Replace with real SLAM algorithm
  # Suitable format for our topics
  map_data = Float32MultiArray()
  loc_data = Float32MultiArray() 

  map_data_list = [23.2,152.3,193.99,34.212,1852.3,123.5,31.25] # Just some random numbers for a toy map
  loc_data_list = [23.5,10.0] # Just some random numbers for toy x- and y-coordinates
  map_data.data = map_data_list 
  loc_data.data = loc_data_list

  pub_map.publish(map_data) # Publish map data to /map topic
  pub_loc.publish(loc_data) # Publish localization data to /localization topic


def callback_function(data): 
  """
  Function that is called each time something is published on topic lidar_data
  :param data: the data published on topic lidar_data that activated the callback
  """
  perform_slam(data.data)


def slam():
  """
  Main function of node slam.py
  """
  rospy.init_node('slam',anonymous=False) # Initialize node slam
  rospy.Subscriber('/lidar_data',Float32,callback_function) # Get node to subscribe to topic /lidar_data. Whenever something is published on that topic, callback_function is called
  rospy.spin() # prevents python from exiting before this node is stopped


if __name__ == '__main__':
  slam()