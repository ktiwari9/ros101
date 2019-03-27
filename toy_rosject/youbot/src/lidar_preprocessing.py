#!/usr/bin/env python
# @brief python_file
import rospy
from std_msgs.msg import Float32


def preprocess_data(raw_data):
    """
    Function to get current velocity (speed and heading direction)
    :param raw_data: raw data to be processed

    :return: preprocessed data
    :rtype: Float32
    """
    data = raw_data/1234 # Replace with some real preprocessing algorithm here
    return data


def lidar_preprocessing():
    """
    Main function for lidar_preprocessing.py node
    """
    rospy.init_node('lidar_preprocessing', anonymous=True) # "Initialize node with name lidar_preprocessing"
    pub = rospy.Publisher('lidar_data', Float32, queue_size=10) # "This node publishes to the topic /lidar_data with format Float32"
    loop_rate = rospy.Rate(50) # Specify rate of 50Hz (this variable will become handy when specifying )

    while not rospy.is_shutdown(): # Run loop until rospy is interrupted
        raw_data = 666.0
        data = preprocess_data(raw_data) # Preprocess the data
        pub.publish(data) # Publish data to topic /lidar_data
        loop_rate.sleep() # Sleep for 1/rate

if __name__ == '__main__':
    try:
        lidar_preprocessing()
    except rospy.ROSInterruptException: # This exception is thrown if the node is somehow shutdown (e.g. with Ctrl+c)
        pass