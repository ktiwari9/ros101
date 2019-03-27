#!/usr/bin/env python
# @brief python_file
import rospy
from youbot.srv import ControlCommand

def send_reply(req):
  """
  Function that is called each time the Service receives a control command
  :param req: request signal for service, in this case the desired velocities in x- and- y direction

  :return: response message of service, in this case to signifiy whether request succeeded or not
  :rtype: Boolean
  """
  if(req.x + req.y < 10000):
    response = True
  else:
    response = False
  return response


def wheel_control():
  """
  Main function for node wheel_control.py
  """
  rospy.init_node('wheel_control',anonymous=False) # "Initialize node wheel_control"
  s = rospy.Service('control_wheels', ControlCommand, send_reply) # Create service ControlCommand named 'control_wheels'
  rospy.spin() # prevents python from exiting before this node is stopped

if __name__ == '__main__':
  wheel_control()