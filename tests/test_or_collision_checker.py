#!/usr/bin/env python

import time
import rospy
import criros
import rospkg
import IPython
import openravepy
import numpy as np
from visualization_msgs.msg import MarkerArray

if __name__ == "__main__":

  file_name = 'octomap'
  rospy.init_node('test_octomap')
  rospack = rospkg.RosPack()
  path = rospack.get_path('or_octomap_plugin') + '/tests/'

  # Load OpenRAVE Environment
  env = openravepy.Environment()
  env.Load('worlds/lab_demo.env.xml')
  robot = env.GetRobot('robot')
  robot.SetActiveDOFValues([0, -0.34906585, 2.26892803, 0, 1.22173048, 0])
  env.SetViewer('qtcoin')

  # Initialize octomap
  openravepy.RaveLoadPlugin("or_octomap")
  sensor_server = openravepy.RaveCreateSensorSystem(env, "or_octomap")
  sensor_server.SendCommand("Enable")
  sensor_server.SendCommand("Load " + path + file_name)

  # Create octomap and mask obstacles
  rospy.loginfo('Wait for octomap to be created...')
  try:
    marker_array = MarkerArray()
    time_start = time.time()
    marker_array = rospy.wait_for_message('/occupied_cells_vis_array',
                                                          MarkerArray,
                                                           timeout=10)
    t = time.time() - time_start
    rospy.logwarn('Time for constructing the octomap: {}'.format(t))
    sensor_server.SendCommand('TogglePause')
    time.sleep(0.5)
    sensor_server.SendCommand("Mask " + robot.GetName())
    sensor_server.SendCommand("Mask " + 'denso_base')
    sensor_server.SendCommand("Mask " + "workspace")
  except rospy.exceptions.ROSException:
    rospy.logerr('The octomap is not created.')
    exit()

  # Test collision checker
  durations = []
  lower, upper = robot.GetActiveDOFLimits()
  for i in range(100):
    try:
      qgoal = lower+np.random.rand(len(lower))*(upper-lower)
      robot.SetActiveDOFValues(qgoal)
      time_start = time.time()
      env.CheckCollision(robot)
      durations +=[time.time() - time_start]
    except rospy.exceptions.ROSException:
      print 'Error encountered in collision test.'

  rospy.logwarn('Average time for {} collision tests is: {}'.format(len(durations),
                                                    sum(durations)/len(durations)))
  IPython.embed()
  exit()
