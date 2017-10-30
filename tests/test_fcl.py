#!/usr/bin/env python

import time
import rospy
import criros
import rospkg
import IPython
import openravepy
import numpy as np

if __name__ == "__main__":

  file_name = 'octomap'
  rospy.init_node('test_octomap')
  rospack = rospkg.RosPack()
  path = rospack.get_path('or_octomap_plugin') + '/tests/'

  # Load OpenRAVE Environment and octomap
  env = openravepy.Environment()
  env.Load('worlds/lab_demo.env.xml')
  env.Load(path + file_name + '.wrl')
  robot = env.GetRobot('robot')
  robot.SetActiveDOFValues([0, -0.34906585, 2.26892803, 0, 1.22173048, 0])
  env.SetViewer('qtcoin')

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
