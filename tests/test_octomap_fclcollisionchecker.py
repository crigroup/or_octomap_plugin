#!/usr/bin/env python

import time
import rospy
import criros
import IPython
import openravepy
import numpy as np
from crinspect_openrave import planning

openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Error)
if __name__ == "__main__":

  file_name = 'crinspect_octomap'
  topic_name = '/camera/depth/points'

  rospy.init_node('test_octomap', anonymous=True)
  env = openravepy.Environment()
  # env.Load('robots/denso_with_ftsensor.robot.xml')
  env.Load('worlds/lab_demo.env.xml')
  robot = env.GetRobot('robot')
  robot.SetActiveDOFValues([0, -0.34906585, 2.26892803, 0, 1.22173048, 0])
  # env.SetDefaultViewer()
  env.SetViewer('qtcoin')

  env.Load(file_name + '.wrl')
  octomap = env.GetKinBody(file_name)

  time_start = time.time()
  env.CheckCollision(robot)
  t = time.time()-time_start
  print 'single collision_test time: {}'.format(t)

  print env.CheckCollision(robot)

  lower, upper = robot.GetActiveDOFLimits()
  stop = False
  while not stop:
    if rospy.is_shutdown():
      stop = True
    try:
      qgoal = lower+np.random.rand(len(lower))*(upper-lower)
      print "qgoal: ", qgoal
      traj = planning.plan_to_joint_configuration(robot, qgoal, planner='birrt', max_iters=200, max_ppiters=100)
      print "planning finish."
      if traj is None:
        robot.SetActiveDOFValues([0, -0.34906585, 2.26892803, 0, 1.22173048, 0])
      else:
        controller = robot.GetController()
        controller.SetPath(traj)
        robot.WaitForController(0)
        print 'trajectory execution finish'
    except KeyboardInterrupt:
      print 'Error encountered.'
      # exit()

  IPython.embed()
  exit()

