#!/usr/bin/env python

import rospy
import criros
import IPython
import openravepy
import numpy as np
from crinspect_openrave import planning
from visualization_msgs.msg import MarkerArray





openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Error)
if __name__ == "__main__":

  file_name = 'crinspect_octomap'
  topic_name = '/camera/depth/points'

  rospy.init_node('test_octomap', anonymous=True)
  env = openravepy.Environment()
  env.Load('robots/denso_with_ftsensor.robot.xml')
  robot = env.GetRobots()[0]
  robot.SetActiveDOFValues([0, -0.34906585, 2.26892803, 0, 1.22173048, 0])
  # env.SetDefaultViewer()
  env.SetViewer('qtcoin')

  env.Load(file_name + '.wrl')
  octomap = env.GetKinBody(file_name)

  print env.CheckCollision(robot)

  lower, upper = robot.GetActiveDOFLimits()
  while True:
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
      exit()

  IPython.embed()
  exit()

# plan to some pose
# end_config = np.array([2.336375162301677, -0.7521315673852575, 2.6228803736881265, 0.5508421753993923, -0.01587398287783462, -0.2861305355919238, -0.7392284125047297])
# robot.right_arm.PlanToConfiguration(end_config,execute=True)
