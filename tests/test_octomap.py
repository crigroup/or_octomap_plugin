#!/usr/bin/env python

import openravepy
import rospy
import criros
import numpy as np
import IPython

openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Error)
if __name__ == "__main__":

  rospy.init_node('test_octomap', anonymous=True)
  env = openravepy.Environment()
  env.Load('robots/denso_with_ftsensor.robot.xml')
  robot = env.GetRobots()[0]
  env.SetDefaultViewer()
  # env.SetViewer('qtcoin')

  #Initialize octomap
  openravepy.RaveLoadPlugin("or_octomap")
  sensor_server = openravepy.RaveCreateSensorSystem(env,"or_octomap")
  sensor_server.SendCommand("Enable")
  sensor_server.SendCommand("Mask " + robot.GetName())

  #Create collision checker
  collision_checker = openravepy.RaveCreateCollisionChecker(env,"or_octomap_checker")
  stop = False
  while not stop:
    input = raw_input('Input a command.')
    if input == 'q':
      sensor_server.SendCommand('TogglePause')
      stop = True

  file_name = 'crinspect_octomap'

  sensor_server.SendCommand("Save " + file_name)

  env.Load(file_name + '.wrl')
  octomap = env.GetKinBody(file_name)

  for link in octomap.GetLinks():
    for geom in link.GetGeometries():
      geom.SetAmbientColor(np.array([0.04, 0.04, 0.04]))
      geom.SetDiffuseColor(np.array([0.2, 0.2, 0.2]))

  criros.raveutils.enable_body(octomap, False)
  print env.CheckCollision(robot)
  # criros.raveutils.set_body_transparency(octomap, 0.25)

  IPython.embed()
  exit()

#plan to some pose
#end_config = np.array([2.336375162301677, -0.7521315673852575, 2.6228803736881265, 0.5508421753993923, -0.01587398287783462, -0.2861305355919238, -0.7392284125047297])
#robot.right_arm.PlanToConfiguration(end_config,execute=True)
