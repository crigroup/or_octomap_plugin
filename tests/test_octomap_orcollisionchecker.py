#!/usr/bin/env python

import time
import rospy
import criros
import IPython
import openravepy
import numpy as np
from crinspect_openrave import planning
from visualization_msgs.msg import MarkerArray


def save_marker_array_to_file(marker_array, filename):
  assert type(marker_array) == MarkerArray
  with open(filename, 'w') as f:
    f.write('#VRML V2.0 utf8\n#\n')
    f.write('# created from OctoMap \n')
    for marker in marker_array.markers:
      if len(marker.points) > 0:
        for i in range(len(marker.points)):
          f.write('Transform { translation %f %f %f \n  ' % (marker.points[i].x, marker.points[i].y, marker.points[i].z))
          f.write('children [ Shape { appearance Appearance { material Material { diffuseColor %f %f %f } } ' %
                  (marker.colors[i].r, marker.colors[i].g, marker.colors[i].b) +
                  'geometry Box { size %f %f %f } } ]\n}\n' % (marker.scale.x, marker.scale.y, marker.scale.z))


openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Error)
if __name__ == "__main__":

  file_name = 'crinspect_octomap'
  topic_name = '/camera/depth/points'

  rospy.init_node('test_octomap', anonymous=False)
  env = openravepy.Environment()
  # env.Load('robots/denso_with_ftsensor.robot.xml')
  env.Load('worlds/lab_demo.env.xml')
  robot = env.GetRobot('robot')
  robot.SetActiveDOFValues([0, -0.34906585, 2.26892803, 0, 1.22173048, 0])
  # env.SetDefaultViewer()
  env.SetViewer('qtcoin')

  # Initialize octomap
  openravepy.RaveLoadPlugin("or_octomap")
  sensor_server = openravepy.RaveCreateSensorSystem(env, "or_octomap")
  sensor_server.SendCommand("Enable")
  # print 'collision_checker', env.GetCollisionChecker(), 'kinbodies', env.GetBodies()
  # rospy.set_param('/or_octomap/resolution', 0.05)
  # sensor_server.SendCommand("Reset")
  sensor_server.SendCommand("ResetTopic " + topic_name)

  rospy.loginfo('Wait for octomap created...')
  try:
    marker_array = MarkerArray()
    time_start = time.time()
    marker_array = rospy.wait_for_message('/occupied_cells_vis_array', MarkerArray, timeout=100)
    t = time.time() - time_start
    print 'Time for constructing the octomap: {}'.format(t)
    sensor_server.SendCommand('TogglePause')
    time.sleep(0.5)
    sensor_server.SendCommand("Mask " + robot.GetName())
    sensor_server.SendCommand("Mask " + 'denso_base')
    sensor_server.SendCommand("Mask " + "workspace")
    sensor_server.SendCommand("Save " + file_name)
    # sensor_server.SendCommand("Reset")

  except rospy.exceptions.ROSException:
    rospy.logerr('The octomap is not created.')
    exit()

  env.Load(file_name + '.wrl')
  octomap = env.GetKinBody(file_name)

  criros.raveutils.enable_body(octomap, False)

  # time_start = time.time()
  env.CheckCollision(robot)
  # t = time.time()-time_start
  # print 'single collision_test time: {}'.format(t)
  # print env.CheckCollision(robot)

  lower, upper = robot.GetActiveDOFLimits()
  stop = True
  while not stop:
    if rospy.is_shutdown():
      stop = True
    try:
      qgoal = lower+np.random.rand(len(lower))*(upper-lower)
      print "qgoal: ", qgoal
      # qcur = robot.GetActiveDOFValues()
      # robot.SetActiveDOFValues(qgoal)
      # print 'qcur', qcur
      # time.sleep(0.5)
      # robot.SetActiveDOFValues(qcur)

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
