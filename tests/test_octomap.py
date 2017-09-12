#!/usr/bin/env python

import rospy
import criros
import IPython
import openravepy
import numpy as np
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

  rospy.init_node('test_octomap', anonymous=True)
  env = openravepy.Environment()
  env.Load('robots/denso_with_ftsensor.robot.xml')
  robot = env.GetRobots()[0]
  # env.SetDefaultViewer()
  env.SetViewer('qtcoin')

  # Initialize octomap
  openravepy.RaveLoadPlugin("or_octomap")
  sensor_server = openravepy.RaveCreateSensorSystem(env, "or_octomap")
  sensor_server.SendCommand("Enable")
  sensor_server.SendCommand("Mask " + robot.GetName())

  # Create collision checker
  collision_checker = openravepy.RaveCreateCollisionChecker(env, "or_octomap_checker")

  rospy.loginfo('Wait for octomap created...')
  try:
    marker_array = MarkerArray()
    marker_array = rospy.wait_for_message('/occupied_cells_vis_array', MarkerArray, timeout=40)
    sensor_server.SendCommand('TogglePause')
    save_marker_array_to_file(marker_array=marker_array, filename='crinspect_octomap.wrl')
    sensor_server.SendCommand("Reset")

  except rospy.exceptions.ROSException:
    rospy.logerr('The octomap is not created.')
    exit()

  file_name = 'crinspect_octomap'
  # sensor_server.SendCommand("Save " + file_name)
  env.Load(file_name + '.wrl')
  octomap = env.GetKinBody(file_name)

  # for link in octomap.GetLinks():
  #   for geom in link.GetGeometries():
  #     geom.SetAmbientColor(np.array([0.04, 0.04, 0.04]))
  #     geom.SetDiffuseColor(np.array([0.2, 0.2, 0.2]))

  criros.raveutils.enable_body(octomap, False)
  print env.CheckCollision(robot)
  # criros.raveutils.set_body_transparency(octomap, 0.25)

  IPython.embed()
  exit()

# plan to some pose
# end_config = np.array([2.336375162301677, -0.7521315673852575, 2.6228803736881265, 0.5508421753993923, -0.01587398287783462, -0.2861305355919238, -0.7392284125047297])
# robot.right_arm.PlanToConfiguration(end_config,execute=True)
