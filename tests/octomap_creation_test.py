#!/usr/bin/env python

import time
import rospy
import IPython
import openravepy as orpy
from visualization_msgs.msg import MarkerArray

if __name__ == "__main__":

  # Initialize OpenRAVE environment
  topic_name = '/camera/depth/points'
  file_name = 'crinspect_octomap' # TODO:
  # robot_name = '' # TODO:
  rospy.init_node('test_octomap', anonymous=False)
  env = orpy.Environment()
  env.SetViewer('qtcoin', True)

  # Initialize octomao
  orpy.RaveLoadPlugin("or_octomap") # Load octomap plugin
  sensor_server = orpy.RaveCreateSensorSystem(env, "or_octomap")
  sensor_server.SendCommand("Enable") # Enable octomap thread
  sensor_server.SendCommand("ResetTopic " + topic_name) # Reset the octomap topic to published point cloud

  rospy.loginfo('Wait for octomap created...')
  try:
    marker_array = MarkerArray()
    time_start = time.time()
    marker_array = rospy.wait_for_message('/occupied_cells_vis_array', MarkerArray, timeout=100)
    t = time.time() - time_start
    rospy.loginfo('Time for constructing the octomap: {}'.format(t))
    sensor_server.SendCommand('TogglePause')
    time.sleep(0.5)
    # sensor_server.SendCommand("Mask " + robot_name)
    # sensor_server.SendCommand("Mask " + obstacle_name)
    # sensor_server.SendCommand("Reset")
    sensor_server.SendCommand("Save " + file_name)
  except rospy.exceptions.ROSException, KeyboardInterrupt:
    rospy.logerr('The octomap is not created.')

  # Load octomap into OpenRAVE
  env.Load(file_name + '.wrl')
  # octomap = env.GetKinBody(file_name)
  # criros.raveutils.enable_body(octomap, False)

  # Test octomap collision check
  # time_start = time.time()
  # env.CheckCollision(robot)
  # t = time.time()-time_start
  # rospy.loginfo('single collision_test time: {}'.format(t))

  IPython.embed()
  exit()
