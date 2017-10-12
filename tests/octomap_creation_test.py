#!/usr/bin/env python

import time
import rospy
import IPython
import openravepy as orpy
from visualization_msgs.msg import MarkerArray

class OctomapCreation(object):
  def __init__(self):
    # Initialize values
    self.busy = False
    self.topic_name = '/camera/depth/points' # TODO:
    self.file_name = 'crinspect_octomap'  # TODO:
    self.octomap_resolution = '0.005'
    self.octomap_range = '2.0'
    self.octomap_frame = 'denso/base_link'
    # robot_name = '' # TODO:
    self.durations = []
    # Initialize OpenRAVE environment
    self.env = orpy.Environment()
    self.env.SetViewer('qtcoin')
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Error)
    # Initialize ROS subscribers and publisher
    self.sub_octomap = rospy.Subscriber('/occupied_cells_vis_array',
                                        MarkerArray,
                                        callback=self.process_octomap,
                                        queue_size=1)
    # Initialize octomap plugin
    orpy.RaveLoadPlugin("or_octomap")  # Load octomap plugin
    self.sensor_server = orpy.RaveCreateSensorSystem(self.env, "or_octomap")
    self.sensor_server.SendCommand("Enable") # Enable octomap thread
    self.sensor_server.SendCommand("ResetTopic " +
                        self.topic_name) # Reset the registered pcl topic
    self.sensor_server.SendCommand("ResetResolution " +
                        self.octomap_resolution) # Reset the octomap resolution
    self.sensor_server.SendCommand("ResetRange " +
                        self.octomap_range) # Reset the octomap max range
    self.sensor_server.SendCommand("ResetFrameID " +
                        self.octomap_frame) # Reset the octomap frame
    self.time_start = time.time()

  def process_octomap(self, msg):
    assert(type(msg)==MarkerArray)
    if self.busy:
      rospy.loginfo('Processing previous data.')
      return
    for marker in msg.markers:
      if len(marker.points) > 0:
        self.busy = True
        break

  def execute(self):
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
      if self.busy:
        self.durations += [time.time()-self.time_start]
        self.sensor_server.SendCommand('TogglePause')
        time.sleep(0.5)
        # self.sensor_server.SendCommand("Mask " + robot_name)
        # self.sensor_server.SendCommand("Mask " + obstacle_name)
        # self.sensor_server.SendCommand("Reset")
        self.sensor_server.SendCommand("Save " + self.file_name)
        self.env.Load(self.file_name + '.wrl')
        self.busy=False
        rospy.signal_shutdown('Keyboard shutdown')
      rate.sleep()

if __name__ == "__main__":

  rospy.init_node('test_octomap')
  octomap_creation = OctomapCreation()
  octomap_creation.execute()

  IPython.embed()
  exit()
