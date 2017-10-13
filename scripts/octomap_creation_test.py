#!/usr/bin/env python

import time
import rospy
import IPython
import argparse
import openravepy as orpy
from visualization_msgs.msg import MarkerArray

def parse_args():
  format_class = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser(formatter_class=format_class,
                  description='Simulation of a complete inspection task')
  # Inspection parameters
  parser.add_argument('--file-name', metavar='', type=str,
    default='octomap',
    help='File name for the generated octomap. Default=%(default)s')
  parser.add_argument('--topic', metavar='', type=str,
    default='/camera/depth/points',
    help='Point cloud topic name. Default=%(default)s')
  parser.add_argument('--frame', metavar='', type=str,
    default='map',
    help='The W.R.T. frame. Default=%(default)s')
  parser.add_argument('--range', metavar='', type=float, default=2.0,
    help='Max range of the input point cloud (m). Default=%(default).2f')
  parser.add_argument('--resolution', metavar='', type=float, default=0.005,
    help='Leaf cube size (mm) of the octomap. Default=%(default).2f')
  args = parser.parse_args(rospy.myargv()[1:])
  return args

class OctomapCreation(object):
  def __init__(self, args):
    # Initialize values
    self.busy = False
    self.topic_name = args.topic
    self.file_name = args.file_name
    self.octomap_resolution = str(args.resolution)
    self.octomap_range = str(args.range)
    self.octomap_frame = args.frame
    # env_name = '' # TODO:
    # robot_name = '' # TODO:
    # obstacle = '' # TODO:
    self.durations = []
    # Initialize OpenRAVE environment
    self.env = orpy.Environment()
    self.env.SetViewer('qtcoin')
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Error)
    # self.env.Load(env_name)
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
      clock=time.time()-self.time_start
      if clock > 10:
        rospy.logwarn('Timeout.')
        rospy.signal_shutdown('Timeout shutdown')
      if self.busy:
        self.durations += [time.time()-self.time_start]
        self.sensor_server.SendCommand('TogglePause')
        time.sleep(0.5)
        # self.sensor_server.SendCommand("Mask " + robot_name)
        # self.sensor_server.SendCommand("Mask " + obstacle_name)
        self.sensor_server.SendCommand("Save " + self.file_name)
        self.env.Load(self.file_name + '.wrl')
        self.busy=False
        rospy.loginfo('Octomap created.')
        rospy.signal_shutdown('Terminal shutdown')
      rate.sleep()

if __name__ == "__main__":

  node_args = parse_args()
  rospy.init_node('test_octomap', log_level=rospy.DEBUG)
  octomap_creation = OctomapCreation(node_args)
  octomap_creation.execute()

  # IPython.embed()
  exit()
