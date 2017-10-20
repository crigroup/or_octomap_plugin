#!/usr/bin/env python

import time
import rospy
import rospkg
import IPython
import argparse
import openravepy as orpy
import dynamic_reconfigure.server
from octomap_server.cfg import OctomapServerConfig
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
  parser.add_argument('--timeout', metavar='', type=float, default=10,
    help='Timeout (s) for the octomap creation. Default=%(default).2f')
  args = parser.parse_args(rospy.myargv()[1:])
  return args

class OctomapCreation(object):
  def __init__(self, args):
    # Initialize values
    self.busy = False
    self.topic_name = args.topic
    rospack = rospkg.RosPack()
    path = rospack.get_path('or_octomap_plugin') + '/tests/'
    self.file_name = path+args.file_name
    self.octomap_resolution = str(args.resolution)
    self.octomap_range = args.range
    self.octomap_frame = args.frame
    self.timeout = args.timeout
    self.pause = False
    # self.env_name = '' # TODO:
    # self.robot_name = '' # TODO:
    # self.obstacle_name = '' # TODO:
    self.durations = []
    # Initialize OpenRAVE environment
    self.env = orpy.Environment()
    self.env.SetViewer('qtcoin')
    orpy.RaveSetDebugLevel(orpy.DebugLevel.Error)
    # self.env.Load(self.env_name)
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
    self.sensor_server.SendCommand("ResetFrameID " +
                                   self.octomap_frame) # Reset the octomap frame

    # Initialize dynamic reconfigure server
    self.server_d = dynamic_reconfigure.server.Server(OctomapServerConfig, self.cb_reconfig_server, '')

    self.sensor_server.SendCommand('TogglePause')
    rospy.sleep(0.1)
    rospy.set_param('or_octomap/sensor_model_max_range', self.octomap_range)
    self.sensor_server.SendCommand('Reset')
    self.time_start = time.time()

  def cb_reconfig_server(self, config, level):
    if not self.pause:
      self.sensor_server.SendCommand('TogglePause')
      self.pause = not self.pause
      rospy.sleep(0.1)
    rospy.set_param('or_octomap/max_depth', config['max_depth'])
    rospy.set_param('or_octomap/pointcloud_min_z', config['pointcloud_min_z'])
    rospy.set_param('or_octomap/pointcloud_max_z', config['pointcloud_max_z'])
    rospy.set_param('or_octomap/occupancy_min_z', config['occupancy_min_z'])
    rospy.set_param('or_octomap/occupancy_max_z', config['occupancy_max_z'])
    rospy.set_param('or_octomap/filter_speckles', config['filter_speckles'])
    rospy.set_param('or_octomap/filter_ground', config['filter_ground'])
    rospy.set_param('or_octomap/compress_map', config['compress_map'])
    rospy.set_param('or_octomap/incremental_2D_projection', config['incremental_2D_projection'])
    rospy.set_param('or_octomap/ground_filter_distance', config['ground_filter_distance'])
    rospy.set_param('or_octomap/ground_filter_angle', config['ground_filter_angle'])
    rospy.set_param('or_octomap/ground_filter_plane_distance', config['ground_filter_plane_distance'])
    rospy.set_param('or_octomap/sensor_model_max_range', config['sensor_model_max_range'])
    rospy.set_param('or_octomap/sensor_model_min', config['sensor_model_min'])
    rospy.set_param('or_octomap/sensor_model_max', config['sensor_model_max'])
    rospy.set_param('or_octomap/sensor_model_hit', config['sensor_model_hit'])
    rospy.set_param('or_octomap/sensor_model_miss', config['sensor_model_miss'])
    if self.pause:
      self.sensor_server.SendCommand('TogglePause')
      self.pause = not self.pause
    return config

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
      if clock > self.timeout:
        rospy.logwarn('Timeout.')
        rospy.signal_shutdown('Timeout shutdown')
      if self.busy:
        self.durations += [time.time()-self.time_start]
        self.sensor_server.SendCommand('TogglePause')
        self.pause = not self.pause
        time.sleep(0.5)
        # self.sensor_server.SendCommand("Mask " + self.robot_name)
        # self.sensor_server.SendCommand("Mask " + self.obstacle_name)
        self.sensor_server.SendCommand("Save " + self.file_name)
        self.env.Load(self.file_name + '.wrl')
        self.busy=False
        rospy.loginfo('Octomap created.')
        # rospy.signal_shutdown('Terminal shutdown')
        self.sensor_server.SendCommand('TogglePause')
        self.pause = not self.pause
        break
      rate.sleep()

if __name__ == "__main__":

  node_args = parse_args()
  rospy.init_node('octomap_creation', log_level=rospy.DEBUG)
  octomap_creation = OctomapCreation(node_args)
  octomap_creation.execute()

  IPython.embed()
  exit()
