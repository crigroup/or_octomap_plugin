## Description
This package is developed based on the package [or_octomap](https://github.com/personalrobotics/or_octomap), which is used to create OctoMap in OpenRAVE environment. General information about OctoMap is available at [OctoMap](https://octomap.github.io/). More information about the octomap pipeline in ROS can ben found at http://wiki.ros.org/octomap.

## Travis - Continuous Integration
[![Build Status](https://travis-ci.org/crigroup/or_octomap_plugin.svg?branch=master)](https://travis-ci.org/crigroup/or_octomap_plugin)

## Documentation

* Throughout the various files in this repository.
* `or_octomap_plugin` documentation: https://crigroup.github.io/or_octomap_plugin/

## Run
Once the camera is launched and the point cloud topic is being published, the octomap can be created by running: 
```
rosrun or_octomap_plugin octomap_creation.py
```
In order to successfully create the octomap, some command line parameters need to be set correctly:
```
Parameters:
--file-name : File name for the generated octomap.        Default=octomap.
--topic     : Point cloud topic name.                     Default=/camera/depth/points.
--frame     : The frame name of the octomap.              Default=map.
--range     : Max range of the input point cloud (m).     Default=2.0
--resolution: Leaf cube size (mm) of the octomap.         Default=0.005
--timeout   : Timeout (s) for the octomap creation.       Default=10
```

## Test resualt
Some test results working with the ensenso camera, can be found at [wiki/Test-Results](https://github.com/crigroup/or_octomap_plugin/wiki/Test-Results)

## Note 
#### Stop debug information
When the octomap is created, there is some annoying info `Writing 181520 nodes to output stream...` keep publishing, which could not be stopped from the command or code. The most efficient way to stop it is to add the following line `#define NDEBUG` to the file `/opt/ros/kinetic/include/octomap/octomap_types.h` and then run `catkin_make`, which would look like:
```c++
#define NDEBUG 
// no debug output if not in debug mode:
#ifdef NDEBUG
  #ifndef OCTOMAP_NODEBUGOUT
    #define OCTOMAP_NODEBUGOUT
  #endif
#endif
```
#### Collision Checking
When `or_octomap` is created and enabled, `or_octomap_checker` will be set as the collision checker of OpenRAVE. When this is done, all collision checks get tested against the octomap before they are tested against other OpenRAVE kinbodies. In experiments, `fcl` collision checker is still at least ten times faster than the `or_octomap_checker`, therefore it is recommanded to load the generated octomap mesh file into OpenRAVE, and use `fcl` to make collision check. 
