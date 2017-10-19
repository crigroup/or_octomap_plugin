## Description
This package is developed based on the package [or_octomap](https://github.com/personalrobotics/or_octomap), which is used to create octomap in OpenRAVE environment.

## Dependency
```
sudo apt-get update
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-mapping ros-kinetic-octomap-ros ros-kinetic-octomap-server
```
## Run
Once the camera is launched and point cloud topic is being published, the octomap can be created by running: 
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
When the octomap is created, there is some annoying info `Writing 181520 nodes to output stream...` keep publishing, which could not be stopped from the command or code. The most efficient way to stop it is to add the following line `#define NDEBUG` to the file `/opt/ros/kinetic/include/octomap/octomap_types.h`, which would look like:
```c++
#define NDEBUG 
// no debug output if not in debug mode:
#ifdef NDEBUG
  #ifndef OCTOMAP_NODEBUGOUT
    #define OCTOMAP_NODEBUGOUT
  #endif
#endif
```
