or_octomap
==========

or_octomap is a collision checker and sensor system plugin for OpenRAVE, intended to allow OpenRAVE meshes to be collision checked against octrees. 

Two plugins are created:

`or_octomap` : A sensor system. When the "Enable" command is called, an OctomapServer is created which takes in new data from a point cloud. OpenRAVE objects can be filtered out of the octomap.

`or_octomap_checker`: A collision checker. When `or_octomap` is created and enabled, `or_octomap_checker` is set as the collision checker. When this is done, all collision checks get tested against the octomap before they are tested against other OpenRAVE kinbodies.

## Dependencies

```
sudo apt-get update
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-msgs ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-mapping ros-kinetic-octomap-ros ros-kinetic-octomap-server
```

### Note: 
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
