////
// OctomapSensorSystem.h
//
//  Created on: Jan 24, 2014
//      Author: mklingen
////

#ifndef OCTOMAPSENSORSYSTEM_H_
#define OCTOMAPSENSORSYSTEM_H_

#include <openrave/openrave.h>
#include <octomap_server/OctomapServer.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <boost/thread/mutex.hpp>

#include <std_msgs/ColorRGBA.h>

namespace or_octomap
{
  class OctomapCollisionChecker;
  class OctomapInterface : public OpenRAVE::SensorSystemBase, public octomap_server::OctomapServer
  {
    public:
      OctomapInterface(ros::NodeHandle& nodeHandle, OpenRAVE::EnvironmentBasePtr env);
      virtual ~OctomapInterface();

      virtual bool SendCommand(std::ostream &os, std::istream &is);
      virtual void Reset();

      void SetEnabled(bool enabled);
      inline bool IsEnabled() { return m_isEnabled; }

      octomap::OcTree* GetTree() { return m_octree; }

      // Not implemented virtual functions.
      virtual void AddRegisteredBodies(const std::vector< OpenRAVE::KinBodyPtr > &vbodies) { }
      virtual OpenRAVE::KinBody::ManageDataPtr AddKinBody(OpenRAVE::KinBodyPtr pbody,
                                                  OpenRAVE::XMLReadableConstPtr pdata)
      {
        return OpenRAVE::KinBody::ManageDataPtr();
      }
      virtual bool RemoveKinBody(OpenRAVE::KinBodyPtr pbody) { return false; }
      virtual bool IsBodyPresent(OpenRAVE::KinBodyPtr pbody) { return false;}
      virtual bool EnableBody(OpenRAVE::KinBodyPtr pbody, bool bEnable) { return false; }
      virtual bool SwitchBody (OpenRAVE::KinBodyPtr pbody1, OpenRAVE::KinBodyPtr pbody2) { return false; }

      bool TogglePause(std::ostream &os, std::istream &i) { m_isPaused = !m_isPaused; return true;}
      bool Enable(std::ostream &os, std::istream &i) { SetEnabled(true); return true;}
      bool Disable(std::ostream &os, std::istream &i) { SetEnabled(false); return true; }
      bool MaskObject(std::ostream &os, std::istream &i);
      bool SaveTree(std::ostream &os, std::istream &i);
      bool ResetTree(std::ostream &os, std::istream &i);
      bool ResetTopic(std::ostream &os, std::istream &i);
      bool ResetResolution(std::ostream &os, std::istream &i);
      bool ResetFrame(std::ostream &os, std::istream &i);
      boost::mutex & GetMutex(){return m_cloudQueueMutex;}

      void Spin();
      void TestCollision();

    protected:
      void InsertScans();
      void InsertCloudWrapper(const sensor_msgs::PointCloud2::ConstPtr& cloud);
      void CreateFakeBody();
      void DestroyFakeBody();
      bool m_isEnabled;
      OctomapCollisionChecker* m_collisionChecker;
      ros::CallbackQueue m_queue;
      bool m_shouldExit;
      boost::mutex m_cloudQueueMutex;
      std::vector<sensor_msgs::PointCloud2ConstPtr> m_cloudQueue;
      bool m_isPaused;

      void updateParam();
  }; // end class OctomapInterface
}// end namespace or_octomap
#endif
