
////
// OctomapSensorSystem.cpp
//
//  Created on: Jan 24, 2014
//      Author: mklingen
////

#include "OctomapInterface.h"
#include "OctomapCollisionChecker.h"
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
using namespace OpenRAVE;
using namespace octomap_server;

#define SAFE_DELETE(x) if((x)) { delete (x);  x = NULL; }

namespace or_octomap
{
    
    OctomapInterface::OctomapInterface(ros::NodeHandle& nodeHandle, OpenRAVE::EnvironmentBasePtr env) :
        SensorSystemBase(env), OctomapServer(nodeHandle), m_shouldExit(false)
    {

        m_isPaused = false;
        m_pointCloudSub->unsubscribe();

        //delete m_pointCloudSub;
        delete m_tfPointCloudSub;

        m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh,  "/cloud_in", 1, ros::TransportHints(), &m_queue);
        m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1, m_nh, ros::Duration(0.01));
        m_tfPointCloudSub->registerCallback(boost::bind(&OctomapInterface::InsertCloudWrapper, this, _1));

        ROS_INFO("Frame ID: %s", m_worldFrameId.c_str());
        ROS_INFO("Topic: %s", m_pointCloudSub->getTopic().c_str());

        RegisterCommand("Enable", boost::bind(&OctomapInterface::Enable, this, _1, _2),
                        "Begin collision testing octomap");
        RegisterCommand("Disable", boost::bind(&OctomapInterface::Disable, this, _1, _2),
                        "Stop collision testing octomap.");
        RegisterCommand("Mask", boost::bind(&OctomapInterface::MaskObject, this, _1, _2),
                        "Mask an object out of the octomap");
        RegisterCommand("TogglePause", boost::bind(&OctomapInterface::TogglePause, this, _1, _2),
                        "Toggles the octomap to being paused/unpaused for collecting data");
        RegisterCommand("Save", boost::bind(&OctomapInterface::SaveTree, this, _1, _2),
                        "Save the OcTree to a file.");
        RegisterCommand("Reset", boost::bind(&OctomapInterface::ResetTree, this, _1, _2),
                        "Reset the octomap.");
        RegisterCommand("ResetTopic", boost::bind(&OctomapInterface::ResetTopic, this, _1, _2),
                        "Reset the point cloud topic.");


        m_collisionChecker = NULL;
        boost::thread spinThread = boost::thread(boost::bind(&OctomapInterface::Spin, this));
        //boost::thread(boost::bind(&OctomapInterface::TestCollision, this));
    }
    
    void OctomapInterface::InsertCloudWrapper(const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        boost::mutex::scoped_lock(m_cloudQueueMutex);

        if(!m_isPaused)
        {
            m_cloudQueue.push_back(cloud);
        }
    }

    void OctomapInterface::InsertScans()
    {
        while(m_cloudQueue.size() > 0)
        {
            boost::mutex::scoped_lock(m_cloudQueueMutex);

            insertCloudCallback(m_cloudQueue.front());
            m_cloudQueue.erase(m_cloudQueue.begin());

        }
    }



    void OctomapInterface::TestCollision()
    {
        float timey = 0.0f;
        bool lastCollisionState = false;
        while(!m_shouldExit)
        {
            timey += 0.01f;
            OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
            OpenRAVE::KinBodyConstPtr fakeBody = GetEnv()->GetKinBody("_OCTOMAP_MAP_");
            OpenRAVE::KinBodyPtr fuze = GetEnv()->GetKinBody("fuze_bottle");
            if(fakeBody.get())
            {
                boost::mutex::scoped_lock(m_cloudQueueMutex);
                bool collides = GetEnv()->CheckCollision(fakeBody, report);

                ROS_INFO("Collides: %s", collides ? "true" : "false");

                OpenRAVE::Transform t;
                t.identity();

                float r = 0.1 * cos(timey * 0.1f);
                t.trans = OpenRAVE::Vector(r * sin(timey) + 0.5f, r * cos(timey), 0.0f);

                if(fuze.get())
                {
                    fuze->SetTransform(t);
                }

                lastCollisionState = collides;
            }

            usleep(10000);
        }
    }

    void OctomapInterface::Spin()
    {
        ros::WallDuration timeout(0.01);
        while(!m_shouldExit)
        {
            m_queue.callOne(timeout);
            InsertScans();
        }
    }

    OctomapInterface::~OctomapInterface()
    {
        SetEnabled(false);
        m_shouldExit = true;
    }

    void OctomapInterface::SetEnabled(bool enabled)
    {
        ROS_INFO("Set enabled called!");
        m_isEnabled = enabled;

        if(enabled)
        {
            if(m_collisionChecker)
            {
                GetEnv()->SetCollisionChecker(m_collisionChecker->GetWrappedChecker());
                SAFE_DELETE(m_collisionChecker);
            }

            OpenRAVE::CollisionCheckerBasePtr collisionCheckerBase = RaveCreateCollisionChecker(GetEnv(), "or_octomap_checker");

            m_collisionChecker = dynamic_cast<OctomapCollisionChecker*>(collisionCheckerBase.get());
            //new OctomapCollisionChecker(GetEnv(), GetEnv()->GetCollisionChecker(), this);
            m_collisionChecker->SetInterface(this);
            m_collisionChecker->SetWrappedChecker(GetEnv()->GetCollisionChecker());
            CreateFakeBody();
            GetEnv()->SetCollisionChecker(collisionCheckerBase);
        }
        else
        {
            if(m_collisionChecker)
            {
                GetEnv()->SetCollisionChecker(m_collisionChecker->GetWrappedChecker());
                SAFE_DELETE(m_collisionChecker);
                DestroyFakeBody();
            }
        }
    }


    void OctomapInterface::DestroyFakeBody()
    {
        OpenRAVE::KinBodyPtr kinBody = GetEnv()->GetKinBody("_OCTOMAP_MAP_");

        if(kinBody.get())
        {
            GetEnv()->Remove(kinBody);
        }
    }

    void OctomapInterface::CreateFakeBody()
    {
        ROS_DEBUG("Creating fake body!");
        OpenRAVE::KinBodyPtr kinbody = RaveCreateKinBody(GetEnv(), "");
        kinbody->SetName("_OCTOMAP_MAP_");
        GetEnv()->Add(kinbody);
    }

    bool OctomapInterface::SendCommand(std::ostream &os, std::istream &is)
    {
        return SensorSystemBase::SendCommand(os, is);
    }

    void OctomapInterface::Reset()
    {
        std_srvs::EmptyRequest req;
        std_srvs::EmptyResponse res;
        resetSrv(req, res);
    }

    bool OctomapInterface::ResetTree(std::ostream &os, std::istream &i)
    {
        ROS_INFO("Reset the octomap.");
        if(!m_isPaused)
        {
            m_isPaused = !m_isPaused;
        }
        ros::Duration(5).sleep();

        double resolution;
        std::string frameID;
        double maxRange;
        if(ros::param::get("/or_octomap/resolution", resolution)){
            m_res = resolution;
            m_octree->setResolution(resolution);
            ROS_INFO("Reset /or_octomap/resolution as %f", resolution);
          }
        if(ros::param::get("/or_octomap/frame_id", frameID)){
            m_worldFrameId = frameID;
            ROS_INFO("Reset /or_octomap/frame_id as %s", frameID.c_str());
          }
        if(ros::param::get("/or_octomap/sensor_model/max_range", maxRange)){
            m_maxRange = maxRange;
            ROS_INFO("Reset /or_octomap/sensor_model/max_range as %f", maxRange);
          }
        std_srvs::EmptyRequest req;
        std_srvs::EmptyResponse res;
        resetSrv(req, res);
        if(m_isPaused)
        {
            m_isPaused = !m_isPaused;
        }
        return true;
    }

    bool OctomapInterface::MaskObject(std::ostream &os, std::istream &i)
    {

        std::string objectName;
        i >> objectName;

        ROS_INFO("Masking object %s\n", objectName.c_str());

        if(objectName == "" || !IsEnabled())
        {
            return false;
        }


        bool toReturn = m_collisionChecker->MaskObject(objectName);
        publishAll();

        return toReturn;
    }


    bool OctomapInterface::SaveTree(std::ostream &os, std::istream &i){

      std::string file_name;
      i >> file_name;

      ROS_INFO("Saving the tree to the file: %s", file_name.c_str());

      if(file_name == "" || !IsEnabled())
      {
          return false;
      }

      std::size_t octomapSize = m_octree->size();
      // TODO: estimate num occ. voxels for size of arrays (reserve)
      if (octomapSize <= 1){
        ROS_WARN("The octree is empty, could not save the octomap to the file %s.", file_name.c_str());
        return false;
      }

      std::string vrmlFilename = "";
      vrmlFilename = file_name + ".wrl";

      std::ofstream outfile (vrmlFilename.c_str());
      outfile << "#VRML V2.0 utf8\n#\n";
      outfile << "# created from OctoMap \n";

      octomap::OcTree* tree = GetTree();

      std::size_t count(0);
      for(octomap::OcTree::iterator it = tree->begin(m_maxTreeDepth), end=tree->end(); it!= end; ++it) {
        if(tree->isNodeOccupied(*it)){
          count++;
          double size = it.getSize();
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          double minX, minY, minZ, maxX, maxY, maxZ;
          m_octree->getMetricMin(minX, minY, minZ);
          m_octree->getMetricMax(maxX, maxY, maxZ);
          double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
          std_msgs::ColorRGBA _color = heightMapColor(h);

          outfile << "Transform { translation "
              << it.getX() << " " << it.getY() << " " << it.getZ()
              << " \n  children [ Shape { "
              << "appearance Appearance { material Material { diffuseColor "
              << _color.r << " "<< _color.g << " " << _color.b << " } } "
              << "geometry Box { size " << size << " " << size << " " << size << " }"
              << " } ]\n}\n";
        }
      }

      outfile.close();
      std::cout << "Finished writing "<< count << " voxels to " << vrmlFilename << std::endl;
      return true;

    }

    bool OctomapInterface::ResetTopic(std::ostream &os, std::istream &i){

      std::string topic_name;
      i >> topic_name;

      ROS_INFO("Reset the point cloud topic");

      if(topic_name == "" || !IsEnabled())
      {
          return false;
      }

      m_pointCloudSub->unsubscribe();

      //delete m_pointCloudSub;
      delete m_tfPointCloudSub;

      m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh,  topic_name, 1, ros::TransportHints(), &m_queue);
      m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 1, m_nh, ros::Duration(0.1));
      m_tfPointCloudSub->registerCallback(boost::bind(&OctomapInterface::InsertCloudWrapper, this, _1));

      ROS_INFO("Frame ID: %s", m_worldFrameId.c_str());
      ROS_INFO("Topic: %s", m_pointCloudSub->getTopic().c_str());

    }


}
