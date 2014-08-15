// this code is based on 
// openni_tracker.cpp 
// author: Steve Ataucuri Cruz

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <automatic_painting/Skeletons.h>
#include <automatic_painting/Features.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>
#include <ctime>


#ifndef PI
#define PI 3.14159265359
#endif
#ifndef HALFPI
#define HALFPI 1.57079632679
#endif
#ifndef QUARTPI
#define QUARTPI 0.785398163397
#endif

using namespace std;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr cloud;

string frame_id("openni_depth_frame");
const float alpha = 0.5;
double x_t = 0;
double y_t = 0;
double z_t = 0;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete for user %d", nId);
    ROS_INFO("Start painting a surface for training ...");
    
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

class UserTracker
{
  private:
    ros::NodeHandle nh;
    tf::Transformer transformer;
    ros::Time timestamp;
    string frame_id;
    ros::Publisher skeleton_pub;
    ros::Subscriber sub;
    ros::Publisher features_pub;

    // joint information
    automatic_painting::SkeletonJoint rightHandJoint;
    automatic_painting::SkeletonJoint torsoJoint;

    int K;
    
    float min_yaw;
    float max_yaw;
    float min_pitch;
    float max_pitch;
    
    float resolution; // resolution for the class
  
  public:

    UserTracker(float minpitch, float minyaw, float maxpicth, float maxyaw, float res)
    {
      K = 5;

      min_yaw = minyaw;
      max_yaw = maxyaw;
      min_pitch = minpitch;
      max_pitch = maxpicth;
      resolution = res;
      
      sub = nh.subscribe<PointCloud>("surface", 1, &UserTracker::extractFeatures, this);

      features_pub = nh.advertise<automatic_painting::Features>("features", 1);

    }

    // this function return the localization of a specific joint
    XnPoint3D getBodyLocalization(XnUserID const& user, XnSkeletonJoint const& joint)
    {
      XnPoint3D pt;

      if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
      {
        printf("User not tracked!\n");
        pt.X = 0.0;
        pt.Y = 0.0;
        pt.Z = 0.0;
          
        return pt;
      }

      XnSkeletonJointPosition joint_position;
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);

      pt = joint_position.position;
      pt.X = pt.X/1000; 
      pt.Y = pt.Y/1000;
      pt.Z = pt.Z/1000;

      return pt;
    }

    /* this function set the class to a feature vector according range values of 
       start.launch file       
    */
    int getClass(float xpitch, float xyaw)  
    {
      // resolution = 1.0;
      // variables:
      //    yaw -> [-15, 15]
      //    pitch -> [-10, 10]  
      
      int cls = 0;
      for(float yaw=min_yaw; yaw < max_yaw; yaw = yaw + resolution) // range value for yaw variable
      {
        //for(float pitch=min_pitch; pitch <= max_pitch; pitch = pitch + resolution) 
        //{ 
                
          if(yaw <= xyaw && xyaw <= yaw+resolution )// &&  pitch variable 
          //   (pitch <= xpitch && xpitch <= pitch+resolution)) // yaw variable                                   
          {
            cout << " Class "<<cls<< " yaw value ->"<< xyaw <<"  range ["<<yaw<<" - "<< yaw+resolution<<"] "<<endl; 
            return cls;
          }
               
          cls++;         
        //}
        //cls++;
      }
      return -1; // there is no class
    }
  
    void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, 
                          string const& frame_id, string const& child_frame_id,
                          automatic_painting::SkeletonJoint &skeletonJoint)
    {
      static tf::TransformBroadcaster br; 

      XnSkeletonJointPosition joint_position;
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
      double x = -joint_position.position.X/1000;
      double y = joint_position.position.Y/1000;
      double z = joint_position.position.Z/1000;
   
      XnSkeletonJointOrientation joint_orientation;
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

      XnFloat* m = joint_orientation.orientation.elements;
      KDL::Rotation rotation(m[0], m[1], m[2],
      					   m[3], m[4], m[5],
      					   m[6], m[7], m[8]);
      //get the quaternion of this matrix
      double qx, qy, qz, qw;
      rotation.GetQuaternion(qx, qy, qz, qw);
      
      tf::Transform transform;  
      transform.setOrigin(tf::Vector3(x, y, z));
      transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));
  
      tf::Transform change_frame;
      change_frame.setOrigin(tf::Vector3(0, 0, 0));
      tf::Quaternion frame_rotation;
      frame_rotation.setEulerZYX(HALFPI, 0, HALFPI);
      change_frame.setRotation(frame_rotation);

      transform = change_frame * transform;
      
      geometry_msgs::Vector3 position;
	    geometry_msgs::Quaternion orientation;

      position.x = x;
      position.y = y;
      position.z = z;

      orientation.x = qx;
      orientation.y = qy;
      orientation.z = qz;
      orientation.w = qw;

      skeletonJoint.name = child_frame_id;
	    skeletonJoint.position = position;
	    skeletonJoint.orientation = orientation;
	    

      double roll, pitch, yaw;
      tf::Quaternion q_orig(qx, qy, -qz, qw);
      tf::Quaternion q_fix(0.70710678, 0., 0., 0.70710678);

      tf::Quaternion q_rot =  q_fix * q_orig * q_fix.inverse();
      
      geometry_msgs::TransformStamped t;
      
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = z;
      t.transform.rotation.x = q_rot.x();
      t.transform.rotation.y = q_rot.y();
      t.transform.rotation.z = q_rot.z();
      t.transform.rotation.w = q_rot.w();

      t.header.stamp = ros::Time::now();
      t.header.frame_id = frame_id;
      t.child_frame_id = child_frame_id;
      
      
      tf::Quaternion q(q_rot.x(), q_rot.y(), -q_rot.z(), q_rot.w());
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      skeletonJoint.pitch = pitch; 
      skeletonJoint.yaw = yaw; // * 180 / PI;

      //calculate velocities
      skeletonJoint.velocity.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                      transform.getOrigin().x());
      skeletonJoint.velocity.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                      pow(transform.getOrigin().y(), 2));
              
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
      //br.sendTransform(t);
      return;        
       
    }

    void publishTransforms(const std::string& frame_id) {

      XnUserID users[15];
      XnUInt16 users_count = 15;
      g_UserGenerator.GetUsers(users, users_count);
          
        
      for (int i = 0; i < users_count; ++i) {
          XnUserID user = users[i];
          if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
              continue;
    
          publishTransform(user, XN_SKEL_RIGHT_ELBOW, frame_id, "left_hand", rightHandJoint);	        
      }
  }

  void extractFeatures(const PointCloud::ConstPtr& cloud)  
  //void extractFeatures(const sensor_msgs::PointCloud2::ConstPtr& msg) 
  {
    
    XnUserID users[15];
    XnUInt16 users_count = 15;
    
    g_UserGenerator.GetUsers(users, users_count);
    
		for (int i = 0; i < users_count; ++i) {
      XnUserID user = users[i];
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
        continue;                

      automatic_painting::Skeleton skeleton;
      automatic_painting::Features features;

      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

      //PointCloud::ConstPtr cloud;
      //pcl::fromROSMsg(*msg, cloud);
      kdtree.setInputCloud (cloud); 

      XnPoint3D hand = getBodyLocalization(user, XN_SKEL_LEFT_ELBOW);
      XnPoint3D torso = getBodyLocalization(user, XN_SKEL_TORSO);

      pcl::PointXYZ searchPoint; // this is joint position
      pcl::PointXYZ closestPoint; // the closest point to surface
      geometry_msgs::Vector3 closestPointm;   
      geometry_msgs::Vector3 basePoint;

      searchPoint.x = hand.X;          
      searchPoint.y = hand.Y;
      searchPoint.z = hand.Z;

      basePoint.x = torso.X;
      basePoint.y = torso.Y;
      basePoint.z = torso.Z;

      // K nearest neighbor search
      vector<int> pointIdxNKNSearch(K);
      vector<float> pointNKNSquaredDistance(K);

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
          features.distances.push_back(pointNKNSquaredDistance[i]); 
      }

            
      closestPoint.x = cloud->points[pointIdxNKNSearch[0]].x; 
      closestPoint.y = cloud->points[pointIdxNKNSearch[0]].y; 
      closestPoint.z = cloud->points[pointIdxNKNSearch[0]].z; 
      closestPointm.x = closestPoint.x; 
      closestPointm.y = closestPoint.y;
      closestPointm.z = closestPoint.z;

      ne.setViewPoint (closestPoint.x, closestPoint.y, closestPoint.z); // normals around the closest point
      ne.setInputCloud (cloud);
      ne.setSearchMethod (tree);

      // Use all neighbors in a sphere of radius 3cm
      ne.setRadiusSearch (0.03);

      // Compute the features
      ne.compute (*normals);

      features.rightHand = rightHandJoint;
      //features.torsoJoint = torsoJoint;

      features.closestPoint = closestPointm;
      features.basePoint = basePoint;

      // we only publish if we get angles between a range of values 
      int cls = getClass(rightHandJoint.pitch, rightHandJoint.yaw); 
      
      features.cls = cls;
      features_pub.publish(features);
                  
    } // endfor
  }  
};

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}


int main(int argc, char **argv) {

  ros::init(argc, argv, "galileo");
  ros::NodeHandle nh;
  
  string configFilename = ros::package::getPath("automatic_painting") + "/galileo.xml";
  XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
  CHECK_RC(nRetVal, "InitFromXml");

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
  CHECK_RC(nRetVal, "Find depth generator");

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
  if (nRetVal != XN_STATUS_OK) {
	  nRetVal = g_UserGenerator.Create(g_Context);
	  CHECK_RC(nRetVal, "Find user generator");
  }

  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
	  ROS_INFO("Supplied user generator doesn't support skeleton");
	  return 1;
  }

  XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, 
    UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
	  g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);
        
  ros::NodeHandle pnh("~");

  double min_pitch,min_yaw,max_yaw,max_pitch,res;

  pnh.getParam("camera_frame_id", frame_id);
  pnh.getParam("min_pitch", min_pitch);
  pnh.getParam("min_yaw", min_yaw);
  pnh.getParam("max_pitch", max_pitch);
  pnh.getParam("max_yaw", max_yaw);
  pnh.getParam("resolution", res);

  cout<< min_pitch << " " << min_yaw << " " <<res<<endl;
  UserTracker tracker(min_pitch, min_yaw, max_pitch, max_yaw, res);
 

  while (ros::ok()) {
	  g_Context.WaitAndUpdateAll();
	  tracker.publishTransforms(frame_id);
    ros::spinOnce();
		r.sleep();
  }
  
  g_Context.Release();
	return 0;
} 
