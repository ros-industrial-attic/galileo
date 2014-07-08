// this code is based on 
// openni_tracker.cpp 

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <galileo/Skeletons.h>
#include <galileo/Features.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include <iostream>
#include <vector>
#include <ctime>


#define pi 3.141592653589793

//using std::string;
using namespace std;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
string frame_id("openni_depth_frame");

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
		ROS_INFO("Calibration complete, start tracking user %d", nId);
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
    ros::Publisher pub;
      
    int K;
  
  public:

    UserTracker()
    {
      K = 5;
      //sub = nh.subscribe<sensor_msgs::PointCloud2>("/output", 1, &UserTracker::surfaceDistance, this);    
      //sub = nh.subscribe("/output", 1, &UserTracker::surfaceDistance, this);      
      sub = nh.subscribe<PointCloud>("points", 1, &UserTracker::extractFeatures, this);

      pub = nh.advertise<galileo::Features>("features", 1);
    }
  
    float getMagnitude(KDL::Vector v){
      return dot(v,v);
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

      // Make sure that all joints are found with enough confidence...
      if (joint_position.fConfidence < 0.7)
      {
        pt.X = 0.0;
        pt.Y = 0.0;
        pt.Z = 0.0;
        
        return pt;
      }

      pt = joint_position.position;
      pt.X = pt.X/1000; 
      pt.Y = pt.Y/1000;
      pt.Z = pt.Z/1000;

      return pt;
    }

    // We calculate join angle between three joins
    float getLimbAngle(XnUserID const& user, 
      XnSkeletonJoint const& joint1, 
      XnSkeletonJoint const& joint2,
      XnSkeletonJoint const& joint3)
    {
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
      {
          printf("User not tracked!\n");
          return -998.0; 
      }

      XnSkeletonJointPosition joint_position1, joint_position2, joint_position3;
      
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint1, joint_position1);
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint2, joint_position2);
      g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint3, joint_position3);

      if (joint_position1.fConfidence >= 0.9 &&
          joint_position2.fConfidence >= 0.9)
      {
        // We get position 
        KDL::Vector s(joint_position1.position.X,
                      joint_position1.position.Y,
                      joint_position1.position.Z);
            
        KDL::Vector e(joint_position2.position.X,
                      joint_position2.position.Y,
                      joint_position2.position.Z);
          
        KDL::Vector h(joint_position3.position.X,
                      joint_position3.position.Y,
                      joint_position3.position.Z);
      
        // We calculate two vectors
        // S: Shoulder, E: Elbow, H: Hand
        // S  *
        //    |
        // E  *---*
        //        H
      
        KDL::Vector v1, v2;
        // EH = H-E
        v1 = h-e;
        v2 = s-e;

        // Calculate the magnitudes of the vectors
        float v1_magnitude = sqrt(getMagnitude(v1));
        float v2_magnitude = sqrt(getMagnitude(v2));


        if (v1_magnitude != 0.0 && v2_magnitude != 0.0)
        {
          // calculate join angle between limps
          float theta = acos((dot(v1,v2))/(v1_magnitude*v2_magnitude));
          return theta * 180 / pi;
        }
        else
          return -999.0;
      }
      else
        return -997.0;
    } 

    // We don't need all limps, just need some important angles of limps
    void checkUserPose(XnUserID const& userid)
    {  
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(userid))
            return;
      
      // Right elbow angle
      float right_elbow_angle = getLimbAngle(userid, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW,
        XN_SKEL_RIGHT_HAND) * pi / 180.0;
      printf("Right elbow angle %f\n", right_elbow_angle);

      // Left elbow angle
      float left_elbow_angle = getLimbAngle(userid, XN_SKEL_LEFT_SHOULDER, 
        XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND) * pi / 180.0;
      printf("Left elbow angle: %f\n", left_elbow_angle);

      // right shoulder angle
      float right_shoulder_angle = getLimbAngle(userid, XN_SKEL_RIGHT_HIP, 
        XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW) * pi / 180.0;
      printf("Right Shoulder angle %f\n", right_shoulder_angle);

      // Left shoulder angle
      float left_shoulder_angle = getLimbAngle(userid, XN_SKEL_LEFT_HIP, 
        XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW) * pi / 180.0;
      printf("Left Shoulder angle %f\n", left_shoulder_angle);

    }

    float getDistance(XnPoint3D point1, XnPoint3D point2)
    {
          return  sqrt(pow((point2.X - point1.X),2) + 
          pow((point2.Y - point1.Y),2) + 
          pow((point2.Z - point1.Z),2));
    }

    void checkHandPose(XnUserID const & user)
    {
      if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
      {
        printf("User not tracked!\n");
        return;
      }

      XnPoint3D point1  = getBodyLocalization(user, XN_SKEL_RIGHT_HAND);
      XnPoint3D point2  = getBodyLocalization(user, XN_SKEL_TORSO);
            
    }
    
    void printJoinState(galileo::SkeletonJoint &skeletonJoint)
    {
      //ROS_DEBUG("angles: %.4f, %.4f, %.4f", skeletonJoint.roll, skeletonJoint.pitch, skeletonJoint.yaw);
      printf("angles:(%f, %f, %f)  pos:( %f, %f, %f) \n ",
       skeletonJoint.orientation.x, skeletonJoint.orientation.y, skeletonJoint.orientation.z,
       skeletonJoint.position.x, skeletonJoint.position.y, skeletonJoint.position.z);
    } 
     /*   
    float getPitch(tf::Quaternion &q)
    {
      return atan2(2*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    }
 
    float getYaw(tf::Quaternion q)
    {
      return asin(-2*(x*z - w*y));
    }
    //var yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    //var pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
    //var roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)s
    
    float getRoll(tf::Quaternion q)
    {
      return atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
    }*/
    
    void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, 
                          string const& frame_id, string const& child_frame_id,
                          galileo::SkeletonJoint &skeletonJoint)
    {
        static tf::TransformBroadcaster br; 

        XnSkeletonJointPosition joint_position;
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);

        double x = -joint_position.position.X / 1000.0;
        double y = joint_position.position.Y / 1000.0;
        double z = joint_position.position.Z / 1000.0;

        XnSkeletonJointOrientation joint_orientation;
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

        XnFloat* m = joint_orientation.orientation.elements;
	      /*m[0] = -m[0];
	      m[1] = -m[1];
	      m[2] = -m[2];
	      m[6] = -m[6];
	      m[7] = -m[7];
	      m[8] = -m[8];*/

        KDL::Rotation rotation(m[0], m[1], m[2],
        					   m[3], m[4], m[5],
        					   m[6], m[7], m[8]);
        printf("m[] %f %f %f %f %f %f %f %f %f\n",m[0], m[1], m[2],m[3], m[4], m[5],m[6], m[7], m[8]);
        double qx, qy, qz, qw;
        rotation.GetQuaternion(qx, qy, qz, qw);
              
        double roll, pitch, yaw;
        rotation.RPY(roll, pitch, yaw);
        
        tf::Transform transform;
        ros::Time t;
        
        t = ros::Time::now();

        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));
    
        tf::Transform change_frame;
        change_frame.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion frame_rotation;
        frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
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
		    skeletonJoint.confidence = joint_position.fConfidence;

        
        skeletonJoint.transform.translation.x = transform.getOrigin().x();
	      skeletonJoint.transform.translation.y = transform.getOrigin().y();
  	    skeletonJoint.transform.translation.z = transform.getOrigin().z();

	      skeletonJoint.transform.rotation.x = transform.getRotation().x();
	      skeletonJoint.transform.rotation.y = transform.getRotation().y();
	      skeletonJoint.transform.rotation.z = transform.getRotation().z();
	      skeletonJoint.transform.rotation.w = transform.getRotation().w();
      
        //skeletonJoint.transform = transform;

        skeletonJoint.roll = atan2(m[7],m[8]);//roll;//tf::Quaternion(qx, qy, qz, qw).getRoll();
        skeletonJoint.pitch = pitch;//tf::Quaternion(qx, qy, qz, qw).getPitch();
        skeletonJoint.yaw = yaw;//tf::Quaternion(qx, qy, qz, qw).getYaw();
        
        
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));

        return;        
       
    }

    void publishTransforms(const std::string& frame_id) {

      XnUserID users[15];
      XnUInt16 users_count = 15;
      g_UserGenerator.GetUsers(users, users_count);
          
      galileo::Skeletons skls;        
         
      for (int i = 0; i < users_count; ++i) {
          XnUserID user = users[i];
          if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
              continue;
    
          galileo::Skeleton skeleton;
  
          publishTransform(user, XN_SKEL_HEAD, frame_id, "head", skeleton.head);
          publishTransform(user, XN_SKEL_TORSO, frame_id, "torso", skeleton.torso);

          publishTransform(user, XN_SKEL_LEFT_SHOULDER, frame_id, "left_shoulder", skeleton.left_shoulder);
          publishTransform(user, XN_SKEL_LEFT_ELBOW, frame_id, "left_elbow", skeleton.left_elbow);
          publishTransform(user, XN_SKEL_LEFT_HAND, frame_id, "left_hand", skeleton.left_hand);
          publishTransform(user, XN_SKEL_LEFT_HIP, frame_id, "left_hip", skeleton.left_hip);
        
          publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder", skeleton.right_shoulder);
          publishTransform(user, XN_SKEL_RIGHT_ELBOW, frame_id, "right_elbow", skeleton.right_elbow);
          publishTransform(user, XN_SKEL_RIGHT_HAND, frame_id, "right_hand", skeleton.right_hand); 
          publishTransform(user, XN_SKEL_RIGHT_HIP, frame_id, "right_hip", skeleton.right_hip);
          
	        skeleton.userid = user;
          skls.skeletons.push_back(skeleton);

          //skeleton_pub.publish(skeleton);

          //checkUserPose(user);           
          //printJoinState(skeleton.left_hand);
          //XnPoint3D left_hand = getBodyLocalization(user, XN_SKEL_LEFT_HAND);
          //calculateDistancesSurface(left_hand, 10);
      }
  }

  void extractFeatures(const PointCloud::ConstPtr& cloud)
  {

    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);
          
    for (int i = 0; i < users_count; ++i) {
          XnUserID user = users[i];
          if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;                
        
          galileo::Skeleton skeleton;
          galileo::Features features;

          publishTransform(user, XN_SKEL_RIGHT_HAND, frame_id, "right_hand", skeleton.right_hand);

          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
          pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
          pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

          kdtree.setInputCloud (cloud);

          XnPoint3D left_hand = getBodyLocalization(user, XN_SKEL_RIGHT_HAND);
          XnPoint3D torso = getBodyLocalization(user, XN_SKEL_TORSO);

          pcl::PointXYZ searchPoint; // this is joint position
          pcl::PointXYZ closestPoint; // the closest point to surface
          geometry_msgs::Vector3 closestPointm;   
          geometry_msgs::Vector3 basePoint;

          searchPoint.x = skeleton.right_hand.position.x;          
          searchPoint.y = skeleton.right_hand.position.y;
          searchPoint.z = skeleton.right_hand.position.z;

          basePoint.x = torso.X;
          basePoint.y = torso.Y;
          basePoint.z = torso.Z;


          // K nearest neighbor search
          vector<int> pointIdxNKNSearch(K);
          vector<float> pointNKNSquaredDistance(K);

          std::cout << "K nearest neighbor of hand point (" << searchPoint.x 
                    << " " << searchPoint.y 
                    << " " << searchPoint.z
                    << ") with K=" << K << std::endl;
          std::cout << "skeleton joint position of hand (" << left_hand.X
                    << " " << left_hand.Y
                    << " " << left_hand.Z
                    << ")" << std::endl;

          printJoinState(skeleton.right_hand);

          features.joint = skeleton.right_hand;

          if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
          {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {  cout << " " << cloud->points[ pointIdxNKNSearch[i] ].x 
                   << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                   << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                   << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
               features.distances.push_back(pointNKNSquaredDistance[i]); 
               //std::sort(features.distances.begin(), features.distances.end());
            }
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
          
          /*for (size_t i = 0; i < normals->points.size(); ++i)
            std::cerr << " " << normals->points[i].normal_x 
                      << " " << normals->points[i].normal_y
                      << " " << normals->points[i].normal_z << std::endl;
          */

          features.closestPoint = closestPointm;
          features.basePoint = basePoint;

          pub.publish(features);
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

  
  string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
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

  UserTracker tracker;

	ros::Rate r(30);
        
  ros::NodeHandle pnh("~");
 
  pnh.getParam("camera_frame_id", frame_id);
  
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		
    //tracker.publishTransforms(frame_id);
    ros::spinOnce();
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
