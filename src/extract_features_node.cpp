// this code is based on 
// openni_tracker.cpp 

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <galileo/Skeletons.h>
#include <galileo/Features.h>

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
    ros::Publisher features_pub;

    // joint information
    galileo::SkeletonJoint rightHandJoint;
    galileo::SkeletonJoint torsoJoint;

    int K;
    
    float min_yaw;
    float max_yaw;
    float min_pitch;
    float max_pitch;
    
    float resolution; // resolution for the class
  
  public:

    UserTracker()
    {
      K = 5;

      min_yaw = -15.0;
      max_yaw = 15.0;
      min_pitch = -10.0;
      max_pitch = 10.0;
      resolution = 1.0;
      
      sub = nh.subscribe<PointCloud>("points", 1, &UserTracker::extractFeatures, this);

      features_pub = nh.advertise<galileo::Features>("features", 1);
      //skeleton_pub = nh.advertise<galileo::Skeletons>("skeleton", 1);

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
          return theta * 180 / PI;
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
        XN_SKEL_RIGHT_HAND) * PI / 180.0;
      printf("Right elbow angle %f\n", right_elbow_angle);

      // Left elbow angle
      float left_elbow_angle = getLimbAngle(userid, XN_SKEL_LEFT_SHOULDER, 
        XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND) * PI / 180.0;
      printf("Left elbow angle: %f\n", left_elbow_angle);

      // right shoulder angle
      float right_shoulder_angle = getLimbAngle(userid, XN_SKEL_RIGHT_HIP, 
        XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW) * PI / 180.0;
      printf("Right Shoulder angle %f\n", right_shoulder_angle);

      // Left shoulder angle
      float left_shoulder_angle = getLimbAngle(userid, XN_SKEL_LEFT_HIP, 
        XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW) * PI / 180.0;
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
    
    int getClass(float xpitch, float xyaw)  
    {
       int label = 0;
       for(float yaw=min_yaw; yaw <= max_yaw; yaw = yaw + resolution)
       {
        for(float pitch=min_pitch; pitch <= max_pitch; pitch = pitch + resolution)
        { //cout<<"[pitch="<<pitch<<" , yaw="<<yaw<<"]"<< " ( " << xpitch <<","<< xyaw<<")"<<endl;
          if(pitch==xpitch && yaw==xyaw) 
            return label;
          else           
            label++;         
        }
        label++;
       }
      return -1;
    }
    
    float getPitch(tf::Quaternion &q)
    {
      return atan2(2*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
    }

    void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, 
                          string const& frame_id, string const& child_frame_id,
                          galileo::SkeletonJoint &skeletonJoint)
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
        //printf("m[] %f %f %f %f %f %f %f %f %f\n",m[0], m[1], m[2],m[3], m[4], m[5],m[6], m[7], m[8]);
        //printf("q[] %f %f %f %f\n",qx, qy, qz, qw) ;

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
		    
  
        double roll, pitch, yaw, XX, YY, ZZ;
        tf::Quaternion q_orig(qx, qy, -qz, qw);
        tf::Quaternion q_fix(0.70710678, 0., 0., 0.70710678);

        tf::Quaternion q_rot =  q_fix * q_orig * q_fix.inverse();
        /*
        tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), 
                         transform.getRotation().z(), transform.getRotation().w());*/

        geometry_msgs::TransformStamped t;
        //tf::Quaternion q(qx, qy, qz, qw);        

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
 
        XX  =  m[6];
        YY =  m[7];
        ZZ = m[8];

        x_t  = XX*alpha + (x_t * (1 - alpha));
        y_t  = YY*alpha + (y_t * (1 - alpha));
        z_t  = ZZ*alpha + (z_t * (1 - alpha));
        
        skeletonJoint.pitch = pitch * 180 / PI;
        skeletonJoint.yaw = yaw * 180 / PI;
        //skeletonJoint.pitch2 = (atan2(m[6],sqrt(m[7]*m[7]+m[8]*m[8]))*180.0)/PI;
        //skeletonJoint.pitch2 = (atan2(x_t,sqrt(y_t*y_t + z_t*z_t))*180.0)/PI;
        
        //skeletonJoint.yaw2 = m[0]!=0? (atan2(m[3],m[0])*180)/PI : 0;

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
    
      
          publishTransform(user, XN_SKEL_TORSO, frame_id, "torso", torsoJoint);
          //publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder", rightHandJoint);
          //publishTransform(user, XN_SKEL_RIGHT_ELBOW,     frame_id, "right_elbow", rightHandJoint);
          publishTransform(user, XN_SKEL_RIGHT_HAND, frame_id, "right_hand", rightHandJoint);
        
	        
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

          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
          pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
          pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
          pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

          kdtree.setInputCloud (cloud); // wait const PointCloudConstPtr &cloud or const PointCloudConstPtr &cloud

          XnPoint3D hand = getBodyLocalization(user, XN_SKEL_RIGHT_HAND);
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

          std::cout << "K nearest neighbor of hand point (" << searchPoint.x 
                    << " " << searchPoint.y 
                    << " " << searchPoint.z
                    << ") with K=" << K << std::endl;

          if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
          {
            for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            {  cout << " " << cloud->points[ pointIdxNKNSearch[i] ].x 
                   << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                   << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                   << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
               features.distances.push_back(pointNKNSquaredDistance[i]); 
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

          features.rightHand = rightHandJoint;
          //features.torsoJoint = torsoJoint;

          features.closestPoint = closestPointm;
          features.basePoint = basePoint;

          // we publish only if we get angles between param and calculate the class of features
          int cls = getClass(rightHandJoint.pitch, rightHandJoint.yaw);
          /*if(label=!-1)
          {
            features.label = label;
            features_pub.publish(features);
          }*/
          
          features.cls = cls; //rand() % 400 + 1;
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
  
  string configFilename = ros::package::getPath("galileo") + "/galileo.xml";
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
	  tracker.publishTransforms(frame_id);
    ros::spinOnce();
		r.sleep();
  }
  
  g_Context.Shutdown();
	return 0;
} 
