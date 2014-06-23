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
//#include <galileo/SkeletonJoint.h>


#define pi 3.141592653589793

using std::string;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

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
  public:
    /*
    void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& child_frame_id);
    //void getMarker(geometry_msgs::Point32* point, std::string const& frame_id);
    XnPoint3D getBodyLocalization(XnUserID const& user, XnSkeletonJoint const& join);
    void publishUser(XnUserID const& user);
    void publishUsers();
    float getMagnitude(KDL::Vector v);
    float getLimbAngle(XnUserID const& user, XnSkeletonJoint const& joint1, 
        XnSkeletonJoint const& joint2, XnSkeletonJoint const& joint3);
    void checkUserPose(XnUserID const& userid);
    void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, 
                      string const& frame_id, string const& child_frame_id);
    void publishTransforms(const std::string& frame_id);
    */
    UserTracker()
    {
    
    }
  
    float getMagnitude(KDL::Vector v){
	    //return sqrt(v.x() * v.X + v.Y * v.Y + v.Z * v.Z);
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

    void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, 
                          string const& frame_id, string const& child_frame_id)
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
        KDL::Rotation rotation(m[0], m[1], m[2],
        					   m[3], m[4], m[5],
        					   m[6], m[7], m[8]);
        double qx, qy, qz, qw;
        rotation.GetQuaternion(qx, qy, qz, qw);

        char child_frame_no[128];
        snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

        // #4994
        tf::Transform change_frame;
        change_frame.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion frame_rotation;
        frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
        change_frame.setRotation(frame_rotation);

        transform = change_frame * transform;
      
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
        //transformer.setTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));

        
    }

    void publishTransforms(const std::string& frame_id) {
        XnUserID users[15];
        XnUInt16 users_count = 15;
        g_UserGenerator.GetUsers(users, users_count);
        
               
        for (int i = 0; i < users_count; ++i) {
            XnUserID user = users[i];
            if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
                continue;


            publishTransform(user, XN_SKEL_HEAD, frame_id, "head");
            publishTransform(user, XN_SKEL_NECK, frame_id, "neck");
            publishTransform(user, XN_SKEL_TORSO, frame_id, "torso");

            publishTransform(user, XN_SKEL_LEFT_SHOULDER, frame_id, "left_shoulder");
            publishTransform(user, XN_SKEL_LEFT_ELBOW, frame_id, "left_elbow");
            publishTransform(user, XN_SKEL_LEFT_HAND, frame_id, "left_hand");

            publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
            publishTransform(user, XN_SKEL_RIGHT_ELBOW, frame_id, "right_elbow");
            publishTransform(user, XN_SKEL_RIGHT_HAND, frame_id, "right_hand");

            publishTransform(user, XN_SKEL_LEFT_HIP, frame_id, "left_hip");
            publishTransform(user, XN_SKEL_LEFT_KNEE, frame_id, "left_knee");
            publishTransform(user, XN_SKEL_LEFT_FOOT, frame_id, "left_foot");

            publishTransform(user, XN_SKEL_RIGHT_HIP, frame_id, "right_hip");
            publishTransform(user, XN_SKEL_RIGHT_KNEE, frame_id, "right_knee");
            publishTransform(user, XN_SKEL_RIGHT_FOOT, frame_id, "right_foot");
          
            checkUserPose(user);           
        }
    }

};
/*
void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);
    
    Skeletons skeletons;

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;
        
        Skeleton skeleton;      
        skeleton.userid = user
        publishTransform(user, XN_SKEL_HEAD,           frame_id, "neck", skeleton.head);
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck", skeleton.neck);
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso", skeleton.torso);

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder", skeleton.left_shoulder);
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow", skeleton_left_elbow);
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand", skeleton.left_hand);

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder", skeleton.right_shoulder);
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow", skeleton.right_elbow);
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand", skeleton.right_hand);

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip", skeleton.left_hip);
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee", skeleton.left_knee);
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot", skeleton.left_foot);

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip", skeleton.right_hip);
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee", skeleton.right_knee);
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot", skeleton.right_foot);

        skls.skeletons.push_back(skeleton);
    }
}
*/

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}



int main(int argc, char **argv) {
  ros::init(argc, argv, "galileo");
  ros::NodeHandle nh;

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
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

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
  string frame_id("openni_depth_frame");
  pnh.getParam("camera_frame_id", frame_id);
  
  UserTracker tracker;
        
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		
    tracker.publishTransforms(frame_id);

		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
