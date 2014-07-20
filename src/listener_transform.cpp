#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
//#include <galileo/Skeleton.h>
#include <vector>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define pi 3.141592653589793 

using std::string;

void getTransform(tf::StampedTransform transform, 
                  tf::TransformListener *listener, 
                  const std::string &from,
                  const std::string &to){
   
  try
  {
	  
    ros::Time now = ros::Time::now();
    (*listener).waitForTransform(from, to, ros::Time(0), ros::Duration(3.0));
    (*listener).lookupTransform(from, to, ros::Time(0), transform);

  }
    /*catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }*/
  catch (tf::ExtrapolationException ex)
  {
        ROS_ERROR("%s",ex.what());
  }
    

}

void print(tf::Matrix3x3 m){
  ROS_DEBUG("RotM:\n%.5f, %.5f, %.5f\n%.5f, %.5f, %.5f\n%.5f, %.5f, %.5f\n",
            m[0][0], m[0][1], m[0][2],
            m[1][0], m[1][1], m[1][2],
            m[2][0], m[2][1], m[2][2]);
}

void print(tf::Quaternion q){
  ROS_DEBUG("Quat: %.4f, %.4f, %.4f, %.4f", q.getX(), q.getY(), q.getZ(), q.w());
}

void print(tf::Vector3 v){
  ROS_DEBUG("Vec: %.4f, %.4f, %.4f, %.4f", v.getX(), v.getY(), v.getZ(), v.w());
}

std::string stream(tf::Vector3 v){
  std::stringstream ss;
  ss << v.x() << " "  << v.y() << " "  << v.z();
  return ss.str();
}

std::string stream(tf::Quaternion v){
  std::stringstream ss;
  ss << v.x() << " "  << v.y() << " "  << v.z()<< " "  << v.w();
  return ss.str();
}

std::string stream(tf::StampedTransform tf){
  std::stringstream ss;
  ss << stream(tf.getOrigin()) << " " 
     << stream(tf.getRotation()) << " "
     << tf.frame_id_ << " " << tf.child_frame_id_;
  return ss.str();
}

float getMagnitude(XnVector3D v){
	return sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener_transform");

	ros::NodeHandle node;
	ros::Publisher skeleton_pub = node.advertise<geometry_msgs::Twist>("skeleton",10);

	tf::TransformListener *listener;
	tf::Vector3 vec;

	ros::Rate rate(10.0);

	double elbowAngle = 0.0;
	
	ros::Time timestamp = ros::Time::now();

	while(node.ok())
  {
	  tf::StampedTransform transform;
		
    //geometry_msgs::Twist velocity;
    try
    {
      //getTransform(rShoulderToRElbow, listener, "/right_shoulder_2", "/right_elbow_2");
      //getTransform(&rElbowToRHand, listener, "/right_elbow_2", "/right_hand_2");
      //listener.lookupTransform("right_shoulder_2", "right_elbow_2", ros::Time(0), rShoulderToRElbow);
	    //(*listener).lookupTransform("/right_elbow_2", "/right_hand_2", ros::Time(0), rElbowToRHand);

      //(*listener).waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
      (*listener).lookupTransform("/base_link", "/right_hand_1", ros::Time(0), transform);
 
      ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
     	/*printf("(%f, %f, %f)",  
              rElbowToRHand.getOrigin()[0],
						  rElbowToRHand.getOrigin()[1],
						  rElbowToRHand.getOrigin()[2]);*/
			
    }
    catch (tf::TransformException ex)
    {
	    ROS_ERROR("... %s",ex.what());
    }
    
    rate.sleep();

	  return 0;
  }
} 
