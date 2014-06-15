#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
//#include <galileo/Skeleton.h>
#include <vector>


#define pi 3.141592653589793 

using std::string;

void getTransform(tf::StampedTransform *transform, 
                  tf::TransformListener *listener, 
                  const std::string &from,
                  const std::string &to){
   
    try{
        (*listener).lookupTransform(from, to, ros::Time(0), (*transform));
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener_transform");

	ros::NodeHandle node;
	ros::Publisher skeleton_pub = node.advertise<geometry_msgs::Twist>("skeleton",10);

	tf::TransformListener listener;

	std::vector<std::string> frames;

	frames.push_back("head");
	frames.push_back("neck");
	frames.push_back("torso");

	frames.push_back("left_shoulder");
	frames.push_back("left_elbow");
	frames.push_back("left_hand");

	frames.push_back("right_shoulder");
	frames.push_back("right_elbow");
	frames.push_back("right_hand");
	
	ros::Rate rate(10.0);

	double elbowAngle = 0.0;


	while(node.ok())
	{
		tf::StampedTransform transform;
		tf::StampedTransform rShoulderToRElbow;
        tf::StampedTransform rElbowToRHand;

        geometry_msgs::Twist velocity;

        getTransform(&rShoulderToRElbow, &listener, "/right_shoulder", "/right_elbow");
        getTransform(&rElbowToRHand, &listener, "/right_elbow", "/right_hand");

        //we use the angle between two vectors
        elbowAngle = (rShoulderToRElbow.getOrigin().angle(rElbowToRHand.getOrigin()));
        velocity.angular.z = atan2(transform.getOrigin().y(), transform.getOrigin().x());
		velocity.linear.x = sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));

		printf("angle:%f   angular:%f   linear:%f",elbowAngle, velocity.angular.z, velocity.linear.x);
		//skeleton_pub.name.push_back()

		/*
		// we can get from fixed frame
		for(int i=0; i< frames.size();i++)
		{

			try
			{
				listener.lookupTransform("/openni_depth_frame", frames[i], ros::Time(0), transform);
				
			}catch(tf::TransformException e){
				ROS_ERROR("%s", e.what());

			}
		} */

		rate.sleep();

	}	
	return 0;
}
