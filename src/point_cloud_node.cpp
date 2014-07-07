#include <ros/ros.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void 
cloud_cb (const pcl::PCLPointCloud2ConstPtr& input)
{
  // data points processing
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
 
  sor.setInputCloud (input);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (cloud_filtered);
  
  // Publish the data
  pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_points");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  
  ros::spin ();
}

