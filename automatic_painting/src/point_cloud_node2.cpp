// author: Steve Ataucuri Cruz

#include <ros/ros.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

/**
* \brief this class allows to calculate cloud points of a surface and apply it a planar segmentation of the surface
* in order to publish the new cloud data in a topic called /surface with less points.
*
*/

template <typename PointType>
class Surface
{
  public:
    /** Data type to Cloud point */
    typedef pcl::PointCloud<PointType> Cloud;
    /** Data type to Cloud Ptr */
    typedef typename Cloud::Ptr CloudPtr;
    /** Data type to Cloud ConstPtr */
    typedef typename Cloud::ConstPtr CloudConstPtr;

    /** Types for segmentation */
    pcl::VoxelGrid<PointType> grid_;
    pcl::SACSegmentation<PointType> seg_;
    pcl::ExtractIndices<PointType> extract_;

    /** Ros Objects */
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    /** Constructor */
    Surface(double threshold = 0.01)
    {
      grid_.setFilterFieldName ("z");
      grid_.setFilterLimits (0.0f, 3.0f);
      grid_.setLeafSize (0.01f, 0.01f, 0.01f);

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMaxIterations (1000);
      seg_.setDistanceThreshold (threshold);

      extract_.setNegative (false);

      sub = nh.subscribe("input", 1, &Surface::cloud_cb, this);
      pub = nh.advertise<sensor_msgs::PointCloud2> ("surface", 1);

    }

    /** this function publish the segmentation of planar surface */
    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
    {

      if(input->height*input->width == 0)
        return;
     
      CloudPtr cloud_filtered (new Cloud);
      CloudPtr temp_cloud1 (new Cloud);
      CloudPtr temp_cloud2 (new Cloud);
      CloudPtr cloud(new Cloud);
      sensor_msgs::PointCloud2 mcloudPtr;
 
      pcl::fromROSMsg (*input, *cloud);
            
      
      grid_.setInputCloud (cloud);
      grid_.filter (*cloud_filtered);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

      seg_.setInputCloud (cloud_filtered);
      seg_.segment (*inliers, *coefficients);

      extract_.setInputCloud (cloud_filtered);
      extract_.setIndices (inliers);
      extract_.filter (*temp_cloud1);

      pcl::toROSMsg(*temp_cloud1, mcloudPtr); 
      mcloudPtr.header.frame_id = input->header.frame_id;
      pub.publish (mcloudPtr);
 
    }
};

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_points");
  
  double threshold = 0.01;

  Surface<pcl::PointXYZRGBA> surface(threshold);

  ros::spin();

  return(0);
}
