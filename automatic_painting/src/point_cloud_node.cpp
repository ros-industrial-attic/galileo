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


template <typename PointType>
class CloudPoints
{
  public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    pcl::visualization::CloudViewer viewer;
    pcl::VoxelGrid<PointType> grid_;
    pcl::SACSegmentation<PointType> seg_;
    pcl::ExtractIndices<PointType> extract_;

    boost::mutex mtx_;
    CloudConstPtr cloud_;    

    ros::NodeHandle nh;
    ros::Publisher pub;

    CloudPoints( double threshold = 0.01)
      : viewer ("Planar Segmentation Viewer")
    {
      pub = nh.advertise<sensor_msgs::PointCloud2> ("surface", 1);

      grid_.setFilterFieldName ("z");
      grid_.setFilterLimits (0.0f, 3.0f);
      grid_.setLeafSize (0.01f, 0.01f, 0.01f);

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMaxIterations (1000);
      seg_.setDistanceThreshold (threshold);

      extract_.setNegative (false);
    }

    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      set (cloud);
    }

    void
    set (const CloudConstPtr& cloud)
    {
      //lock while we set our cloud;
      boost::mutex::scoped_lock lock (mtx_);
      cloud_  = cloud;
    }

    CloudPtr
    get ()
    {
      //lock while we swap our cloud and reset it.
      boost::mutex::scoped_lock lock (mtx_);
      CloudPtr cloud_filtered (new Cloud);
      CloudPtr temp_cloud1 (new Cloud);
      CloudPtr temp_cloud2 (new Cloud);

      //first apply a filter
      grid_.setInputCloud (cloud_);
      grid_.filter (*cloud_filtered);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

      seg_.setInputCloud (cloud_filtered);
      seg_.segment (*inliers, *coefficients);

      extract_.setInputCloud (cloud_filtered);
      extract_.setIndices (inliers);
      extract_.filter (*temp_cloud1);

      return (temp_cloud1);
    }

    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&CloudPoints::cloud_cb_, this, _1);
      boost::signals2::connection c = interface->registerCallback (f);
      
      interface->start ();
      
      while (!viewer.wasStopped ())
      {
        if (cloud_)
        {
          //the call to get() sets the cloud_ to null;
          CloudPtr c = get();
          viewer.showCloud (c); 
          sensor_msgs::PointCloud2 msg;
          //pcl::PCLPointCloud2 msg = *c;
          msg.header.frame_id =  "sensor_frame";
          pcl::toROSMsg(*c, msg);
          pub.publish (msg);
        }
        ros::spinOnce();
      }

      interface->stop ();
    }

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "cloud_points");
  
  double threshold = 0.01;

  pcl::OpenNIGrabber grabber;

  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    CloudPoints<pcl::PointXYZRGBA> v(threshold);
    v.run ();
  }
  else
  {
    CloudPoints<pcl::PointXYZ> v (threshold);
    v.run ();
  } 
 
}
