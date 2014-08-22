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

    /** this varible allows showing cloud points */
    pcl::visualization::CloudViewer viewer;
    /** Types for segmentation */
    pcl::VoxelGrid<PointType> grid_;
    pcl::SACSegmentation<PointType> seg_;
    pcl::ExtractIndices<PointType> extract_;

    /** this variable allows block the resource */
    boost::mutex mtx_;
    /** this is the cloud object */
    CloudConstPtr cloud_;    

    /** principal node */
    ros::NodeHandle nh;

    /** principal publisher*/
    ros::Publisher pub;

    /**
    * \param threshold this param indicates the distance of threshold for segmentation
    */
    Surface( double threshold = 0.01)
      : viewer ("Planar Segmentation Viewer")
    {
      pub = nh.advertise<sensor_msgs::PointCloud2> ("surface", 1);

      grid_.setFilterFieldName ("z");
      grid_.setFilterLimits (0.0f, 3.0f);
      grid_.setLeafSize (0.01f, 0.01f, 0.01f);

      /** we segment a planar surface  */
      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setMaxIterations (1000);
      seg_.setDistanceThreshold (threshold);

      extract_.setNegative (false);
    }
    /** this function proccess the cloud data every second*/
    void 
    cloud_cb_ (const CloudConstPtr& cloud)
    {
      set (cloud);
    }

    /** this function blocks the cloud resource */
    void
    set (const CloudConstPtr& cloud)
    {
      //lock while we set our cloud;
      boost::mutex::scoped_lock lock (mtx_);
      cloud_  = cloud;
    }

    /** this function apply a planar segmentation to cloud points */
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

    /** this function publishes the new cloud points in a topic called /surface */
    void
    run ()
    {
      pcl::Grabber* interface = new pcl::OpenNIGrabber ();

      boost::function<void (const CloudConstPtr&)> f = boost::bind (&Surface::cloud_cb_, this, _1);
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
  
  /** threshold for segmentation */
  double threshold = 0.01;

  /** this represents the driver for asus/kinect sensor */
  pcl::OpenNIGrabber grabber;

  if (grabber.providesCallback<pcl::OpenNIGrabber::sig_cb_openni_point_cloud_rgba> ())
  {
    Surface<pcl::PointXYZRGBA> v(threshold);
    v.run ();
  }
  else
  {
    Surface<pcl::PointXYZ> v (threshold);
    v.run ();
  } 
 
}
