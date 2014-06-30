#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <iostream>

using namespace std;
//using namespace pcl;
static int num_s, num_f; 

class SimpleOpenNIProcessor
{
  public:
  bool hasColor_;

  SimpleOpenNIProcessor () : viewer ("PCL OpenNI Viewer"), hasColor_(false) {}
  SimpleOpenNIProcessor (bool hasColor) : viewer ("PCL OpenNI Viewer"), hasColor_(hasColor) {}
  /*
  void proccess_cloud2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud){ 

      boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> > pointer(cloud); 
      if (!viewer.wasStopped()) 
        viewer.showCloud(pointer); 
  } */

  void proccess_cloud2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  {

      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
          double now = pcl::getTime();
          std::cout << "distance of center pixel :" 
                    << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z 
                    << " mm. Average framerate: " 
                    << double(count)/double(now - last)
                    << " Hz" <<  std::endl;
         count = 0;
         last = now;
      }
      
      if (!viewer.wasStopped()){
        viewer.showCloud (cloud);
      }
  }
  
  void proccess_cloud1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {

      static unsigned count = 0;
      static double last = pcl::getTime ();
      if (++count == 30)
      {
          double now = pcl::getTime();
          std::cout << "distance of center pixel :" 
                    << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z 
                    << " mm. Average framerate: " 
                    << double(count)/double(now - last)
                    << " Hz" <<  std::endl;
         count = 0;
         last = now;
      }
      
      if (!viewer.wasStopped()){
        viewer.showCloud (cloud);
      }
  }
  
  void save_cloud1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    
    pcl::io::savePCDFileASCII ("surface.pcd", *cloud);
    //std::cerr << "Saved " << cloud.points.size () << " data points with colors." << std::endl;
    std::cerr << "Saved data points with colors." << std::endl;
    /*
    for (size_t i = 0; i < cloud.points.size (); ++i)
      std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    */
  } 

  void save_cloud2(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  {
    
    pcl::io::savePCDFileASCII ("surface.pcd", *cloud);
    //std::cerr << "Saved " << cloud.points.size () << " data points with colors." << std::endl;
    std::cerr << "Saved data points with colors." << std::endl;
    /*
    for (size_t i = 0; i < cloud.points.size (); ++i)
      std::cerr << " " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
    */
  } 

  void run ()
  {
    num_s=0;
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    if(hasColor_)
    {
      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
        f = boost::bind (&SimpleOpenNIProcessor::proccess_cloud1, this, _1);

      boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> 
        g = boost::bind (&SimpleOpenNIProcessor::save_cloud1, this, _1);

      boost::signals2::connection a = interface->registerCallback (f);
      boost::signals2::connection b = interface->registerCallback (g);
    }else
    {
      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> 
        f = boost::bind (&SimpleOpenNIProcessor::proccess_cloud2, this, _1);

      boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> 
        g = boost::bind (&SimpleOpenNIProcessor::save_cloud2, this, _1);

      boost::signals2::connection a = interface->registerCallback (f);
      boost::signals2::connection b = interface->registerCallback (g);

    }
    // start receiving point clouds
    interface->start ();

    while (true)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }

  pcl::visualization::CloudViewer viewer;
};

int main (int argc, char** argv)
{
	
  SimpleOpenNIProcessor v(true);
  v.run ();

  return (0);
}


