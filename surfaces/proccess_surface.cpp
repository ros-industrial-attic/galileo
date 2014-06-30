#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>

#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>


//using namespace std;
//using namespace pcl;
class Surface
{
public:
  bool hasColor;
  pcl::visualization::CloudViewer viewer; 

  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr(new pcl::PointCloud<pcl::PointXYZRGBA>); 
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr; 
  //pcl::PointCloud<pcl::PointXYZ>::Ptr fallbackCloud;
  
  Surface () : viewer ("PCL OpenNI Viewer") {}
  
 
 
  void planar_segmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

    // Build a filter to remove spurious NaNs
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.1);
    pass.filter (*cloud_filtered);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud_projected);
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_hull);
 
  }

  void basic_segmentation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  { 

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

  }
  
  void local_features(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  {
    
	  // Object for storing the normals.
	  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	  // Object for storing the PFH descriptors for each point.
	  pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors(new pcl::PointCloud<pcl::PFHSignature125>());

	  // Note: you would usually perform downsampling now. It has been omitted here
	  // for simplicity, but be aware that computation can take a long time.

	  // Estimate the normals.
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	  normalEstimation.setInputCloud(cloud);
	  normalEstimation.setRadiusSearch(0.03);
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	  normalEstimation.setSearchMethod(kdtree);
	  normalEstimation.compute(*normals);

	  // PFH estimation object.
	  pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	  pfh.setInputCloud(cloud);
	  pfh.setInputNormals(normals);
	  pfh.setSearchMethod(kdtree);
	  // Search radius, to look for neighbors. Note: the value given here has to be
	  // larger than the radius used to estimate the normals.
	  pfh.setRadiusSearch(0.05);

	  pfh.compute(*descriptors);
    
  }

  void greedy_projection(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
  {
    // Load input file into a PointCloud<T> with an appropriate type
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.025);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();  
  
    pcl::io::saveVTKFile("mesh.vtk", triangles);

  }

  void loadPoints(std::string filename, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud )
  {
    cout<<filename<<endl;
    try
    {
      pcl::io::loadPCDFile<pcl::PointXYZRGBA>(filename.c_str(), cloud);
      hasColor = true;
    }
    catch (pcl::PCLException e1)
    {
      try
      {
        // ...and if it fails, fall back to just depth.
        pcl::io::loadPCDFile<pcl::PointXYZ>(filename.c_str(), cloud);
      }
      catch (pcl::PCLException e2)
      {
        return;
      }

      hasColor = false;
    }

    cout << "Loaded " << filename << "." << endl;
    if (!hasColor)
      cout << "This cloud has no RGBA color information present." << endl;
    else cout << "This cloud has RGBA color information present." << endl;
    
  }

};

int main (int argc, char** argv)
{
  string filename;

  if (pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		filename = argv[2];
		
	}  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  Surface surface;
  surface.loadPoints(filename, *cloud)
);
  
  return 0;
}

