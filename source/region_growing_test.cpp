#include <iostream>
#include <vector>
using namespace std;

#include "typedefs.h"
#include "io.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;

int main (int argc, char* argv[])
{
  if (argc < 3) {
    cout << "Enter arguments\n"
            "\t1) Input file\n"
            "\t2) type (ASCII / pcdbinary) \n\n";
    return -1;
  }

  CloudPtr cloud (new Cloud);
  
  string sourcefile = argv[1];
  string type = argv[2];

  if (type.compare ("ascii") == 0) {
    readASCIIFile (sourcefile.c_str (), cloud);
  }
  else if (type.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (sourcefile.c_str (), cloud);
  }


  pcl::search::Search<Point>::Ptr tree = 
    boost::shared_ptr<pcl::search::Search<Point> > (new pcl::search::KdTree<Point>);

  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<Point, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (10); //50
  ne.compute (*normals);

  //pcl::IndicesPtr indices (new std::vector <int>);
  //pcl::PassThrough<Point> pass;
  //pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.filter (*indices);

  pcl::RegionGrowing<Point, pcl::Normal> reg;
  reg.setMinClusterSize (10); // 100
  reg.setMaxClusterSize (10000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (10); //30
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI); // 7.0
  reg.setCurvatureThreshold (0.5); // 1.0

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud  = reg.getColoredCloud ();

  PCLVisualizer* p = new PCLVisualizer (argc, argv, "Segmentation");
  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);

  p->addPointCloud (colored_cloud, "segmented_cloud");

  p->spin ();


  //pcl::visualization::CloudViewer viewer ("Cluster viewer");
  //viewer.showCloud (colored_cloud);
  //while (!viewer.wasStopped ())
  //{
  //}

  return 0;
}
