#include <iostream>
using namespace std;

#include "typedefs.h"
#include "io.h"
#include "config.h"
#include "keypoints_interface.h"

#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;


void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
{
  if (cloud->points.size () <= N) {
    pcl::copyPointCloud (*cloud, *sampledcloud);
    sampledcloud->width = 1;
    sampledcloud->height = sampledcloud->points.size ();
  }
  else {
    srand (time (NULL));
    for (int i = 0; i < N; i++) {
      int k = rand () % cloud->points.size ();
      sampledcloud->points.push_back (cloud->points[k]);
    }
    sampledcloud->height = 1;
    sampledcloud->width = sampledcloud->points.size ();
  }
}

int main (int argc, char *argv[])
{
  if (argc < 2) {
    cout << "\nEnter the configuration file ..\n";
    return -1;
  }

  if (loadConfigFile (argv[1]) == -1) {
    return -1;
  }
  
  InputParamsPtr args = getInputParams ();

  cout << "Keypoint type = " << args->keypoint_type << endl;
  CloudPtr cloud ( new Cloud );

  string infile = args->sourcefile;
  if (args->filetype1.compare ("ascii") == 0) {
    readASCIIFile (infile.c_str (), cloud);
  }
  else if (args->filetype1.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (infile.c_str (), cloud);
  }

  KeyPointsInterface kpi;
  kpi.setKeyPointType (args->keypoint_type);
  kpi.setParams (args->keypoint_par);

  CloudPtr sampledcloud (new Cloud);
  sampleCloud (cloud, args->vis_num_points, sampledcloud);
  cout << "# of points displayed = " << sampledcloud->points.size () << endl;

  //kpi.setInputCloud (sampledcloud);
  kpi.setInputCloud (cloud);
  kpi.compute ();
  
  CloudPtr keypoints (new Cloud);
  keypoints = kpi.getKeypoints ();

  cout << "# of keypoints = " << keypoints->points.size () << endl;

  PCLVisualizer* viz = new PCLVisualizer (argc, argv, 
                                          "Keypoints Visualization");
  viz->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);

  int color[3] = {255, 255, 255};
  PointCloudColorHandlerCustom <Point> tgt_h1 (cloud, color[0], color[1], color[2]);
  viz->addPointCloud (cloud, tgt_h1, "cloud");
  //viz->addPointCloud (sampledcloud, tgt_h1, "cloud");
  //viz->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
  //                                        10, "cloud");

  color[1] = color[2] = 0;
  PointCloudColorHandlerCustom <Point> tgt_h2 (keypoints, color[0], color[1], color[2]);
  viz->addPointCloud (keypoints, tgt_h2, "keypoints");
  viz->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
                                          3, "keypoints");
  writePointCloud (keypoints, "keypoints.asc");

  viz->spin ();

  return 0;
}
