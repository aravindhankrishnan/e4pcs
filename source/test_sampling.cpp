#include <iostream>
using namespace std;

#include "io.h"
#include "typedefs.h"

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

void displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
                        char* name, int& viewport)
{
  PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
  viz->addPointCloud (cloud, tgt_h, name, viewport);
}

int main (int argc, char *argv[])
{
  if (argc < 3) {
    cout << "\nEnter\n"
            "\t1) Input cloud\n"
            "\t2) Type (ascii / pcdbinary)\n"
            "\n\n";
    return -1;
  }

  string sourcefile = argv[1];
  string filetype = argv[2];

  CloudPtr cloud (new Cloud);

  if (filetype.compare ("ascii") == 0) {
    readASCIIFile (sourcefile.c_str (), cloud);
  }
  else if (filetype.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (sourcefile.c_str (), cloud);
  }

  PCLVisualizer* p = new PCLVisualizer (argc, argv, "Uniform Sampling");

  int vp1 = 1;
  p->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);

  int vp2 = 2;
  p->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);

  CloudPtr sampledcloud (new Cloud);

  pcl::UniformSampling<Point> us;
  us.setInputCloud (cloud);
  double radius = 30;
  us.setRadiusSearch (radius);
  pcl::PointCloud<int> subsampled_indices;
  us.compute (subsampled_indices);
  std::sort (subsampled_indices.points.begin (), subsampled_indices.points.end ());
  pcl::copyPointCloud (*cloud, subsampled_indices.points, *sampledcloud);

  cout << "Points = " << cloud->points.size () << " "
    << sampledcloud->points.size () << endl;

  int color[3] = { 255, 255, 255};

  displayPointCloud (p, cloud, color, (char *) "opcloud1", vp1);
  displayPointCloud (p, sampledcloud, color, (char *) "opcloud2", vp2);

  p->spin ();
  return 0;
}
