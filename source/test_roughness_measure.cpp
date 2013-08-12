#include <iostream>
using namespace std;

#include "roughness_measure.h"
#include "io.h"

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;


void displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
                        char* name, int& viewport)
{
  if (!cloud) {
    return;
  }

  PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
  viz->addPointCloud (cloud, tgt_h, name, viewport);
}


int main (int argc, char *argv[])
{
  if (argc < 4) {
    cout << "Enter arguments ..\n"
            "\t1) File name\n"
            "\t2) File type\n"
            "\t3) Roughness method (RMS / HoPD / EoC)\n"
            "\n";
    return -1;
  }

  string filename = argv[1];
  string filetype = argv[2];
  string method = argv[3];

  CloudPtr cloud (new Cloud);
  if (filetype.compare ("ascii") == 0) {
    readASCIIFile (filename.c_str (), cloud);
  }
  else if (filetype.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (filename.c_str (), cloud);
  }
  else {
    cout << "\nUnknown file type provided..\n";
    cout << "Options (ascii / pcdbinary)\n\n";
    return -1;
  }

  RoughnessMeasurePtr rm;

  if (method.compare ("RMS") == 0) {
    rm.reset (new RoughnessMeasureRMS ());
  }
  else if (method.compare ("HoPD") == 0) {
    rm.reset (new RoughnessMeasureHoPD ());
  }
  else if (method.compare ("EoC") == 0) {
    rm.reset (new RoughnessMeasureEoC ());
  }
  else {
    cout << "\nUnknown input roughness measure provided ..\n";
    cout << "Options (RMS / HoPD / EoC)\n\n";
    return -1;
  }

  rm->setInputCloud (cloud);
  rm->computeRoughness ();

  boost::shared_ptr <PCLVisualizer> viz (new PCLVisualizer);

  int vp1 = 1, vp2 = 2;
  viz->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  viz->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);
  viz->setBackgroundColor (113.0/255, 113.0/255, 154.0/255, 0);

  int color[3] = {255, 0, 0};
  displayPointCloud (viz.get (), cloud, color, (char *) "opcloud1", vp1);

  if (method.compare ("RMS") == 0) {
    CloudPtr projected_cloud = ((RoughnessMeasureRMS*)rm.get ())->getProjectedCloud ();
    color[2] = 255;
    displayPointCloud (viz.get (), projected_cloud, color, (char *) "opcloud2", vp2);
  }

  viz->spin ();

  return 0;
}
