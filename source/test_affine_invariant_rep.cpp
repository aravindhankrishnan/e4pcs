#include <iostream>
using namespace std;

#include "io.h"
#include "typedefs.h"
#include "affine_invariant_representation.h"

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
  
  if (argc < 5) {
    cout << "Enter arguments..\n"
             "1) Cloud 1\n"
             "2) filetype [ascii / pcdbinary]\n"
             "2) Cloud 2\n"
             "4) filetype [ascii / pcdbinary]\n";
    return -1;
  }

  using namespace E4PCS;

  string sourcefile = argv[1];
  string filetype1 = argv[2];
  string targetfile = argv[3];
  string filetype2 = argv[4];

  CloudPtr cloud1 (new Cloud);
  CloudPtr cloud2 (new Cloud);

  if (filetype1.compare ("ascii") == 0) {
    readASCIIFile (sourcefile.c_str (), cloud1);
  }
  else if (filetype1.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (sourcefile.c_str (), cloud1);
  }

  if (filetype2.compare ("ascii") == 0) {
    readASCIIFile (targetfile.c_str (), cloud2);
  }
  else if (filetype2.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (targetfile.c_str (), cloud2);
  }


  AffInvRep air;
  air.setCloud1 (cloud1);
  air.setCloud2 (cloud2);
  air.compute ();

  CloudPtr aircloud1 (new Cloud);
  CloudPtr aircloud2 (new Cloud);

  aircloud1 = air.getAIRCloud1 ();
  aircloud2 = air.getAIRCloud2 ();

  int vp1 = 1, vp2 = 2;

  PCLVisualizer* pviz = new PCLVisualizer (argc, argv, 
                            "Affine Invariant Representation");

  pviz->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  pviz->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  pviz->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);


  int color[3] = { 255, 0, 0};
  displayPointCloud (pviz, cloud1, color, (char *) "opcloud1", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (pviz, cloud2, color, (char *) "opcloud2", vp1);

  color[0] = 255; color[1] = 0; color[2] = 0;
  displayPointCloud (pviz, aircloud1, color, (char *) "opcloud11", vp2);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (pviz, aircloud2, color, (char *) "opcloud12", vp2);

  pviz->addCoordinateSystem (1.0);

  pviz->spin ();
  return 0;
}
