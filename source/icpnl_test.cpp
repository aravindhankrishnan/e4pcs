#include <iostream>
using namespace std;

#include "io.h"
#include "typedefs.h"


#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;


void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
{
  if (cloud->points.size () <= N) {
    sampledcloud = cloud;
  }
  else {
    srand (time (NULL));
    for (int i = 0; i < N; i++) {
      int k = rand () % cloud->points.size ();
      sampledcloud->points.push_back (cloud->points[k]);
    }
    sampledcloud->width = 1;
    sampledcloud->height = sampledcloud->points.size ();
  }
}

void displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
                        char* name, int& viewport)
{
  PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
  viz->addPointCloud (cloud, tgt_h, name, viewport);
}



int main (int argc, char *argv[])
{

  if (argc < 3) {
    cout << "Enter the two files for registration ..\n";
    return -1;
  }

  pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

  string sourcefile = argv[1];
  string targetfile = argv[2];

  CloudPtr cloud1 ( new Cloud );
  CloudPtr cloud2 ( new Cloud );

  readPCDBinaryFile (sourcefile.c_str (), cloud1);
  readPCDBinaryFile (targetfile.c_str (), cloud2);


  //pcl::IterativeClosestPointNonLinear <Point, Point> icp;
  pcl::IterativeClosestPoint <Point, Point> icp;
  icp.setInputSource (cloud1);
  icp.setInputTarget (cloud2);
  icp.setMaximumIterations (2500);
  icp.setTransformationEpsilon (0.0000001);


  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  //Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity ();
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  CloudPtr output (new Cloud);
  icp.align (*output, init_guess);

  //pcl::transformPointCloud (*cloud1, *output, icp.getFinalTransformation ());

  std::cout << "ICP NL has converged:" << icp.hasConverged ()
            << " score: " << icp.getFitnessScore () << std::endl;

  cout << "--------- Final transformation ---------------\n";
  cout << icp.getFinalTransformation () << "\n\n";


  CloudPtr cloud1_ig (new Cloud);
  pcl::transformPointCloud (*cloud1, *cloud1_ig, init_guess);

  PCLVisualizer* p = new PCLVisualizer (argc, argv, "Registration");
  int vp1 = 1;
  p->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  int vp2 = 2;
  p->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);
  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);


  int color[3] = { 255, 0, 0};
  displayPointCloud (p, cloud1_ig, color, (char *) "opcloud1", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (p, cloud2, color, (char *) "opcloud2", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (p, output, color, (char *) "opcloud11", vp2);

  color[0] = 255; color[1] = 0; color[2] = 0;
  displayPointCloud (p, cloud2, color, (char *) "opcloud12", vp2);

  p->spin ();

  return 0;
}

//--------- Final transformation ---------------
//  0.753716  -0.656835  0.0219197    2.01435
//  0.656678   0.754027  0.0147124  0.0698813
//-0.0261917 0.00330526   0.999651  0.0253391
//         0          0          0          1
