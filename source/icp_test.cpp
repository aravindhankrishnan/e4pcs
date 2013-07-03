#include <iostream>
using namespace std;


#include "io.h"
#include "typedefs.h"

#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

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
    cout << "\nEnter arguments\n"
            "\t1) Source file\n"
            "\t2) type (ASCII / pcdbinary) \n"
            "\t3) Target file\n"
            "\t4) type (ASCII / pcdbinary) \n"
            "\t5) [Optional] initial guess matrix\n\n";
    return -1;
  }

  pcl::console::setVerbosityLevel (pcl::console::L_DEBUG);

  string sourcefile = argv[1];
  string sourcetype = argv[2];
  string targetfile = argv[3];
  string targettype = argv[4];

  CloudPtr cloud1 ( new Cloud );
  CloudPtr cloud2 ( new Cloud );

  if (sourcetype.compare ("ascii") == 0) {
    readASCIIFile (sourcefile.c_str (), cloud1);
  }
  else if (sourcetype.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (sourcefile.c_str (), cloud1);
  }

  if (targettype.compare ("ascii") == 0) {
    readASCIIFile (targetfile.c_str (), cloud2);
  }
  else if (targettype.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (targetfile.c_str (), cloud2);
  }

  Eigen::Matrix4f init_guess;
  if (argc == 6) {
    ifstream ifile (argv[5]);
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        float val;
        ifile >> val;
        init_guess (i, j) = val;
      }
    }
  }
  else {
    init_guess = Eigen::Matrix4f::Identity ();
  }

  CloudNormalPtr src (new CloudNormal);
  pcl::copyPointCloud (*cloud1, *src);
  CloudNormalPtr tgt (new CloudNormal);

  pcl::copyPointCloud (*cloud2, *tgt);

  pcl::NormalEstimation<PointNormal, PointNormal> norm_est;
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  norm_est.setKSearch (30);
  norm_est.setInputCloud (tgt);
  norm_est.compute (*tgt);

  pcl::IterativeClosestPoint <PointNormal, PointNormal> icp;
  typedef pcl::registration::TransformationEstimationPointToPlaneLLS <PointNormal, PointNormal> PointToPlane;
  boost::shared_ptr<PointToPlane> p2pl (new PointToPlane);
  icp.setTransformationEstimation (p2pl);
  icp.setInputSource (src);
  icp.setInputTarget (tgt);
  icp.setMaxCorrespondenceDistance (10);
  icp.setMaximumIterations (1000);
  icp.setTransformationEpsilon (1e-5);

  CloudNormalPtr output (new CloudNormal);
  icp.align (*output);


  std::cout << "ICP has converged:" << icp.hasConverged ()
            << " score: " << icp.getFitnessScore () << std::endl;

  //pcl::transformPointCloud (*cloud1, *output, ndt.getFinalTransformation ());


  cout << "\n\n------ Initial Guess -----------\n";
  cout << init_guess << "\n\n";

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
  CloudPtr tmp (new Cloud);
  pcl::copyPointCloud (*output, *tmp);
  displayPointCloud (p, tmp, color, (char *) "opcloud11", vp2);

  color[0] = 255; color[1] = 0; color[2] = 0;
  displayPointCloud (p, cloud2, color, (char *) "opcloud12", vp2);

  p->spin ();

  return 0;
}
