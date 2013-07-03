#include <iostream>
using namespace std;

#include "io.h"
#include "typedefs.h"


#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

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


  CloudPtr sampledcloud1 (new Cloud);
  pcl::ApproximateVoxelGrid<Point> avg;
  avg.setLeafSize (5, 5, 5);
  avg.setInputCloud (cloud1);
  avg.filter (*sampledcloud1);
  std::cout << "Filtered cloud contains " << sampledcloud1->size ()
            << " data points from " << sourcefile << endl;



  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<Point, Point> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (1e-5);

  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);

  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (5.0);

  ndt.setMaximumIterations (500);

  //ndt.setInputSource (sampledcloud1);
  ndt.setInputSource (cloud1);

  ndt.setInputTarget (cloud2);


  //Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  //Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  //Eigen::Matrix4f init_guess_test = (init_translation * init_rotation).matrix ();
  //cout << init_guess_test << endl;
  //Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity ();


  CloudPtr output (new Cloud);
  ndt.align (*output, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  //pcl::transformPointCloud (*cloud1, *output, ndt.getFinalTransformation ());

  cout << "\n\n------ Initial Guess -----------\n";
  cout << init_guess << "\n\n";

  cout << "--------- Final transformation ---------------\n";
  cout << ndt.getFinalTransformation () << "\n\n";


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
