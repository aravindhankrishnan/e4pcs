#include <iostream>
#include <fstream>
using namespace std;

#include "typedefs.h"
#include "io.h"

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
{
  if (cloud->points.size () <= N) {
    sampledcloud = cloud;
    return;
  }

  srand (time (NULL));
  for (int i = 0; i < N; i++) {
    int k = rand () % cloud->points.size ();
    sampledcloud->points.push_back (cloud->points[k]);
  }
  sampledcloud->width = 1;
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
    cout << "Enter arguments..\n"
             "1) Cloud 1\n"
             "2) Cloud 2\n"
             "3) Transformation matrix\n"
             "4) Num points to display\n\n";
    return -1;
    
  }
  PCLVisualizer* final = new PCLVisualizer (argc, argv, "Final output");


  CloudPtr cloud1 (new Cloud);
  CloudPtr cloud2 (new Cloud);

  //readPCDBinaryFile (argv[1], cloud1);
  //readPCDBinaryFile (argv[2], cloud2);

  readASCIIFile (argv[1], cloud1);
  readASCIIFile (argv[2], cloud2);

  cout << "# points in cloud 1 = " << cloud1->points.size () << endl;
  cout << "# points in cloud 2 = " << cloud2->points.size () << endl;

  ifstream ifile (argv[3]);

  if (!ifile) {
    cout << "Couldn't open file " << argv[3] << endl;
    return -1;
  }

  Eigen::Matrix4f transformation;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      double val = 0.;
      ifile >> val;
      transformation (i, j) = val;
    }
  }
  cout << transformation << endl;


  CloudPtr op_cloud1 (new Cloud);
  CloudPtr op_cloud2 (new Cloud);
  int numPoints = atoi (argv[4]);
  sampleCloud (cloud1, numPoints, op_cloud1);
  sampleCloud (cloud2, numPoints, op_cloud2);

  cout << "# points in sampled cloud 1 = " << op_cloud1->points.size () << endl;
  cout << "# points in sampled cloud 2 = " << op_cloud2->points.size () << endl;

  int vp1 = 1, vp2 = 2;
  final->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  final->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  final->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);


  CloudPtr op_cloud3 (new Cloud);
  Eigen::Matrix3f R = transformation.block <3, 3> (0, 0);
  Eigen::Vector3f T = transformation.block <3, 1> (0, 3);

  BOOST_FOREACH (Point& pt, op_cloud1->points) {
    Eigen::Vector3f v = R * pt.getVector3fMap () + T;
    Point np (v (0), v (1), v (2));
    op_cloud3->points.push_back (np);
  }
  op_cloud3->width = 1;
  op_cloud3->height = op_cloud3->points.size ();


  cout << "# points in cloud 3 = " << op_cloud3->points.size () << endl;

  int vp0 = 0;
  int color[3] = { 255, 0, 0};
  displayPointCloud (final, op_cloud1, color, (char *) "opcloud1", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (final, op_cloud2, color, (char *) "opcloud2", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (final, op_cloud2, color, (char *) "opcloud11", vp2);

  color[0] = 255; color[1] = 0; color[2] = 0;
  displayPointCloud (final, op_cloud3, color, (char *) "opcloud12", vp2);


  final->spin ();
  return 0;
}
