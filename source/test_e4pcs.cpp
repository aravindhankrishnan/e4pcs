#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "io.h"
#include "e4pcs.h"
#include "typedefs.h"

#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

float sphere_radius;

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


void addGaussianNoise (CloudPtr cloud, float sd)
{
  using namespace boost;
  mt19937 rng (time (0));
  normal_distribution<> nd (0.0, sd);

  variate_generator <mt19937&, normal_distribution<> > noise (rng, nd);


  BOOST_FOREACH (Point& pt, cloud->points) {
    float nx = noise ();
    float ny = noise ();
    float nz = noise ();

    pt.x += nx;
    pt.y += ny;
    pt.z += nz;
  }
}


int main (int argc, char *argv[])
{
  if (argc < 6) {
    cout << "Enter arguments\n"
             "1) source cloud\n"
             "2) target cloud\n"
             "3) Number of quads\n"
             "4) Number of points to be sampled\n"
             "4) Sphere radius\n";
    return -1;
  }

  sphere_radius = atof (argv[5]);

  using namespace E4PCS;

  CloudPtr cloud1 ( new Cloud );
  CloudPtr cloud2 ( new Cloud );


  //readPCDBinaryFile (argv[1], cloud1);
  //readPCDBinaryFile (argv[2], cloud2);

  readASCIIFile (argv[1], cloud1);
  readASCIIFile (argv[2], cloud2);

  PCLVisualizer* p = new PCLVisualizer (argc, argv, "Choosing plane");

  int vp1 = 1;
  p->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  //p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255, vp1);

  int vp2 = 2;
  p->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);
  //p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255, vp2);

  //p->setBackgroundColor (25./255, 25.0/255, 25.0/255);
  //p->setBackgroundColor (255, 255, 255, vp2);

  float D = 5.0;
  float sd = D / 6.; // the point can lie in a 'D/2' m radius. so 6 sigma = D.
  addGaussianNoise (cloud2, sd);

  CloudPtr sampledcloud1 (new Cloud);
  CloudPtr sampledcloud2 (new Cloud);

  int numPoints = atoi (argv[4]);
  sampleCloud (cloud1, numPoints, sampledcloud1);
  sampleCloud (cloud2, numPoints, sampledcloud2);


  Extended4PCS e4pcs (D);
  e4pcs.setSource (sampledcloud1);
  e4pcs.setTarget (sampledcloud2);
  e4pcs.setVisualizer (p);
  e4pcs.setNumQuads (atoi (argv[3]));

  e4pcs.align (); 

  Eigen::Matrix4f transformation;

  e4pcs.getTransformation (transformation);

  cout << "\n\n";

  cout << "# of points in cloud 1 = " << cloud1->points.size () << endl;
  cout << "# of points in cloud 2 = " << cloud2->points.size () << endl;
  cout << "# of points on the MAX plane = " << e4pcs.getPlanePoints ().size () << endl;

  ofstream ofile ("transform.txt");
  ofile << transformation << "\n";
  ofile.close ();


  cout << "\n\nVisualizing ..\n\n";

  PCLVisualizer* final = new PCLVisualizer (argc, argv, "Final output");


  CloudPtr op_cloud1 (new Cloud);
  CloudPtr op_cloud2 (new Cloud);
  numPoints = 20000;
  sampleCloud (cloud1, numPoints, op_cloud1);
  sampleCloud (cloud2, numPoints, op_cloud2);

  cout << "# points in sampled cloud 1 = " << op_cloud1->points.size () << endl;
  cout << "# points in sampled cloud 2 = " << op_cloud2->points.size () << endl;

  //int vp1 = 1, vp2 = 2;
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




  p->spin ();

  return 0;
}
