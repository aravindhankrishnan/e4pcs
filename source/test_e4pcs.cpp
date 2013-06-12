#include <iostream>
#include <sstream>
using namespace std;

#include "io.h"
#include "e4pcs.h"

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

  CloudPtr sampledcloud1 (new Cloud);
  CloudPtr sampledcloud2 (new Cloud);

  int numPoints = atoi (argv[4]);
  sampleCloud (cloud1, numPoints, sampledcloud1);
  sampleCloud (cloud2, numPoints, sampledcloud2);


  Extended4PCS e4pcs;
  e4pcs.setSource (sampledcloud1);
  e4pcs.setTarget (sampledcloud2);
  e4pcs.setVisualizer (p);
  e4pcs.setNumQuads (atoi (argv[3]));

  e4pcs.align (); 

  cout << "\n\n";

  cout << "# of points on the plane = " << e4pcs.getPlanePoints ().size () << endl;

  p->spin ();

  return 0;
}
