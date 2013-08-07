#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "io.h"
#include "e4pcs.h"
#include "typedefs.h"

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

//float dx = 566001.63;
//float dy = 3682196.04;

//float dx = 513780.071;
//float dy = 3519794.27;

float dx = 0.;
float dy = 0.;

void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
{
  if (cloud->points.size () <= N) {
    sampledcloud = cloud;
  }
  else {
    srand (time (NULL));
    for (int i = 0; i < N; i++) {
      int k = rand () % cloud->points.size ();
      Point pt = cloud->points[k];
      pt.x -= dx;
      pt.y -= dy;
      sampledcloud->points.push_back (pt);
    }
    sampledcloud->width = 1;
    sampledcloud->height = sampledcloud->points.size ();
  }
}

int main (int argc, char *argv[])
{

  if (argc < 3) {
    cout << "Enter arguments ..\n"
            "\t1) Input cloud\n"
            "\t2) Number of points to be sampled\n"
            "\t3) Output cloud\n"
            "\n";
    return -1;
  }

  CloudPtr cloud1 ( new Cloud );

  readASCIIFile (argv[1], cloud1);

  CloudPtr sampledcloud1 (new Cloud);

  int numPoints = atoi (argv[2]);
  sampleCloud (cloud1, numPoints, sampledcloud1);

  writeASCIIFile (argv[3], sampledcloud1);

  return 0;
}
