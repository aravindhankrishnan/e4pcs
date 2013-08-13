#include <iostream>
using namespace std;

#include "io.h"
#include "typedefs.h"


#define foreach BOOST_FOREACH

int main (int argc, char *argv[])
{
  if (argc < 7) {
    cout << "Enter arguments..\n"
            "\t1) file name 1\n"
            "\t2) file type 1\n"
            "\t3) file name 2\n"
            "\t4) file type 2\n"
            "\t5) closest point threshold\n"
            "\t6) Output file name\n"
            "\n";
    return -1;
  }

  string filename1 = argv[1];
  string filetype1 = argv[2];
  string filename2 = argv[3];
  string filetype2 = argv[4];
  float threshold = atof (argv[5]);
  string outfile = argv[6];

  CloudPtr cloud1 = readPointCloud (filename1, filetype1);
  CloudPtr cloud2 = readPointCloud (filename2, filetype2);

  KdTreePtr kdtree (new KdTree);
  kdtree->setInputCloud (cloud1);

  int K = 1;
  vector <int> ids (K);
  vector <float> dists (K);

  CloudPtr newcloud (new Cloud);
  foreach (Point& pt, cloud2->points) {
    if (kdtree->nearestKSearch (pt, K, ids, dists) > 0) {
      if (dists[0] < threshold) {
        newcloud->points.push_back (pt);
      }
    }
  }

  newcloud->width = 1;
  newcloud->height = newcloud->points.size ();

  writePointCloud (newcloud, outfile);

  return 0;
}
