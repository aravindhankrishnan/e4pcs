#include "partial_overlap.h"

#include <vector>
using namespace std;

#define foreach BOOST_FOREACH

float PartialOverlap::findOverlapRatio ()
{
  if (!cloud1 or !cloud2) {
    throw std::runtime_error ("One of the input clouds not set");
  }

  if (threshold == 0.) {
    throw std::runtime_error ("Threshold not set");
  }

  KdTreePtr kdtree (new KdTree);

  CloudPtr t1 = (cloud1->points.size () < cloud2->points.size ()) ? cloud1 : cloud2;
  kdtree->setInputCloud (t1);

	int K = 1;
	vector<int> ids (K);
	vector<float> dist (K);


  CloudPtr t2 = (cloud1->points.size () > cloud2->points.size ()) ? cloud1 : cloud2;

  int count = 0;

  foreach (Point& pt, t2->points) {
    if(kdtree->nearestKSearch(pt, K, ids, dist) > 0) {
      if (dist[0] < threshold) {
        count++;
      }
    }
  }

  overlap_ratio = (float)count / t1->points.size ();
  return overlap_ratio;
}
