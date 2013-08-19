#include "roughness_measure.h"

#include <iostream>
#include <numeric>
using namespace std;

#define foreach BOOST_FOREACH

void RoughnessMeasureRMS::computeRoughness ()
{
  Eigen::Vector4f plane_eq;
  plane_eq.setZero ();

  findMaxPlane (plane_eq);

  float sum_dist = 0.;
  float pd = numeric_limits <float>::min (); // perpendicular disatance
  for (int i = 0; i < cloud->points.size (); i++) {
    Eigen::Vector4f q = cloud->points[i].getVector4fMap ();
    float l = fabs (plane_eq.dot (q));
    sum_dist += l;

    if (l > pd) {
      pd = l;
    }
  }
  cout << "Longest perpendicular distance from Max plane = " << pd << endl;
  cout << "Sum of distances = " << sum_dist << endl;

  projected_cloud.reset (new Cloud);
  project3dto2d (cloud, projected_cloud, plane_eq); 

  CloudPtr tmp_cloud (new Cloud);
  tmp_cloud = removeAffineComponents (projected_cloud);

  area = findArea (tmp_cloud);
  cout << "\t[[ Area = " << area  << " sq. m =  " 
       << area / 1e6 << " sq. km ]]\n";
  cout << "\n\nRoughness measure = " << sum_dist / area << "\n\n";
}

void RoughnessMeasureRMS::findMaxPlane (Eigen::Vector4f& plane_eq)
{
  int z = 0;
  int max_pts = 0;

  srand (time (NULL));
  int N = cloud->points.size ();
  while (z++ < random_tries) {
    int a = 0, b = 0, c = 0;

    a = rand () % N;
    b = rand () % N;
    c = rand () % N;

    Eigen::Vector4f tmp_peq;
    tmp_peq.setZero ();
    findPlaneEquation (cloud, a, b, c, tmp_peq);

    int count = countPointsOnPlane (cloud, tmp_peq); 
    //cout << count << " ";

    if (count > max_pts) {
      max_pts = count;
      plane_eq.setZero ();
      plane_eq = tmp_peq;
    }
  }
  cout << endl;

  cout << "Plane equation " << plane_eq.transpose () << endl;
}

void RoughnessMeasureRMS::project3dto2d (CloudPtr cloud, CloudPtr projected_cloud,
                                         Eigen::Vector4f& plane_eq)
{
  //http://stackoverflow.com/questions/8942950/how-do-i-find-the-orthogonal-projection-of-a-point-onto-a-plane
  Eigen::Vector3f n = plane_eq.block <3, 1> (0, 0);
  Eigen::Vector3f p;
  findCentroidOfPlanePts (plane_eq, cloud, p);


  for (int i = 0; i < cloud->points.size (); i++) {
    Eigen::Vector3f q = cloud->points[i].getVector3fMap ();
    Eigen::Vector3f qproj = q - ((q - p).dot (n) * n);
    Point tp (qproj (0), qproj (1), qproj (2));
    //cout << qproj (2) << " ";
    projected_cloud->points.push_back (tp);
  }
  cout << endl;
  projected_cloud->width = 1;
  projected_cloud->height = projected_cloud->points.size ();
}

CloudPtr RoughnessMeasureRMS::removeAffineComponents (CloudPtr projected_cloud)
{
  Eigen::Vector3f centroid;
  centroid.setZero ();
  for (int i = 0; i < cloud->points.size (); i++) {
    centroid += projected_cloud->points[i].getVector3fMap ();
  }
  centroid /= projected_cloud->points.size ();

  Eigen::Matrix3f covariance;
  covariance.setZero ();
  for (int i = 0; i < projected_cloud->points.size (); i++) {
    Eigen::Vector3f t = projected_cloud->points[i].getVector3fMap () -
                        centroid;
    covariance += (t * t.transpose ());
  }

  //cout << "--- Covariance ---\n" << covariance << "\n\n";

  Eigen::EigenSolver <Eigen::Matrix3f> es (covariance);
  //cout << "The eigen values are " << es.eigenvalues () << endl;
  //cout << "\n\n\n" << endl;

  Eigen::Vector3d eigenvalues;
  eigenvalues.setZero ();
  pcl::eigen33 (covariance, eigenvalues);
  //cout << "The eigen values are " << eigenvalues.transpose () << endl << endl;

  Eigen::Vector3f eigenvector1;
  eigenvector1.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [2], eigenvector1);

  Eigen::Vector3f eigenvector2;
  eigenvector2.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [1], eigenvector2);

  Eigen::Vector3f eigenvector3;
  eigenvector3.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [0], eigenvector3);

  CloudPtr tmp_cloud (new Cloud);
  pcl::copyPointCloud (*projected_cloud, *tmp_cloud);
  for (int i = 0; i < tmp_cloud->points.size (); i++) {
    Point& pt = tmp_cloud->points[i];
    float x = pt.getVector3fMap ().dot (eigenvector1);
    float y = pt.getVector3fMap ().dot (eigenvector2);
    float z = pt.getVector3fMap ().dot (eigenvector3);
    pt.x = x, pt.y = y, pt.z = z;
  }

  return tmp_cloud;

}

float RoughnessMeasureRMS::findArea (CloudPtr cloud)
{
  float minx, maxx, miny, maxy;
  findMinMax (cloud, minx, miny, maxx, maxy);

  int nx = ceil ((maxx-minx) / resolution);
  int ny = ceil ((maxy-miny) / resolution);
  cout << "nx = " << nx << " ny = " << ny << endl;

  vector <vector <int> > partitions;
  partitions.reserve (nx);
  for (int i = 0; i < nx; i++) {
    partitions.push_back (vector <int> ());
  }

  for (int i = 0; i < nx; i++) {
    vector<int> &vec = partitions[i];
    for (int j = 0; j < ny; j++) {
      int v;
      vec.push_back (v);
    }
  }

	int N = cloud->points.size ();

	for (size_t i = 0; i < N; i++) {
		int ix = 0, iy = 0;
		float x = cloud->points[i].x;
		float y = cloud->points[i].y;
		ix = int (floor ( (x-minx) / resolution ) );
		iy = int (floor ( (y-miny) / resolution ) );

		int& count = partitions[ix][iy];
    count++;
	}

  float area = 0.0;
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
      if (partitions[i][j] > 10) {
        area += resolution * resolution;
      }
    }
  }
  return area;
}

void RoughnessMeasureRMS::findMinMax (CloudPtr cloud, float& minx, float& miny, 
                                      float& maxx, float& maxy)
{
	minx = cloud->points[0].x;
	maxx = cloud->points[0].x;
	miny = cloud->points[0].y;
	maxy = cloud->points[0].y;

	size_t N = cloud->points.size ();

	for (size_t i=0; i<N; i++) {
		if (cloud->points[i].x > maxx) {
			maxx = cloud->points[i].x;
		}
		if (cloud->points[i].x < minx) {
			minx = cloud->points[i].x;
		}
		if (cloud->points[i].y > maxy) {
			maxy = cloud->points[i].y;
		}
		if (cloud->points[i].y < miny) {
			miny = cloud->points[i].y;
		}
	}
}

void RoughnessMeasureRMS::findCentroidOfPlanePts (Eigen::Vector4f& plane_eq, CloudPtr cloud,
                                                  Eigen::Vector3f& p)
{
  int count = 0;
  p.setZero ();

  for (int i = 0; i < cloud->points.size (); i++) {
    Eigen::Vector4f pt = cloud->points[i].getVector4fMap ();

    float residue = pt.dot (plane_eq);
    if ( residue < plane_fit_threshold) {
      Eigen::Vector3f t = pt.block <3, 1> (0, 0);
      p += t;
      count++;
    }
  }
  p /= count;
}

int RoughnessMeasureRMS::countPointsOnPlane (CloudPtr cloud, 
                                             Eigen::Vector4f& plane_eq)
{
  int count = 0;
  for (int i = 0; i < cloud->points.size (); i++) {
    Point pt = cloud->points[i];
    float residue = fabs (pt.getVector4fMap ().dot (plane_eq));
    
    if (residue < plane_fit_threshold) {
      count++;
    }
  }
  return count;
}

void RoughnessMeasureRMS::findPlaneEquation (CloudPtr cloud, int a, int b, int c, 
                                             Eigen::Vector4f& eq)
{
  Point pa = cloud->points[a];
  Point pb = cloud->points[b];
  Point pc = cloud->points[c];

  Eigen::Vector3f centroid;
  centroid.setZero ();

  centroid (0) = (pa.x + pb.x + pc.x ) / 3.0;
  centroid (1) = (pa.y + pb.y + pc.y ) / 3.0;
  centroid (2) = (pa.z + pb.z + pc.z ) / 3.0;

  Eigen::Matrix3f covariance;
  covariance.setZero ();

  Eigen::Vector3f V;
  V.setZero ();
  V = (pa.getVector3fMap () - centroid) ;
  covariance += (V * V.transpose ());

  V.setZero ();
  V = (pb.getVector3fMap () - centroid);
  covariance += (V * V.transpose ());

  V.setZero ();
  V = (pc.getVector3fMap () - centroid);
  covariance += (V * V.transpose ());

  Eigen::EigenSolver <Eigen::Matrix3f> es (covariance);
  //cout << "The eigen values are " << es.eigenvalues () << endl;
  //cout << "\n\n\n" << endl;

  Eigen::Vector3d eigenvalues;
  eigenvalues.setZero ();
  pcl::eigen33 (covariance, eigenvalues);
  //cout << "The eigen values are " << eigenvalues.transpose () << endl << endl;

  Eigen::Vector3f eigenvector1;
  eigenvector1.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [2], eigenvector1);

  Eigen::Vector3f eigenvector2;
  eigenvector2.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [1], eigenvector2);

  Eigen::Vector3f eigenvector3;
  eigenvector3.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [0], eigenvector3);

  eq.block <3,1> (0, 0) = eigenvector3;
  eq (3) = -(centroid.dot (eigenvector3));
}


void RoughnessMeasureHoPD::computeRoughness ()
{
  Eigen::Vector4f plane_eq;
  plane_eq.setZero ();

  findMaxPlane (plane_eq);

  vector <int> histogram;
  histogram.resize (200);

  float pd = numeric_limits <float>::min (); // perpendicular disatance
  for (int i = 0; i < cloud->points.size (); i++) {
    Eigen::Vector4f q = cloud->points[i].getVector4fMap ();
    float l = fabs (plane_eq.dot (q));

    histogram[(int)(floor (l))]++;

    if (l > pd) {
      pd = l;
    }
  }

  cout << "Histogram\n";
  foreach (int& i, histogram) {
    cout << i << " ";
  }
  cout << endl;

  vector <float> probabilities;
  copy (histogram.begin (), histogram.end (), back_inserter (probabilities));
  int N = cloud->points.size ();
  foreach (float& f, probabilities) {
    f /= N;
  }
  cout << "Probabilities\n";
  foreach (float& f, probabilities) {
    cout << f << " ";
  }
  cout << endl;

  float entropy = 0.;
  foreach (float& p, probabilities) {
    if (p) {
      entropy += (-p * log (p));
    }
  }

  entropy /= probabilities.size ();

  cout << "\nEntropy = " << entropy << endl;
}


void RoughnessMeasureEoC::computeRoughness ()
{
  CloudNormalPtr cloud_normals (new CloudNormal);
  pcl::copyPointCloud (*cloud, *cloud_normals);
	pcl::NormalEstimation <Point, PointNormal> ne;
	ne.setInputCloud (cloud);
	KdTreePtr tree (new KdTree);
	ne.setSearchMethod (tree);
  ne.setKSearch (10);
	ne.compute (*cloud_normals);

  int N = 80;
  float step = 1.0 / (double) N;
  cout << "Step = " << step << endl;
  vector <float> p;
  p.resize (N);
  fill_n (p.begin (), N, 0.0);

  foreach (PointNormal& pt, cloud_normals->points) {
    float curvature = pt.curvature;
    int bin = (int) ceil (curvature/step);
    p[bin]++;
  }

  float sum = std::accumulate (p.begin (), p.end (), 0.0);
  foreach (float& f, p) {
    f /= sum;
  }

  cout << "Sum of probabilities = " << accumulate (p.begin (), p.end (), 0.0) << endl;
  cout << endl;

  float entropy = 0.;
  foreach (float& f, p) {
    if (f) {
      entropy += (-f * log (f));
    }
  }

  entropy /= cloud_normals->points.size ();

  cout << "\nEntropy = " << entropy << endl;
}
