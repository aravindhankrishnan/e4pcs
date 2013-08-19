#include "keypoints_interface.h"

#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <functional>
#include <stdexcept>
using namespace std;

#define foreach BOOST_FOREACH

namespace E4PCS
{

void KeyPointsInterface::compute ()
{
  if (keypoint_type.compare ("sift") == 0) {
    computeSIFT ();
  }
  else if (keypoint_type.compare ("harris") == 0) {
    computeHarris ();
  }
  else if (keypoint_type.compare ("curvature") == 0) {
    computeCurvatureKeypoint ();
  }
  else if (keypoint_type.compare ("iss") == 0) {
    computeISS ();
  }
  else if (keypoint_type.compare ("curvaturelocalmaxima") == 0) {
    computeCurvatureLocalMaxima ();
  }
  else if (keypoint_type.compare ("principalcurvatures") == 0) {
    computePrincipalCurvatures ();
  }
  else if (keypoint_type.compare ("boundarypoints") == 0) {
    computeBoundaryPoints ();
  }
  cout << __PRETTY_FUNCTION__ << " :: # keypoints = " << keypoints->points.size () << endl;
}

void KeyPointsInterface::computeCurvatureKeypoint ()
{
  CurvatureKeypointParams& par = *( (CurvatureKeypointParams*) keypoint_par.get () );

  CloudNormalPtr normals (new CloudNormal);
  copyPointCloud (*cloud, *normals);

  pcl::NormalEstimation<Point, PointNormal> ne;
  KdTreePtr kdtree (new KdTree);


  ne.setInputCloud (cloud);
  ne.setSearchMethod (kdtree);
  //ne.setRadiusSearch (radius_search);
  ne.setKSearch (par.k_search);
  ne.compute (*normals);

  sort (normals->points.begin (), normals->points.end (), CurvatureSortFunctor ());

  for (int i = 0; i < par.num_points; i++) {
    //cout << normals->points[i].curvature << " ";
    PointNormal pt = normals->points[i];
    Point p (pt.x, pt.y, pt.z);
    keypoints->points.push_back (p);
  }
  //cout << endl;

  keypoints->width = 1;
  keypoints->height = keypoints->points.size ();
}

void KeyPointsInterface::computeHarris ()
{
  /*
  pcl::HarrisKeypoint3D <Point, PointIntensity>* harris=
    new pcl::HarrisKeypoint3D<Point, PointIntensity> (3);

  harris->setNonMaxSupression (true);

  harris->setRadius (5);
  harris->setRadiusSearch (5);
  //harris3D->setRefine (false);

  //harris->setMethod (pcl::HarrisKeypoint6D <Point, PointIntensity>::HARRIS);
  //harris3D->setMethod (pcl::HarrisKeypoint3D <Point, PointIntensity>::LOWE);
  //harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI>::NOBLE);
  //harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI>::TOMASI);
  //harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI>::CURVATURE);

  boost::shared_ptr< pcl::Keypoint<Point, PointIntensity> > keypoint_detector;
  keypoint_detector.reset (harris);

  keypoint_detector->setInputCloud (cloud);
  CloudIntensityPtr result (new CloudIntensity);
  keypoint_detector->compute (*result);
  cout << "# of keypoints = " << result->points.size () << endl;
  copyPointCloud (*result, *keypoints);
  */
}

void KeyPointsInterface::computePrincipalCurvatures ()
{
  PrincipalCurvatureParams& par = *( (PrincipalCurvatureParams*) keypoint_par.get () );
  int K = par.k_search;
  pcl::NormalEstimation<Point, PointNormal> ne;
  CloudNormalPtr cloud_normals (new CloudNormal);
  KdTreePtr kdtree (new KdTree);

  ne.setInputCloud (cloud);
  ne.setSearchMethod (kdtree);
  ne.setKSearch (K);
  ne.compute (*cloud_normals);

  pcl::PrincipalCurvaturesEstimation <Point, PointNormal, PrincipalCurvatures> pce;
  pce.setInputCloud (cloud);
  pce.setInputNormals (cloud_normals);

  pce.setSearchMethod (kdtree);
  pce.setKSearch (K);

  CloudPCPtr cloud_pc (new CloudPC);
  pce.compute (*cloud_pc);

  for (int i = 0; i < cloud->points.size (); i++) {
    float pcs[3];
    pcs[0] = cloud_pc->points[i].principal_curvature[0];
    pcs[1] = cloud_pc->points[i].principal_curvature[1];
    pcs[2] = cloud_pc->points[i].principal_curvature[2];

    sort (pcs, pcs+3, less <float> ());

    if ( (pcs[0] < 0.01 * pcs[1]) and 
         (pcs[1] > 0.9 * pcs[2]) 
       ) {
      keypoints->push_back (cloud->points[i]);
    }
  }
  keypoints->width = 1;
  keypoints->height = keypoints->points.size ();
}

void KeyPointsInterface::computeCurvatureLocalMaxima ()
{
  CurvatureLocalMaximaParams& par = *( (CurvatureLocalMaximaParams*) keypoint_par.get () );
  int K = par.k_search;

  cout << "K parameter = " << K << endl;

  pcl::NormalEstimation<Point, PointNormal> ne;
  CloudNormalPtr cloud_normals (new CloudNormal);
  KdTreePtr kdtree (new KdTree);

  ne.setInputCloud (cloud);
  ne.setSearchMethod (kdtree);
  ne.setKSearch (K);
  ne.compute (*cloud_normals);

  cout << "Normals computed ..\n";

  vector <bool> flags (cloud_normals->points.size ());
  fill_n (flags.begin (), cloud_normals->points.size (), false);

  vector <int> ids;
  vector <float> dists;

  for (int i = 0; i < cloud->points.size (); i++) {
    
    if (flags[i]) {
      continue;
    }

    Point& pt = cloud->points[i];
    ids.clear ();
    dists.clear ();

    if (kdtree->nearestKSearch (pt, K*5, ids, dists) > 0) {

      bool status = true;
      foreach (int& k, ids) {
        if (cloud_normals->points[i].curvature > 
            cloud_normals->points[k].curvature ) {
          status = false;
          break;
        }
      }

      if (status) {
        keypoints->push_back (cloud->points[i]);
        flags[i] = true;
        foreach (int& k, ids) {
          flags[k] = true;
        }
      }

    }
  }

  keypoints->width= 1;
  keypoints->height = keypoints->points.size ();
}

void KeyPointsInterface::computeBoundaryPoints ()
{
  BoundaryPointsParams& par = *( (BoundaryPointsParams*) keypoint_par.get () );
  int K = par.k_search;
  float border_radius = par.border_radius;
  const int min_neighbours = 5;
  const float angle_threshold = M_PI / 2.0;

  pcl::NormalEstimation<Point, PointNormal> ne;
  pcl::PointCloud<PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  KdTreePtr kdtree (new KdTree);

  ne.setInputCloud (cloud);
  ne.setSearchMethod (kdtree);
  ne.setKSearch (K);
  ne.compute (*cloud_normals);


  pcl::BoundaryEstimation<Point, PointNormal, pcl::Boundary> be;
  be.setInputCloud (cloud);

  vector <int> ids;
  vector <float> dists;

  for (int i = 0; i < cloud->points.size (); i++) {
    Point& pt = cloud->points[i];
    ids.clear ();
    dists.clear ();
    if (kdtree->radiusSearch (pt, border_radius, ids, dists) > 0) {
      if (ids.size () < min_neighbours) {
        continue;
      }

      Eigen::Vector4f u = Eigen::Vector4f::Zero ();
      Eigen::Vector4f v = Eigen::Vector4f::Zero ();

      be.getCoordinateSystemOnPlane (cloud_normals->points[i], u, v);
      if (be.isBoundaryPoint (*cloud, i, ids, u, v, angle_threshold) ) {
        keypoints->points.push_back (cloud->points[i]);
      }
    }
  }
  keypoints->width = 1;
  keypoints->height = keypoints->points.size ();
}

void KeyPointsInterface::computeISS ()
{ 
  cout << "Computing ISS points ...\n";
  KdTreePtr tree (new KdTree);

  pcl::ISSKeypoint3D<Point, Point> iss_detector;

  ISSKeypointParams& par = *( (ISSKeypointParams*) keypoint_par.get () );

  iss_detector.setSearchMethod (tree);
  //iss_detector.setNonMaximaSupression (false);
  iss_detector.setSalientRadius (6 * par.model_resolution);
  iss_detector.setNonMaxRadius (4 * par.model_resolution);
  iss_detector.setThreshold21 (par.ratio_21);
  iss_detector.setThreshold32 (par.ratio_32);
  iss_detector.setMinNeighbors (par.min_neighbours);
  iss_detector.setBorderRadius (3 * par.model_resolution);
  iss_detector.setNormalRadius (3 * par.model_resolution);
  iss_detector.setNumberOfThreads (4);
  iss_detector.setInputCloud (cloud);
  iss_detector.compute (*keypoints);
}


void KeyPointsInterface::computeSIFT ()
{
  SIFTKeypointParams& par = *( (SIFTKeypointParams*) keypoint_par.get () );
  CloudNormalPtr normals (new CloudNormal);
  copyPointCloud (*cloud, *normals);

  pcl::NormalEstimation<Point, PointNormal> ne;
  KdTreePtr kdtree (new KdTree);

  ne.setInputCloud (cloud);
  ne.setSearchMethod (kdtree);
  ne.setKSearch (par.k_search);
  ne.compute (*normals);


  //pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
  //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZ>());

  //ne.setInputCloud (cloud);
  //ne.setSearchMethod (tree_n);
  //ne.setRadiusSearch (radius_search);
  //ne.compute (*normals);
  //using namespace pcl;

  pcl::SIFTKeypoint <PointNormal, pcl::PointWithScale> sift;
  pcl::PointCloud <pcl::PointWithScale> result;
  KdTreeNormalPtr tree (new KdTreeNormal);
  sift.setSearchMethod (tree);
  sift.setScales (par.min_scale, par.n_octaves, par.n_scales_per_octave);
  sift.setMinimumContrast (par.min_contrast);
  sift.setInputCloud (normals);
  sift.compute (result);

  copyPointCloud (result, *keypoints);

}

}
