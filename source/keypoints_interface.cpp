#include "keypoints_interface.h"

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <stdexcept>
using namespace std;

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
}

void KeyPointsInterface::computeCurvatureKeypoint ()
{
  CurvatureKeypointParams& par = *( (CurvatureKeypointParams*) keypoint_par.get () );

  CloudNormalPtr normals (new CloudNormal);
  copyPointCloud (*cloud, *normals);

  pcl::NormalEstimation<Point, PointNormal> ne;
  pcl::PointCloud<PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
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

void KeyPointsInterface::computeISS ()
{ 
  cout << "Computing ISS points ...\n";
  KdTreePtr tree (new KdTree);

  pcl::ISSKeypoint3D<Point, Point> iss_detector;

  ISSKeypointParams& par = *( (ISSKeypointParams*) keypoint_par.get () );

  iss_detector.setSearchMethod (tree);
  //iss_detector.setNonMaximaSupression (false);
  iss_detector.setSalientRadius (6 * par.model_resolution);
  //iss_detector.setNonMaxRadius (4 * par.model_resolution);
  iss_detector.setNonMaxRadius (0.1);
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
  pcl::PointCloud<PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
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
