#include "affine_invariant_representation.h"

#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH 

namespace E4PCS {

void AffInvRep::compute ()
{
  centroid1.setZero ();

  foreach (Point& pt, cloud1->points) {
    centroid1 += pt.getVector3fMap ();
  }

  centroid1 /= cloud1->points.size ();

  foreach (Point& pt, cloud1->points) {
    Point npt;
    Eigen::Vector3f t = (pt.getVector3fMap () - centroid1);
    npt.x = t (0), npt.y = t (1), npt.z = t (2);
    aircloud1->points.push_back (npt);
  }
  aircloud1->width = 1;
  aircloud1->height = aircloud1->points.size ();

  findPrincipalComponents ();

  for (int i = 0; i < aircloud1->points.size (); i++) {
    Point& pt = aircloud1->points[i];
    Eigen::Vector3f c = pt.getVector3fMap ();
    float x = c.dot (eigenvector1);
    float y = c.dot (eigenvector2);
    float z = c.dot (eigenvector3);

    pt = Point (x, y, z);
  }

  for (int i = 0; i < aircloud2->points.size (); i++) {
    Point& pt = aircloud2->points[i];
    Eigen::Vector3f c = pt.getVector3fMap ();
    float x = c.dot (eigenvector1);
    float y = c.dot (eigenvector2);
    float z = c.dot (eigenvector3);

    pt = Point (x, y, z);
  }

}

void AffInvRep::findPrincipalComponents ()
{
  centroid2.setZero ();

  foreach (Point& pt, cloud2->points) {
    centroid2 += pt.getVector3fMap ();
  }

  centroid2 /= cloud2->points.size ();

  Eigen::Matrix3f covariance;
  covariance.setZero ();

  foreach (Point& pt, cloud2->points) {
    Point npt;
    Eigen::Vector3f V = (pt.getVector3fMap () - centroid2);
    npt.x = V (0), npt.y = V (1), npt.z = V (2);
    aircloud2->points.push_back (npt);
    covariance += (V * V.transpose ());
  }

  aircloud2->width = 1;
  aircloud2->height = aircloud2->points.size ();

  Eigen::EigenSolver <Eigen::Matrix3f> es (covariance);

  Eigen::Vector3d eigenvalues;
  eigenvalues.setZero ();
  pcl::eigen33 (covariance, eigenvalues);

  eigenvector1.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [2], eigenvector1);

  eigenvector2.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [1], eigenvector2);
}


}
