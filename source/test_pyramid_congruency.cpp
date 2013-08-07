#include <iostream>
#include <fstream>
using namespace std;

#include <boost/shared_ptr.hpp>

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

#include "typedefs.h"
using namespace E4PCS;

#include <Geometry/AngleAxis.h>

struct Pyramid
{
  Point a;
  Point b;
  Point c;
  Point d;
  Point e;
};

float sphere_radius = 0.2;


void findPlaneEquation (Point& pa, Point& pb, Point& pc, 
                        Eigen::Vector4f& eq)
{
  eq.setZero ();
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

  //cout << "Eigen values = " << eigenvalues[0] << " " << eigenvalues[1] << " " << eigenvalues[2] << endl;

  float A = eigenvector3 (0);
  float B = eigenvector3 (1);
  float C = eigenvector3 (2);
  float D = - (A*centroid (0) + B*centroid (1) + C*centroid (2));

  eq.block <3,1> (0, 0) = eigenvector3;
  eq (3) = -(centroid.dot (eigenvector3));
}

void getEdgeLengths (Pyramid& pyramid, vector <float>& edge_lengths)
{
  Eigen::Vector3f a = pyramid.a.getVector3fMap ();
  Eigen::Vector3f b = pyramid.b.getVector3fMap ();
  Eigen::Vector3f c = pyramid.c.getVector3fMap ();
  Eigen::Vector3f d = pyramid.d.getVector3fMap ();
  Eigen::Vector3f e = pyramid.e.getVector3fMap ();

  edge_lengths[0] = (a-e).norm ();
  edge_lengths[1] = (b-e).norm ();
  edge_lengths[2] = (c-e).norm ();
  edge_lengths[3] = (d-e).norm ();
}

void pyramidApexBaseDistanceCheck (Pyramid& pyramid, 
                                   Pyramid& mpyramid)
{
  Eigen::Vector4f plane_eq1;
  plane_eq1.setZero ();
  findPlaneEquation (pyramid.a, pyramid.b, pyramid.c, plane_eq1);

  Eigen::Vector4f plane_eq2;
  plane_eq2.setZero ();
  findPlaneEquation (mpyramid.a, mpyramid.b, mpyramid.c, plane_eq2);

  cout << "\n\nApex base distance = " << plane_eq1.dot (pyramid.e.getVector4fMap ())
       << " " << plane_eq2.dot (mpyramid.e.getVector4fMap ()) << endl;
}

void pyramidEdgeLengthCheck (Pyramid& pyramid, 
                             Pyramid& mpyramid)
{
  vector <float> edge_lengths1;
  vector <float> edge_lengths2;

  edge_lengths1.resize (4);
  edge_lengths2.resize (4);

  getEdgeLengths (pyramid, edge_lengths1);
  getEdgeLengths (mpyramid, edge_lengths2);

  cout << "Pyramid Edge Length Check ::  ";

  for (int i = 0; i < 4; i++) {
    cout << fabs (edge_lengths1[i] - edge_lengths2[i]) << "\t";
  }
  cout << endl;
}




void drawPyramid (Pyramid& pyramid, 
                  boost::shared_ptr<PCLVisualizer> viz,
                  int var = 0)
{
  ostringstream ostr;

  static int lineId = 39745;
  static int sphereId = 12445;

  double red = 0, green = 1, blue = 0;

  CloudPtr source (new Cloud);
  source->points.push_back (pyramid.a);
  source->points.push_back (pyramid.b);
  source->points.push_back (pyramid.c);
  source->points.push_back (pyramid.d);
  source->points.push_back (pyramid.e);

  source->width = 1;
  source->height = source->points.size ();

  for (int k = 0; k <4; k++) {
    ostr.str ("");
    ostr << "sphere" << sphereId++;
    viz->addSphere (source->points[k], sphere_radius, 
                    red, green, blue, ostr.str ().c_str ());
  }
  ostr.str ("");
  ostr << "sphere" << sphereId++;
  if (var) {
    viz->addSphere (pyramid.e, sphere_radius, 
        1, 0, 0, ostr.str ().c_str ());
  }
  else {
    viz->addSphere (pyramid.e, sphere_radius, 
        1, 0, 1, ostr.str ().c_str ());
  }

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[0], source->points[1], 
                1, 1, 0, ostr.str ().c_str ());

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[0], source->points[2], 
                 0, 1, 1, ostr.str ().c_str ());

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[0], source->points[3], 
                 0, 1, 1, ostr.str ().c_str ());

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[1], source->points[2], 
                 0, 1, 1, ostr.str ().c_str ());

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[1], source->points[3], 
                 0, 1, 1, ostr.str ().c_str ());

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[2], source->points[3], 
                 1, 1, 0, ostr.str ().c_str ());

  for (int i = 0; i < 4; i++) {
    ostr.str ("");
    ostr << "line" << lineId++;
    viz->addLine (source->points[i], source->points[4], 
        0, 1, 1, ostr.str ().c_str ());
  }
}


void find3PlaneIntersection (Eigen::Vector4f& plane_eq1,
                             Eigen::Vector4f& plane_eq2,
                             Eigen::Vector4f& plane_eq3,
                             Eigen::Vector3f& e)
{
  // Refer to Eq 8 in http://mathworld.wolfram.com/Plane-PlaneIntersection.html
  Eigen::Vector3f n1 = plane_eq1.block <3, 1> (0, 0);
  Eigen::Vector3f n2 = plane_eq2.block <3, 1> (0, 0);
  Eigen::Vector3f n3 = plane_eq3.block <3, 1> (0, 0);

  float x1 = -plane_eq1 (3);
  float x2 = -plane_eq2 (3);
  float x3 = -plane_eq3 (3);

  cout << "\n";
  cout << "Normal 1 = " << n1.transpose () << endl;
  cout << "Normal 2 = " << n2.transpose () << endl;
  cout << "Normal 3 = " << n3.transpose () << endl;

  Eigen::Matrix3f mat;
  mat.row (0) = n1;
  mat.row (1) = n2;
  mat.row (2) = n3;

  float K = mat.determinant ();
  
  cout << "\nDeterminant = " << K << endl;

  e.setZero ();
  e = (1./K) * (x1 * (n2.cross (n3)) +
                x2 * (n3.cross (n1)) +
                x3 * (n1.cross (n2)));
  cout << "\nIntersection is " << e.transpose () << endl;

}

void demeanPointCloud (CloudPtr &in, Eigen::Vector4f& centroid,
                             Eigen::MatrixXf &out)
{
  size_t npts = in->points.size ();

  out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

  for (size_t i = 0; i < npts; ++i) {
    out.block<4, 1> (0, i) = in->points[i].getVector4fMap () - centroid;
  }

  out.block (3, 0, 1, npts).setZero ();
}

void getRotationFromCorrelation (Eigen::MatrixXf &cloud_src_demean,
                                       Eigen::Vector4f &centroid_src,
                                       Eigen::MatrixXf &cloud_tgt_demean,
                                       Eigen::Vector4f &centroid_tgt,
                                       Eigen::Matrix3f &R)
{
  R.setIdentity ();

  Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU ();
  Eigen::Matrix3f v = svd.matrixV ();

  if (u.determinant () * v.determinant () < 0) {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  R = v * u.transpose ();
}

void estimateRotation (CloudPtr src, CloudPtr tgt,
                                        Eigen::Matrix3f& R)
{

  Eigen::Vector4f centroid_src, centroid_tgt;
  compute3DCentroid (*src, centroid_src);
  compute3DCentroid (*tgt, centroid_tgt);

  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (src, centroid_src, cloud_src_demean);

  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (tgt, centroid_tgt, cloud_tgt_demean);

  getRotationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, R);
}

void adjustPyramid2PlaneEqn (Eigen::Matrix3f& R, 
                             Eigen::Vector4f& plane_eq, 
                             Point& pt)
{
  plane_eq.block <3, 1> (0, 0) = R * plane_eq.block <3, 1> (0, 0);
  plane_eq (3) = -pt.getVector3fMap ().dot (plane_eq.block <3, 1> (0, 0));
}

void findCongruentPyramid (Pyramid& pyramid1, Pyramid& pyramid2)
{
  Eigen::Vector4f  plane_eq1;
  Eigen::Vector4f  plane_eq2;
  Eigen::Vector4f  plane_eq3;

  findPlaneEquation (pyramid1.a, pyramid1.b, pyramid1.e, plane_eq1);
  findPlaneEquation (pyramid1.b, pyramid1.c, pyramid1.e, plane_eq2);
  findPlaneEquation (pyramid1.a, pyramid1.d, pyramid1.e, plane_eq3);

  cout << plane_eq1.transpose () << endl;
  cout << plane_eq2.transpose () << endl;
  cout << plane_eq3.transpose () << endl;

  CloudPtr src (new Cloud);
  CloudPtr tgt (new Cloud);

  src->points.push_back (pyramid1.a);
  src->points.push_back (pyramid1.b);
  src->points.push_back (pyramid1.c);
  src->points.push_back (pyramid1.d);
  src->width = 1;
  src->height = src->points.size ();

  tgt->points.push_back (pyramid2.a);
  tgt->points.push_back (pyramid2.b);
  tgt->points.push_back (pyramid2.c);
  tgt->points.push_back (pyramid2.d);
  tgt->width = 1;
  tgt->height = tgt->points.size ();

  Eigen::Matrix3f R;
  estimateRotation (src, tgt, R);

  adjustPyramid2PlaneEqn (R, plane_eq1, pyramid2.a);
  adjustPyramid2PlaneEqn (R, plane_eq2, pyramid2.b);
  adjustPyramid2PlaneEqn (R, plane_eq3, pyramid2.a);

  Eigen::Vector3f e;
  find3PlaneIntersection (plane_eq1, plane_eq2, plane_eq3, e);

  pyramid2.e.x = e (0);
  pyramid2.e.y = e (1);
  pyramid2.e.z = e (2);
}

int main (int argc, char *argv[])
{
  if (argc < 3) {
    cout << "Enter all arguments ..\n";
    return -1;
  }

  Pyramid pyramid1;
  Pyramid pyramid2;
  ifstream ifile (argv[1]);

  if (!ifile) {
    cout << "Couldn't open file " << argv[1] << endl;
    return -1;
  }

  int n = atoi (argv[2]);

  for (int i = 0; i < n; i++) {
    ifile >> pyramid1.a.x, ifile >> pyramid1.a.y, ifile >> pyramid1.a.z;
    ifile >> pyramid1.b.x, ifile >> pyramid1.b.y, ifile >> pyramid1.b.z;
    ifile >> pyramid1.c.x, ifile >> pyramid1.c.y, ifile >> pyramid1.c.z;
    ifile >> pyramid1.d.x, ifile >> pyramid1.d.y, ifile >> pyramid1.d.z;
    ifile >> pyramid1.e.x, ifile >> pyramid1.e.y, ifile >> pyramid1.e.z;


    ifile >> pyramid2.a.x, ifile >> pyramid2.a.y, ifile >> pyramid2.a.z;
    ifile >> pyramid2.b.x, ifile >> pyramid2.b.y, ifile >> pyramid2.b.z;
    ifile >> pyramid2.c.x, ifile >> pyramid2.c.y, ifile >> pyramid2.c.z;
    ifile >> pyramid2.d.x, ifile >> pyramid2.d.y, ifile >> pyramid2.d.z;
    ifile >> pyramid2.e.x, ifile >> pyramid2.e.y, ifile >> pyramid2.e.z;
  }

  Pyramid pyramid3 = pyramid2;
  findCongruentPyramid (pyramid1, pyramid3);

  boost::shared_ptr <PCLVisualizer> p (new PCLVisualizer (argc, argv, "Final output"));
  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);

  drawPyramid (pyramid1, p);
  drawPyramid (pyramid2, p);
  drawPyramid (pyramid3, p, 1);

  cout << "Apex difference = " << (pyramid2.e.getVector3fMap () - 
                                   pyramid3.e.getVector3fMap ()).norm () << endl;

  pyramidEdgeLengthCheck (pyramid1, pyramid2);
  pyramidApexBaseDistanceCheck (pyramid1, pyramid2);

  p->spin ();
  return 0;
}

#if 0

int main (int argc, char *argv[])
{
  Pyramid pyramid1;
  pyramid1.a.x = 0, pyramid1.a.y = 0, pyramid1.a.z = 0;
  pyramid1.b.x = 5, pyramid1.b.y = 0, pyramid1.b.z = 0;
  pyramid1.c.x = 0, pyramid1.c.y = 5, pyramid1.c.z = 0;
  pyramid1.d.x = 5, pyramid1.d.y = 5, pyramid1.d.z = 0;
  pyramid1.e.x = 2.5, pyramid1.e.y = 2.5, pyramid1.e.z = 5;

  boost::shared_ptr <PCLVisualizer> p ( new PCLVisualizer (argc, argv, "Final output"));
  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);

  drawPyramid (pyramid1, p);

  Pyramid pyramid2;

  float roll = 10., pitch = 60., yaw = 90.;

  Eigen::Matrix3f R;
  R = Eigen::AngleAxisf (roll * M_PI / 180, Eigen::Vector3f::UnitX ())
    * Eigen::AngleAxisf (pitch * M_PI / 180, Eigen::Vector3f::UnitY ())
    * Eigen::AngleAxisf (yaw * M_PI / 180, Eigen::Vector3f::UnitZ ());

  Eigen::Vector3f T;
  T (0) = 10., T (1) = 0., T (2) = 0.;

  Eigen::Vector3f t;
  t.setZero ();
  t = R * pyramid1.a.getVector3fMap () + T;
  pyramid2.a.getVector3fMap () = t;

  t.setZero ();
  t = R * pyramid1.b.getVector3fMap () + T;
  pyramid2.b.getVector3fMap () = t;

  t.setZero ();
  t = R * pyramid1.c.getVector3fMap () + T;
  pyramid2.c.getVector3fMap () = t;

  t.setZero ();
  t = R * pyramid1.d.getVector3fMap () + T;
  pyramid2.d.getVector3fMap () = t;

  t.setZero ();
  t = R * pyramid1.e.getVector3fMap () + T;
  //pyramid2.e.getVector3fMap () = t;

  //pyramid2.a.x = 10, pyramid2.a.y = 0, pyramid2.a.z = 0;
  //pyramid2.b.x = 15, pyramid2.b.y = 0, pyramid2.b.z = 0;
  //pyramid2.c.x = 10, pyramid2.c.y = 5, pyramid2.c.z = 0;
  //pyramid2.d.x = 15, pyramid2.d.y = 5, pyramid2.d.z = 0;

  findCongruentPyramid (pyramid1, pyramid2);
  cout << "Difference in estimates = " << 
      (pyramid2.e.getVector3fMap () - t).norm () << endl;
  //pyramid2.e.x = 12.5, pyramid2.e.y = 2.5, pyramid2.e.z = 5;

  drawPyramid (pyramid2, p);

  p->spin ();
  return 0;
}

#endif
