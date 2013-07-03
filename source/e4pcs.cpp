#include "e4pcs.h"
#include <utils/timer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <boost/foreach.hpp>

#include <iostream>
#include <functional>
#include <algorithm>
#include <sstream>
#include <limits>

using namespace std;

#define foreach BOOST_FOREACH


extern float sphere_radius;

namespace E4PCS {


void Extended4PCS::align ()
{
  selectMaxPlane ();

  int N = num_quads;
  selectQuads (plane_pts, N);

  //for (int i = 0; i < quads.size (); i++) {
  //  cout << quads[i].q[0] << " " << quads[i].q[1] << " "
  //       << quads[i].q[2] << " " << quads[i].q[3] << endl;
  //}


  cout << "\n\nSelected " << quads.size () << " quads in source cloud ..\n";

  initializeQuadMatchTable ();

  cout << "\n\nFinding matching point pairs ..\n";
  utils::Timer timer ("seconds");
  timer.tic ();

  for (int i = 0; i < target->points.size (); i++) {
    for (int j = i+1; j < target->points.size (); j++) {

      float length = (target->points[i].getVector3fMap () - 
                      target->points[j].getVector3fMap ()).norm ();

      insertToQuadMatchTable (length, i, j);
    }
  }

  cout << "Time taken = " << timer.toc () << " seconds\n\n\n";

  cout << "# entries in Point List table = " << pointListTable.size () << "\n\n";

  cout << "\n\n";


  if (quadMatchTable.size () == 0) {
    cout << "No quads found with the given parameters .. Stopping ..\n";
    return;
  }
    
  cout << "<< QUAD TABLE >> \n-------------------------------------------------------\n";
  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];
    cout << "Quad " << i+1 << " AB length = " << qmatch.ab_len << ",\t# point pairs = " << (*(qmatch.r1_pts)).size () / 2 << endl;
    cout << "Quad " << i+1 << " CD length = " << qmatch.cd_len << ",\t# point pairs = " << (*(qmatch.r2_pts)).size () / 2 << endl;
    cout << "-------------------------------------------------------\n";
  }

  findSimilarQuads ();

  bool emptyFlag = true;
  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];
    cout << "Quad " << i+1 << " # of matching quads = " << qmatch.matches.size () << endl;
    if (qmatch.matches.size ()) {
      emptyFlag = false;
    }
  }

  if (emptyFlag) {
    cout << "\n\n <<< NO MATCHING QUADS FOUND. RELAX SOME PARAMETERS AND TRY AGAIN >>> ..\n";
    return;
  }

  cout << "\n";

  // FIDNING THE BEST MATCHING QUAD
  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];
    findBestQuadMatch (qmatch);
    cout << "Found best match for Quad " << i+1 << "\t";
    cout << qmatch.best_match << "\tRMS error = " << qmatch.least_error << "\n";
  }

  for (int i = 0; i < quadMatchTable.size (); i++) {
    if (quadMatchTable[i].best_match != -1) {
      cout << "Best match for quad " << i+1 << " is " 
        << quadMatchTable[i].best_match + 1 << " of "
        << quadMatchTable[i].matches.size () << "\t"
        << "Error is " << quadMatchTable[i].least_error << endl;
      continue;
    }

    quadMatchTable[i].ignoreMatch = true;
    cout << "Match for quad " << i+1 << " NOT FOUND\n";
  }

  cout << endl;

  // among matching quads, select the ones on a plane
  // All quads in the source cloud are on a plane, so 
  // enforcing the same constraint
  if ( !filterMatchingQuads () ) {
    cout << "\n\n\n";
    cout << "**************** Selecting a NEW PLANE *****************\n\n";
    cleanup ();
    align ();
    return;
  }

  findTransformationParameters ();

  //return true;

  //int color[3] = {255, 255, 255};
  //displayPointCloud (pviz, source, color, (char *) "cloud1", vp1);

  ////color[0] = 0, color[1] = 0, color[2] = 0;
  //displayPointCloud (pviz, target, color, (char *) "cloud2", vp2);


  debugQuadMatch ();

  //for (int i = 0; i < quadMatchTable.size (); i++) {
  //  double r = 0., g = 0., b = 0.;
  //  switch (i%3) {
  //    case 0: r = 1; break;
  //    case 1: g = 1; break;
  //    case 2: b = 1; break;
  //  }

  //  // only plot quads on the dominant plane
  //  if (quadMatchTable[i].ignoreMatch) {
  //    continue;
  //  }

  //  if (i == best_quad) {
  //    plotMatchingQuads (pviz, quadMatchTable[i], r, g, b, vp1, vp2);
  //  }
  //}

 // for (int i = 0; i < quadMatchTable.size (); i++) {
 //   Quad& quad = *(quadMatchTable[i].quad);
 //   if (quadMatchTable[i].ignoreMatch) {
 //     continue;
 //   }

 //   Quad& mquad = quadMatchTable[i].matches[quadMatchTable[i].best_match];

 //   //cout << "Angle between (" << source->points[quad.q[0]] << "," << quad.intersection.transpose ()
 //   //  << ") and (" << quad.intersection.transpose () << "," << source->points[quad.q[3]] << ") is "
 //   //  << quad.intersect_angle << endl;


 //   //cout << "Angle between (" << target->points[mquad.q[0]] << "," << mquad.intersection.transpose ()
 //   //  << ") and (" << mquad.intersection.transpose () << "," << target->points[mquad.q[3]] << ") is "
 //   //  << mquad.intersect_angle << endl;
 // }

}

//void displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
//                        char* name, int& viewport)
//{
//  PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
//  viz->addPointCloud (cloud, tgt_h, name, viewport);
//}

void Extended4PCS::debugQuadMatch ()
{

  int validMatches = 0;

  for (int i = 0; i < quadMatchTable.size (); i++) {
    if (!quadMatchTable[i].ignoreMatch) {
      validMatches++;
    }
  }

  cout << "DEBUG :: # of valid matches = " << validMatches << endl;

  int quadsPerVis = 1;
  int n_vis = floor (validMatches / quadsPerVis);
  if (validMatches % quadsPerVis) {
    n_vis++;
  }

  cout << "DEBUG :: # of visualizers = " << n_vis << endl;

  vector <int> vp (quadsPerVis * 2);
  for (int i = 1; i <= quadsPerVis * 2; i++) {
    vp.push_back (i);
  }

  int color[3] = {255, 255, 255};

  static int cloud_id = 36488;
  for (int i = 0; i < n_vis; i++) {
    ostringstream ostr;
    ostr << "Matching quads set " << i+1;
    PCLVisualizer *v = new PCLVisualizer (argc_, argv_ , ostr.str ().c_str ());
    char *name = NULL;

    float steplen = 1. / quadsPerVis;
    for (int k = 0; k < quadsPerVis; k++) {
      v->createViewPort (0.0, k*steplen, 1./2, (k+1)*steplen, vp[2*k]);
      ostr.str ("");
      ostr << "cloud" << cloud_id++;
      name = (char *)ostr.str ().c_str ();
      displayPointCloud (v, source, color, name, vp[2*k]);

      v->createViewPort (1./2, k*steplen, 1.0, (k+1)*steplen, vp[2*k+1]);
      ostr.str ("");
      ostr << "cloud" << cloud_id++;
      name = (char *)ostr.str ().c_str ();
      displayPointCloud (v, target, color, name, vp[2*k+1]);

      //v->createViewPort (0.0, 1./3, 1./2, 2./3, vp[2]);
      //ostr.str ("");
      //ostr << "cloud" << cloud_id++;
      //name = (char *)ostr.str ().c_str ();
      //displayPointCloud (v, source, color, name, vp[2]);

      //v->createViewPort (1./2, 1./3, 1., 2./3, vp[3]);
      //ostr.str ("");
      //ostr << "cloud" << cloud_id++;
      //name = (char *)ostr.str ().c_str ();
      //displayPointCloud (v, target, color, name, vp[3]);

      //v->createViewPort (0.0, 2./3, 1./2, 1.0, vp[4]);
      //ostr.str ("");
      //ostr << "cloud" << cloud_id++;
      //name = (char *)ostr.str ().c_str ();
      //displayPointCloud (v, source, color, name, vp[4]);

      //v->createViewPort (1./2, 2./3, 1.0, 1.0, vp[5]);
      //ostr.str ("");
      //ostr << "cloud" << cloud_id++;
      //name = (char *)ostr.str ().c_str ();
      //displayPointCloud (v, target, color, name, vp[5]);

    }

    v->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);
    V.push_back (v);
  }

  int z = 0;
  for (int i = 0; i < quadMatchTable.size (); i++) {
    double r = 0., g = 0., b = 0.;
    switch (i%3) {
      case 0: r = 1; break;
      case 1: g = 1; break;
      case 2: b = 1; break;
    }

    if (quadMatchTable[i].ignoreMatch) {
      continue;
    }

    int k = (int) (z / quadsPerVis);

    cout << "z = " << z << " :: Choosing visualizer " << k << endl;

    int q = z % quadsPerVis;
    plotMatchingQuads (V[k], quadMatchTable[i], r, g, b, vp[2*q], vp[2*q+1]);
    z++;
  }

  //foreach (PCLVisualizer* v, V) {
  //  v->spin ();
  //}
}

void Extended4PCS::cleanup ()
{
  plane_pts.clear ();
  quads.clear ();
  pointListTable.clear ();

  foreach (QuadMatch& qm, quadMatchTable) {
    qm.quad = NULL;
  }

  quadMatchTable.clear ();
}

void Extended4PCS::findTransformationParameters ()
{
  double best_error = numeric_limits <double>::max ();
  int index = -1;

  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];

    if (qmatch.ignoreMatch or qmatch.matches.size () == 0) {
      continue;
    }

    if (qmatch.least_error < best_error) {
      index = i;
      best_error = qmatch.least_error;
    }
  }
  
  if (index == -1) {
    cout << "Couln't find matching quads .. Stopping ..\n";
    transform.setZero ();
    transform.block <3, 3> (0, 0) = Eigen::Matrix3f::Identity ();
    return;
  }

  best_quad = index;

  cout << "\nChoosing quad " << index+1 << " for finding the "
    "final transformation\n\n";

  QuadMatch& qmatch = quadMatchTable[best_quad];
  CloudPtr cloud1 (new Cloud);
  CloudPtr cloud2 (new Cloud);

  Quad& q1 = *(qmatch.quad);
  Quad& q2 = qmatch.matches[qmatch.best_match];

  foreach (int& index, q1.q) {
    cloud1->points.push_back (source->points[index]);
  }
  cloud1->width = 1;
  cloud1->height = cloud1->points.size ();

  foreach (int& index, q2.q) {
    cloud2->points.push_back (target->points[index]);
  }
  cloud2->width = 1;
  cloud2->height = cloud2->points.size ();

  Eigen::Matrix3f R;
  Eigen::Vector3f T;

  estimateRigidBodyTransformation (cloud1, cloud2, R, T);

  transform.block <3, 3> (0, 0) = R;
  transform.block <3, 1> (0, 3) = T;

  transform (3, 3) = 1.0;

  cout << "\n---------- TRANSFORMATION ---------------\n"
    << transform << "\n------------------------------------------\n\n";

}

bool Extended4PCS::filterMatchingQuads ()
{
  vector <bool> flags (quadMatchTable.size ());
  vector <int> plane_ids (quadMatchTable.size ());

  int plane_id = 1;

  for (int i = 0; i < quadMatchTable.size (); i++) {

    bool plane_assigned = false;

    if (flags[i]) { 
      continue;
    }

    if (quadMatchTable[i].ignoreMatch) {
      //cout << "\n\nSame plane check : Quad " << i+1 << " and Quad " << j+1 << " ";
      //cout << "\n\nSkipping Quad " << i+1 << " due to ignoreMatch\n";
      continue;
    }

    for (int j = i; j < quadMatchTable.size (); j++) {

      if (flags[j]) {
        continue;
      }


    if (quadMatchTable[j].ignoreMatch) {
      //cout << "\n\nSame plane check : Quad " << i+1 << " and Quad " << j+1 << " ";
      //cout << "Skipping Quad " << j+1 << " due to ignoreMatch\n";
      continue;
    }


      //cout << "\n\nSame plane check : Quad " << i+1 << " and Quad " << j+1 << endl;
        
      if ( samePlaneCheck (quadMatchTable[i].matches[quadMatchTable[i].best_match],
                          quadMatchTable[j].matches[quadMatchTable[j].best_match]) )
      {
        flags[i] = true;
        flags[j] = true;
        plane_ids[i] = plane_id;
        plane_ids[j] = plane_id;
        plane_assigned = true;
      }
    }

    plane_id++;

    if (!plane_assigned) {
      plane_ids[i] = plane_id;
    }
  }

  // contains count of quads on different planes
  vector <int> plane_cnt;

  cout << endl;

  for (int i = 1; i < plane_id; i++) {
    int c = count (plane_ids.begin (), plane_ids.end (), i);
    cout << "# of points on plane " << i << " = " << c << endl;
    plane_cnt.push_back (c);
  }

  // count of quads on the dominant plane (plane containing max quads)
  int max_cnt = *(max_element (plane_cnt.begin (), plane_cnt.end ()));


  // in case there are more planes that have same max_count,
  // we maintain a list
  vector <int> dominant_planes;
  for (int i = 0; i < plane_cnt.size (); i++) {
    if (plane_cnt[i] == max_cnt) {
      dominant_planes.push_back (i+1);
    }
  }

  cout << "\n# of dominant planes = " << dominant_planes.size () << "\n\n";

  for (int i = 0; i < dominant_planes.size (); i++) {
    cout << "# of quads in dominant plane " << i+1 << " is " 
      << plane_cnt[dominant_planes[i]-1] << endl;
  }

  int dom_plane_id = dominant_planes[0];


  for (int i = 0; i < plane_ids.size (); i++) {
    if (plane_ids[i] != dom_plane_id) {
      quadMatchTable[i].ignoreMatch  = true;
    }
  }

  if (plane_cnt.size () == 1) {
    return true;
  }
  cout << "plane_cnt.size () = " << plane_cnt.size () << endl;
  vector <int> tmp;
  foreach (int& i, plane_cnt) {
    tmp.push_back (i);
  }

  sort (tmp.begin (), tmp.end (), greater <int> ());

  int max_cnt1 = tmp[0];
  int max_cnt2 = tmp[1];

  cout << " :: max cnt 1 = " << max_cnt1 << ", max_cnt2 = " << max_cnt2;
  cout << " ::  CONDITION " << (max_cnt1 - max_cnt2)  << " >= " << max_cnt1/2 << endl;

  //if ( (max_cnt1 - max_cnt2) <= 2 ) {
  //  return false;
  //}


  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];
    if (qmatch.matches.size () != 0)
      qmatch.ignoreMatch = false;
    else 
      qmatch.ignoreMatch = true;
  }

  if ( (max_cnt1 - max_cnt2) >= (max_cnt1/2) ) {
    return true;
  }

  return false;
}

bool Extended4PCS::samePlaneCheck (Quad& one, Quad& two)
{
  Eigen::Vector3f centroid;
  centroid.setZero ();

  foreach (int& index, one.q) {
    centroid += target->points[index].getVector3fMap ();
  }

  foreach (int& index, two.q) {
    centroid += target->points[index].getVector3fMap ();
  }

  centroid /= 8.0; // two quads contain 8 points

  Eigen::Matrix3f covariance;
  covariance.setZero ();

  foreach (int& index, one.q) {
    Eigen::Vector3f V;
    V.setZero ();
    V = (target->points[index].getVector3fMap () - centroid) ;
    covariance += (V * V.transpose ());
  }

  foreach (int& index, two.q) {
    Eigen::Vector3f V;
    V.setZero ();
    V = (target->points[index].getVector3fMap () - centroid) ;
    covariance += (V * V.transpose ());
  }

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
  float D = -(A * centroid (0) + B * centroid (1) + C * centroid (2));



  foreach (int& index, one.q) {
    float x = target->points[index].x;
    float y = target->points[index].y;
    float z = target->points[index].z;

    float residue = fabs (A*x + B*y + C*z + D);
    //cout << "Same plane check , residue = " << residue << endl;
    if ( residue > param.plane_fit_threshold ) {
      //cout << "Residue check failed.. returning FALSE..\n";
      return false;
    }
  }

  foreach (int& index, two.q) {
    float x = target->points[index].x;
    float y = target->points[index].y;
    float z = target->points[index].z;

    float residue = fabs (A*x + B*y + C*z + D);
    //cout << "Same plane check , residue = " << residue << endl;
    if ( residue > param.plane_fit_threshold ) {
      //cout << "Residue check failed.. returning FALSE..\n";
      return false;
    }
  }
  
  return true;
}

void Extended4PCS::getRotationFromCorrelation (Eigen::MatrixXf &cloud_src_demean,
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

void Extended4PCS::demeanPointCloud (CloudPtr &in, Eigen::Vector4f& centroid,
                             Eigen::MatrixXf &out)
{
  size_t npts = in->points.size ();

  out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

  for (size_t i = 0; i < npts; ++i) {
    out.block<4, 1> (0, i) = in->points[i].getVector4fMap () - centroid;
  }

  out.block (3, 0, 1, npts).setZero ();
}

void Extended4PCS::estimateTranslation (CloudPtr cloud1, CloudPtr cloud2,
                               Eigen::Matrix3f R, Eigen::Vector3f& T)
{
  Eigen::Vector4f centroid_src, centroid_tgt;
  compute3DCentroid (*cloud1, centroid_src);
  compute3DCentroid (*cloud2, centroid_tgt);

  Eigen::Vector3f tmp1;
  Eigen::Vector3f tmp2;

  tmp1 (0) = centroid_src (0);
  tmp1 (1) = centroid_src (1);
  tmp1 (2) = centroid_src (2);

  tmp2 (0) = centroid_tgt (0);
  tmp2 (1) = centroid_tgt (1);
  tmp2 (2) = centroid_tgt (2);

  T = tmp2 - (R * tmp1);
}

void Extended4PCS::estimateRotation (CloudPtr src, CloudPtr tgt,
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

void Extended4PCS::estimateRigidBodyTransformation (CloudPtr src, CloudPtr tgt,
                                            Eigen::Matrix3f& R,
                                            Eigen::Vector3f& T)
{
  R.setIdentity ();
  T.setZero ();

  if (src->points.size () == 0 &&
      tgt->points.size () == 0) {
    return;
  }

  estimateRotation (src, tgt, R);
  estimateTranslation (src, tgt, R, T);
}

void Extended4PCS::findBestQuadMatch (QuadMatch& qmatch)
{

  if (qmatch.matches.size () == 0) {
    return;
  }
  Quad& quad = *(qmatch.quad);
  CloudPtr cloud1 (new Cloud);
  foreach (int& index, quad.q) {
    cloud1->points.push_back (source->points[index]);
  }
  cloud1->width = 1;
  cloud1->height = cloud1->points.size ();

  double rms_best = numeric_limits <double>::max ();

  //cout << "findBestQuadMatch Error : ";

  for (int i = 0; i < qmatch.matches.size (); i++) {
    Quad& match = qmatch.matches[i];
    CloudPtr cloud2 (new Cloud);
    foreach (int& index, match.q) {
      cloud2->points.push_back (target->points[index]);
    }
    cloud2->width = 1;
    cloud2->height = cloud2->points.size ();

    Eigen::Matrix3f R;
    Eigen::Vector3f T;
    estimateRigidBodyTransformation (cloud1, cloud2, R, T);
    
    double rms_cur = 0;

    CloudPtr sampledsource (new Cloud);
    CloudPtr sampledtarget (new Cloud);

    sampleCloud (source, 300, sampledsource);
    sampleCloud (target, 300, sampledtarget);

    //if ( (rms_cur = estimateError (sampledsource, sampledtarget, R, T) ) < rms_best) {
    if ( (rms_cur = estimateError (source, target, R, T) ) < rms_best) {
      rms_best = rms_cur;
      qmatch.best_match = i;
      qmatch.least_error = rms_best;
    }

    //cout << rms_cur << "\t";

    //int cnt_best  = 0;
    //int cnt_cur = 0;

    //if ( (cnt_cur = estimateError (source, target, R, T) ) > cnt_best ) {
    //  cnt_best = cnt_cur;
    //  qmatch.best_match = i;
    //  qmatch.cnt_best = cnt_best;
    //}

  }
  cout << endl;
}

void Extended4PCS::sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
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

  //pcl::UniformSampling<Point> us;
  //us.setInputCloud (cloud);
  //double radius = 30;
  //us.setRadiusSearch (radius);
  //pcl::PointCloud<int> subsampled_indices;
  //us.compute (subsampled_indices);
  //std::sort (subsampled_indices.points.begin (), subsampled_indices.points.end ());
  //pcl::copyPointCloud (*cloud, subsampled_indices.points, *sampledcloud);

  //cout << "Points = " << cloud->points.size () << " "
  //  << sampledcloud->points.size () << endl;
}

double Extended4PCS::estimateError (CloudPtr src, CloudPtr tgt, 
                            Eigen::Matrix3f& R, Eigen::Vector3f& T)
{
	KdTree kdtree;		
	kdtree.setInputCloud(tgt);
	int K = 1;
	vector<int> ids (K);
	vector<float> dist (K);

	double error = 0.0;
	int count = 0;

	for(int i = 0; i < src->points.size(); i++) {

    Eigen::Vector3f v = src->points[i].getVector3fMap ();
    v = R * v + T;
    Point p1 (v (0), v (1), v (2));

		if(kdtree.nearestKSearch(p1, K, ids, dist) > 0) {
      
			Point p2 = tgt->points[ids[0]];

			double dist = (p1.getVector3fMap () - p2.getVector3fMap ()).norm ();
			if(dist > param.max_range) {
				continue;
			}
			count++;
			error += dist;
		}
	}

  error /= count;

	//cout << "Number of correspondences found = " << count << 
  //  "\tTotal error = " << error << endl;

  return error;
}

void Extended4PCS::plotMatchingQuads (PCLVisualizer* viz, QuadMatch& qmatch, double red,
                              double green, double blue, int vp1, int vp2)
{

  ostringstream ostr;

  static int lineId = 98745;
  static int sphereId = 23445;

  Quad& quad = *(qmatch.quad);
  for (int k = 0; k <4; k++) {
    ostr << "sphere" << sphereId++;
    viz->addSphere (source->points[quad.q[k]], sphere_radius, 
                    red, green, blue, ostr.str ().c_str (), vp1);
  }

  Eigen::Vector3f a = source->points[quad.q[0]].getVector3fMap ();
  Eigen::Vector3f b = source->points[quad.q[1]].getVector3fMap ();

  static int intersectId = 1;
  Eigen::Vector3f e = a + quad.r1 * (b - a);
  Point pt (e (0), e (1), e (2));

  ostr.str ("");
  ostr << "intersect" << intersectId++;
  viz->addSphere (pt, sphere_radius, 1, 1, 0, ostr.str ().c_str (), vp1);

  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[quad.q[0]], source->points[quad.q[1]], 
                1, 1, 0, ostr.str ().c_str (), vp1);
  ostr.str ("");
  ostr << "line" << lineId++;
  viz->addLine (source->points[quad.q[2]], source->points[quad.q[3]], 
                 1, 1, 0, ostr.str ().c_str (), vp1);

  //for (int i = 0; i < qmatch.matches.size (); i++) 
  {
    Quad& quad = qmatch.matches[qmatch.best_match];
    //Quad& quad = qmatch.matches[i];

    for (int k = 0; k <4; k++) {
      ostr << "sphere" << sphereId++;
      Point& pt = target->points[quad.q[k]];
      viz->addSphere (pt, sphere_radius, red, green, blue, 
                      ostr.str ().c_str (), vp2);
    }

    ostr.str ("");
    ostr << "line" << lineId++;
    int x1 = quad.q[0], x2 = quad.q[1], x3 = quad.q[2], x4 = quad.q[3];
    viz->addLine (target->points[x1], target->points[x2], 1, 1, 0, 
                  ostr.str ().c_str (), vp2);

    ostr.str ("");
    ostr << "line" << lineId++;
    viz->addLine (target->points[x3], target->points[x4], 1, 1, 0,
                  ostr.str ().c_str (), vp2);

    Point intersection;
    findIntersection (target, x1, x2, x3, x4, intersection);

    Eigen::Vector3f e = intersection.getVector3fMap ();

    ostr.str ("");
    ostr << "intersect" << intersectId++;
    viz->addSphere (intersection, sphere_radius, 1, 1, 0,
                    ostr.str ().c_str (), vp2);

    // a, b, c and d are the four points
    //Eigen::Vector3f a = target->points[x1].getVector3fMap ();
    //Eigen::Vector3f b = target->points[x2].getVector3fMap ();
    //Eigen::Vector3f c = target->points[x3].getVector3fMap ();
    //Eigen::Vector3f d = target->points[x4].getVector3fMap ();

    //float r1 = (a-e).norm () / (a-b).norm ();
    //float r2 = (c-e).norm () / (c-d).norm ();

    //cout << " --------\n";
    //cout << "r1 = " << qmatch.quad->r1 << " " << r1 << endl;
    //cout << "r2 = " << qmatch.quad->r2 << " " << r2 << endl;
    //cout << " --------\n";
  }
}

void Extended4PCS::displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
                                      char* name, int& viewport)
{
  PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
  viz->addPointCloud (cloud, tgt_h, name, viewport);
}


void Extended4PCS::findSimilarQuads ()
{
  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];
    
    KdTree& kdtree = qmatch.kdtree;
    Quad* quad = qmatch.quad;
    float r1 (quad->r1);
    float r2 (quad->r2);

    vector <int>& r1_pts = *(qmatch.r1_pts);

    //  Creating a point cloud containing r1 intersections
    //  A KdTree is then created for this point cloud
    CloudPtr intersections (new Cloud);

    for (int i = 0; i < r1_pts.size (); i += 2) {
      Eigen::Vector3f a = target->points[r1_pts[i]].getVector3fMap ();
      Eigen::Vector3f b = target->points[r1_pts[i+1]].getVector3fMap ();

      if ( ( (a-b).norm () - qmatch.ab_len)  > param.length_similarity_threshold){
        //cout << "*** Point pairs NOT MATCHING AB length criteria ..\n";
      }
      else {
        //cout << "*** Point pairs MATCHING AB length criteria ..\n";
      }

      Eigen::Vector3f e1_1 = a + r1 * (b-a);
      Point pt1 (e1_1 (0), e1_1 (1), e1_1 (2));
      intersections->points.push_back (pt1);


      Eigen::Vector3f e1_2 = b + r1 * (a-b);
      Point pt2 (e1_2 (0), e1_2 (1), e1_2 (2));
      intersections->points.push_back (pt2);
    }

    //cout << "# of point pairs matching AB = " << r1_pts.size () << endl;
    kdtree.setInputCloud (intersections);
    //cout << "# of r1 intersections = " << intersections->points.size () << endl;


    vector <int>& r2_pts = *(qmatch.r2_pts);

    for (int i = 0; i < r2_pts.size (); i += 2) {
      Eigen::Vector3f a = target->points[r2_pts[i]].getVector3fMap ();
      Eigen::Vector3f b = target->points[r2_pts[i+1]].getVector3fMap ();

      if ( ( (a-b).norm () - qmatch.cd_len)  > param.length_similarity_threshold) {
        //cout << "*** Point pairs NOT MATCHING CD length criteria ..\n";
      }
      else {
        //cout << "*** Point pairs MATCHING CD length criteria ..\n";
      }

      Eigen::Vector3f e2_1 = a + r2 * (b-a);
      Point pt1 (e2_1 (0), e2_1 (1), e2_1 (2));

      int index = findMatchingPoint (kdtree, pt1); // returns index from cloud created in the above loop
      if (index != -1) {
        if ( (index % 2) == 0) {
          // Matching quad pairs are {r2_pts[i], r2_pts[i+1], r1_pts[index], r1_pts[index+1]}
          // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
          Quad mquad;
          mquad.q[0] = r1_pts[index];
          mquad.q[1] = r1_pts[index+1];
          mquad.q[2] = r2_pts[i];
          mquad.q[3] = r2_pts[i+1];
          //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
          //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
          //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

          if (angleCheck (*quad, mquad)) {
            qmatch.matches.push_back (mquad);
          }
        }
        else {
          // Matching quad pairs are {r2_pts[i], r2_pts[i+1], r1_pts[index], r1_pts[index-1]}
          // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
          Quad mquad;
          mquad.q[0] = r1_pts[index];
          mquad.q[1] = r1_pts[index-1];
          mquad.q[2] = r2_pts[i];
          mquad.q[3] = r2_pts[i+1];
          //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
          //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
          //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

          if (angleCheck (*quad, mquad)) {
            qmatch.matches.push_back (mquad);
          }
        }
      }

      Eigen::Vector3f e2_2 = b + r2 * (a-b);
      Point pt2 (e2_2 (0), e2_2 (1), e2_2 (2));

      index = findMatchingPoint (kdtree, pt2);
      if (index != -1) {
        if ( (index % 2) == 0) {
          // Matching quad pairs are (r2_pts[i+1], r2_pts[i], r1_pts[index], r1_pts[index+1])
          // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }

          Quad mquad;
          mquad.q[0] = r1_pts[index];
          mquad.q[1] = r1_pts[index+1];
          mquad.q[2] = r2_pts[i+1];
          mquad.q[3] = r2_pts[i];
          //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
          //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
          //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

          if (angleCheck (*quad, mquad)) {
            qmatch.matches.push_back (mquad);
          }
        }
        else {
          // Matching quad pairs are (r2_pts[i+1], r2_pts[i], r1_pts[index], r1_pts[index-1])
          // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
          
          Quad mquad;
          mquad.q[0] = r1_pts[index];
          mquad.q[1] = r1_pts[index-1];
          mquad.q[2] = r2_pts[i+1];
          mquad.q[3] = r2_pts[i];
          //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
          //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
          //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

          if (angleCheck (*quad, mquad)) {
            qmatch.matches.push_back (mquad);
          }
        }
      }
    }
  }
}

bool Extended4PCS::angleCheck (Quad& quad, Quad& mquad)
{

  Eigen::Vector3f a1 = source->points[quad.q[0]].getVector3fMap ();
  Eigen::Vector3f c1 = source->points[quad.q[2]].getVector3fMap ();
  Point e1_pt;
  findIntersection (source, quad.q[0], quad.q[1], quad.q[2], quad.q[3], e1_pt);
  Eigen::Vector3f e1 = e1_pt.getVector3fMap ();
  quad.intersection = e1;

  Eigen::Vector3f u1 = (a1-e1);
  Eigen::Vector3f v1 = (c1-e1);

  Eigen::Vector3f a2 = target->points[mquad.q[0]].getVector3fMap ();
  Eigen::Vector3f c2 = target->points[mquad.q[2]].getVector3fMap ();
  Point e2_pt;
  findIntersection (target, mquad.q[0], mquad.q[1], mquad.q[2], mquad.q[3], e2_pt);
  Eigen::Vector3f e2 = e2_pt.getVector3fMap ();
  mquad.intersection = e2;

  Eigen::Vector3f u2 = (a2-e2);
  Eigen::Vector3f v2 = (c2-e2);

  float angle1 = atan2 (u1.cross (v1).norm (), u1.dot (v1)) * 180. / M_PI;
  float angle2 = atan2 (u2.cross (v2).norm (), u2.dot (v2)) * 180. / M_PI;

  quad.intersect_angle = angle1;
  mquad.intersect_angle = angle2;


  if (fabs (angle1 - angle2) < param.angle_threshold){
    return true;
  }

  return false;
}

int Extended4PCS::findMatchingPoint (KdTree& kdtree, Point pt)
{
  int K = 1;
  vector <int> ids (K);
  vector <float> dist (K);

  if (kdtree.nearestKSearch (pt, K, ids, dist) > 0) {
    if (dist[0] <= param.e_match_threshold) {
      //cout << "dist = " << dist[0] << endl;
      return ids[0];
    }
  }

  return -1;
}

void Extended4PCS::insertToQuadMatchTable (float length, int p, int q)
{
  for (int i = 0; i < quadMatchTable.size (); i++) {
    QuadMatch& qmatch = quadMatchTable[i];
    //length within 1 meter of the pair in the quad
    if ( fabs (qmatch.ab_len - length) < param.length_similarity_threshold) {
      qmatch.r1_pts->push_back (p);
      qmatch.r1_pts->push_back (q);
      break;
    }
    //length within 1 meter of pair in the quad
    if ( fabs (qmatch.cd_len - length) < param.length_similarity_threshold) {
      qmatch.r2_pts->push_back (p);
      qmatch.r2_pts->push_back (q);
      break;
    }
  }
}

void Extended4PCS::initializeQuadMatchTable ()
{
  quadMatchTable.resize (quads.size ());

  for (int i = 0; i < quads.size (); i++) {

    quadMatchTable[i].quad = &quads[i];

    Eigen::Vector3f a = source->points[quads[i].q[0]].getVector3fMap ();
    Eigen::Vector3f b = source->points[quads[i].q[1]].getVector3fMap ();
    Eigen::Vector3f c = source->points[quads[i].q[2]].getVector3fMap ();
    Eigen::Vector3f d = source->points[quads[i].q[3]].getVector3fMap ();

    quadMatchTable[i].ab_len = (a-b).norm ();
    quadMatchTable[i].cd_len = (c-d).norm ();

    bool found = false;
    vector <PointList>::iterator itr = pointListTable.begin ();
    for (; itr != pointListTable.end (); itr++) {
      // point within 1 m
      if ( fabs (itr->len - quadMatchTable[i].ab_len) < 
          param.length_similarity_threshold) {
        found = true;
        break;
      }
    }

    if (!found) {
      PointList pl;
      pl.len = quadMatchTable[i].ab_len;
      pointListTable.push_back (pl);
    }

    found = false;
    itr = pointListTable.begin ();
    for (; itr != pointListTable.end (); itr++) {
      if ( fabs (itr->len - quadMatchTable[i].cd_len) < 
          param.length_similarity_threshold) {
        found = true;
        break;
      }
    }
    if (!found) {
      PointList pl;
      pl.len = quadMatchTable[i].cd_len;
      pointListTable.push_back (pl);
    }
  }


  for (int i = 0; i < quadMatchTable.size (); i++) {
    vector <PointList>::iterator itr = pointListTable.begin ();
    for (; itr != pointListTable.end (); itr++) {
      // point within 1 m
      if ( fabs (itr->len - quadMatchTable[i].ab_len) < 
          param.length_similarity_threshold) {
        quadMatchTable[i].r1_pts = &(itr->points);
      }
      if ( fabs (itr->len - quadMatchTable[i].cd_len) < 
          param.length_similarity_threshold) {
        quadMatchTable[i].r2_pts = &(itr->points);
      }
    }
  }
}

void Extended4PCS::selectOneQuad (Quad& quad, vector <int>& plane_pts)
{
  int z = 0;

  while (true) {

    if (z==4) {
      break;
    }

    int k = rand () % plane_pts.size ();

    bool too_close = false;
    for (int p = 0; p < z; p++) {
      Point p1 = source->points[plane_pts[k]];
      Point p2 = source->points[quad.q[p]];
      if ( (p1.getVector3fMap () - p2.getVector3fMap ()).norm () < param.min_dist) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      continue;
    }

    quad.q[z++] = plane_pts[k];
  }
}


void Extended4PCS::findIntersection (CloudPtr cloud, int x1, int x2, 
                              int x3, int x4, Point& intersection)
{
  Eigen::Vector3f a = cloud->points[x2].getVector3fMap () - 
                      cloud->points[x1].getVector3fMap () ;

  Eigen::Vector3f b = cloud->points[x4].getVector3fMap () - 
                      cloud->points[x3].getVector3fMap () ;

  Eigen::Vector3f c = cloud->points[x3].getVector3fMap () - 
                      cloud->points[x1].getVector3fMap () ;

  Eigen::Vector3f cb = c.cross (b);
  Eigen::Vector3f ab = a.cross (b);

  // e is the intersection
  Eigen::Vector3f e = cloud->points[x1].getVector3fMap () + 
                      a * (cb.dot (ab) / ab.dot (ab));

  intersection.x = e (0);
  intersection.y = e (1);
  intersection.z = e (2);
}

bool Extended4PCS::checkQuad (Quad& quad)
{
  // check the quad by verifying if the line segements intersect
  // using method described in "http://mathworld.wolfram.com/Line-LineIntersection.html"
  // to compute 3D line intersection
  int& x1 = quad.q[0];
  int& x2 = quad.q[1];
  int& x3 = quad.q[2];
  int& x4 = quad.q[3];

  Point intersection;
  findIntersection (source, x1, x2, x3, x4, intersection);



  Eigen::Vector3f e = intersection.getVector3fMap ();

  // a, b, c and d are the four points
  Eigen::Vector3f a = source->points[x1].getVector3fMap ();
  Eigen::Vector3f b = source->points[x2].getVector3fMap ();
  Eigen::Vector3f c = source->points[x3].getVector3fMap ();
  Eigen::Vector3f d = source->points[x4].getVector3fMap ();


  float length = (a-b).norm ();
  // e should be on the line segment ab, so |e-a| or |e-b| 
  // should not exceed the length of |a-b|
  if ( ((e-b).norm () > length) || ((e-a).norm () > length) ) {
    //cout << "Quad check failed..\n";
    return false;
  }

  length = (c-d).norm ();
  // e should be on the line segment cd, so |e-c| or |e-d| 
  // should not exceed the length of |c-d|
  if ( ((e-c).norm () > length) || ((e-d).norm () > length) ) {
    //cout << "Quad check failed..\n";
    return false;
  }

  float r1 = (a-e).norm () / (a-b).norm ();
  float r2 = (c-e).norm () / (c-d).norm ();

  quad.r1 = r1;
  quad.r2 = r2;

  //cout << "r1 = " << r1 << " r2 = " << r2 << endl;

  //static int lineid = 5000;
  //ostringstream ostr;
  //ostr << "line" << lineid++;
  //viz->addLine (source->points[x1], source->points[x2], 1, 1, 0, ostr.str ().c_str ());
  //ostr.str ("");
  //ostr << "line" << lineid++;
  //viz->addLine (source->points[x3], source->points[x4], 1, 1, 0, ostr.str ().c_str ());
  //Point pi (e (0), e (1), e (2));

  //static int line_intersect_id = 4500;
  //ostr.str ("");
  //ostr << "lineintersect" << line_intersect_id++;
  //viz->addSphere (pi, sphere_radius, 1, 1, 0, ostr.str ().c_str ());

  return true;
}

void Extended4PCS::selectQuads (vector <int>& plane_pts, int N)
{
  int quadId = 1;

  Quad quad;
  // Make sure that atleast one quad gets selected ..
  do {
    selectOneQuad (quad, plane_pts);
    if (checkQuad (quad)) {
      quad.quadId = quadId++;
      quads.push_back (quad);
      break;
    }
    //cout << "Quad check failed ..\n";
  } while (true);

  cout << "\n<< Selected quad 1 >> ..\n";
  // Select the rest N-1 quads
  for (int i = 2; i <= N; i++) { // Select N quads

    Quad quad;

    int tries = 1;
    do {
      selectOneQuad (quad, plane_pts);
      if (checkQuad (quad)) {
        quad.quadId = quadId++;
        quads.push_back (quad);
        break;
      }
      //cout << "Quad check failed ..\n";
    } while (tries++ < 10);

    //if (tries == 11) {
    //  cout << "<< Could not select quad " << i << " tries = " << tries << " >>\n";
    //}
    //else {
      //cout << "<< Selected quad " << i << " Number of tries  = " << tries << " >> .. \n";
      cout << "<< Selected quad " << i << " >> .. \n";
    //}

    //cout << "\nPairwise distances\n-----------------\n";
    //for (int i = 0; i < 4; i++) {
    //  for (int j = 0; j < 4; j++) {
    //    if (i == j) continue;
    //    Point p1 = source->points[quad.q[i]];
    //    Point p2 = source->points[quad.q[j]];

    //    float dist = (p1.getVector3fMap () - p2.getVector3fMap ()).norm ();
    //    cout << dist <<  " ";
    //  }
    //}
    //cout << "\n-----------------\n";
    }
}

void Extended4PCS::selectMaxPlane ()
{
  int a = -1, b = -1, c = -1;

  int z = 0;
  int max_pts = 0;


  srand (time (NULL));

  cout << "FINDING THE BEST PLANE :: ";
  while (z++ < param.random_tries) {
    int ta = 0, tb = 0, tc = 0;
    select3Points (source, ta, tb, tc);
    vector <int> pts;
    findPointsOnPlane (source, ta, tb, tc, pts);
    cout << pts.size () << " ";

    if (pts.size () > max_pts) {
      max_pts = pts.size ();
      a = ta, b = tb, c = tc;
    }
    //cout << "z = " << z << " max points = " << max_pts << endl;
  }
  cout << endl;

  findPointsOnPlane (source, a, b, c, plane_pts);
  cout << "# of points on the plane = " << plane_pts.size () << endl;
}

void Extended4PCS::findPointsOnPlane (CloudPtr cloud, int a, int b,
                              int c,  vector <int>& pts)
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

    //cout << "Eigen values = " << eigenvalues[0] << " " << eigenvalues[1] << " " << eigenvalues[2] << endl;

    float A = eigenvector3 (0);
    float B = eigenvector3 (1);
    float C = eigenvector3 (2);
    float D = - (A*centroid (0) + B*centroid (1) + C*centroid (2));
    //cout << "Plane equation = (" << A << " " << B << " " << C << " " << D << ")" << endl;
    
    pts.push_back (a), pts.push_back (b), pts.push_back (c);

    for (int i = 0; i < cloud->points.size (); i++) {

      if (i == a || i == b || i == c) {
        continue;
      }

      float x = cloud->points[i].x;
      float y = cloud->points[i].y;
      float z = cloud->points[i].z;

      float residue = fabs (A*x + B*y + C*z + D);
      if ( residue < param.plane_fit_threshold) {
        pts.push_back (i);
        //cout << "residue = " << residue << endl;
      }
    }

    //cout << "# of points = " << cloud->points.size () << endl;
    //cout << "# of points on the plane = " << pts.size () << endl;
}

void Extended4PCS::select3Points (CloudPtr cloud, int& a, int& b, int& c)
{
  int N = cloud->points.size ();


  a = rand () % N;
  b = rand () % N;
  c = rand () % N;

  Point pa = cloud->points[a];
  Point pb = cloud->points[b];
  Point pc = cloud->points[c];

  Eigen::Vector3f u = pa.getVector3fMap () - pb.getVector3fMap ();
  Eigen::Vector3f v = pa.getVector3fMap () - pc.getVector3fMap ();

  if (0.5 * u.cross (v).norm () // area of the traingle formed by
      // the 3 points should not be zero, this is a check for collinearity
      && (u.norm () > param.min_dist) // minimum distance between selected points
      && (v.norm () > param.min_dist) ) {
    return;
  }
}


}
