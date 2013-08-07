#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "io.h"
#include "e4pcs.h"
#include "config.h"
#include "typedefs.h"
#include "keypoints_interface.h"

#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;


void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud,
                  string samplingtype)
{
  if (samplingtype.compare ("random") == 0) {
    if (cloud->points.size () <= N) {
      pcl::copyPointCloud (*cloud, *sampledcloud);
      sampledcloud->width = 1;
      sampledcloud->height = sampledcloud->points.size ();
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
    return;
  }


  pcl::UniformSampling<Point> us;
  us.setInputCloud (cloud);
  double radius = 5;
  us.setRadiusSearch (radius);
  pcl::PointCloud<int> subsampled_indices;
  us.compute (subsampled_indices);
  std::sort (subsampled_indices.points.begin (), subsampled_indices.points.end ());
  pcl::copyPointCloud (*cloud, subsampled_indices.points, *sampledcloud);

  cout << "Points = " << cloud->points.size () << " "
    << sampledcloud->points.size () << endl;

}


void displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
                        char* name, int& viewport)
{
  PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
  viz->addPointCloud (cloud, tgt_h, name, viewport);
}


void addGaussianNoise (CloudPtr cloud, float sd)
{
  using namespace boost;
  mt19937 rng (time (0));
  normal_distribution<> nd (0.0, sd);

  variate_generator <mt19937&, normal_distribution<> > noise (rng, nd);


  BOOST_FOREACH (Point& pt, cloud->points) {
    float nx = noise ();
    float ny = noise ();
    float nz = noise ();

    pt.x += nx;
    pt.y += ny;
    pt.z += nz;
  }
}

void findMinMax (CloudPtr cloud, float& minx, float& miny, 
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

void alignToXY (CloudPtr cloud, CloudPtr projectedcloud)
{
  Eigen::Vector3f centroid;
  centroid.setZero ();

  BOOST_FOREACH (Point& pt, cloud->points) {
    centroid += pt.getVector3fMap ();
  }

  centroid /= cloud->points.size ();

  Eigen::Matrix3f covariance;
  covariance.setZero ();

  BOOST_FOREACH (pcl::PointXYZ& pt, cloud->points) {
    Eigen::Vector3f V = (pt.getVector3fMap () - centroid);
    covariance += (V * V.transpose ());
  }


  Eigen::EigenSolver <Eigen::Matrix3f> es (covariance);
  cout << "The eigen values are " << es.eigenvalues () << endl;


  Eigen::Vector3d eigenvalues;
  eigenvalues.setZero ();
  pcl::eigen33 (covariance, eigenvalues);
  cout << "The eigen values are " << eigenvalues.transpose () << endl << endl;

  Eigen::Vector3f eigenvector1;
  eigenvector1.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [2], eigenvector1);

  Eigen::Vector3f eigenvector2;
  eigenvector2.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [1], eigenvector2);
  
  Eigen::Vector3f eigenvector3;
  eigenvector3.setZero ();
  pcl::computeCorrespondingEigenVector (covariance, eigenvalues [0], eigenvector3);

  BOOST_FOREACH (Point& pt, cloud->points) {
    Eigen::Vector3f c = pt.getVector3fMap ();
    float x = c.dot (eigenvector1);
    float y = c.dot (eigenvector2);
    float z = c.dot (eigenvector3);

    projectedcloud->points.push_back (Point (x, y, z));
  }
  projectedcloud->width = 1;
  projectedcloud->height = projectedcloud->points.size ();
}

void makeEven (CloudPtr cloud1, CloudPtr cloud2)
{
  CloudPtr projectedcloud1 (new Cloud);
  CloudPtr projectedcloud2 (new Cloud);

  alignToXY (cloud1, projectedcloud1);
  alignToXY (cloud2, projectedcloud2);

  float minx1, maxx1, miny1, maxy1;
  float minx2, maxx2, miny2, maxy2;

  findMinMax (projectedcloud1, minx1, miny1, maxx1, maxy1);
  findMinMax (projectedcloud2, minx2, miny2, maxx2, maxy2);

  cout << "width = " << fabs (maxx1 - minx1) << " " 
    << fabs (maxx2 - minx2) << endl;

  cout << "height = " << fabs (maxy1 - miny1) << " " 
    << fabs (maxy2 - miny2) << endl;

  float dx =  ((maxx2 - minx2) - (maxx1 - minx1)) / 2 + 5;
  float dy =  ((maxy2 - miny2) - (maxy1 - miny1)) / 2 + 5;

  CloudPtr newcloud2 (new Cloud);

  for (int i = 0; i < cloud2->points.size (); i++) {
    Point pt = projectedcloud2->points[i];
    if ( (pt.x >= minx2 + dx) && (pt.x <= (maxx2 - dx)) &&
         (pt.y >= miny2 + dy) && (pt.y <= (maxy2 - dy)) ) {
      newcloud2->points.push_back (cloud2->points[i]);
    }
  }
  newcloud2->width = 1;
  newcloud2->height = newcloud2->points.size ();

  cout << "Make even =  " << cloud2->points.size () << " " 
    << newcloud2->points.size () << endl;
  
  cloud2->points.clear ();
  
  copyPointCloud (*newcloud2, *cloud2);
  cloud2->width = 1;
  cloud2->width = cloud2->points.size ();

}


int main (int argc, char *argv[])
{
  try {

  if (argc < 2) {
    cout << "Enter the configuration file ..\n";
    return -1;
  }

  if (loadConfigFile (argv[1]) == -1) {
    return -1;
  }

  InputParamsPtr args = getInputParams ();

  //return 0;

  using namespace E4PCS;

  CloudPtr cloud1 ( new Cloud );
  CloudPtr cloud2 ( new Cloud );

  if (args->filetype1.compare ("ascii") == 0) {
    readASCIIFile (args->sourcefile.c_str (), cloud1);
  }
  else if (args->filetype1.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (args->sourcefile.c_str (), cloud1);
  }

  if (args->filetype2.compare ("ascii") == 0) {
    readASCIIFile (args->targetfile.c_str (), cloud2);
  }
  else if (args->filetype2.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (args->targetfile.c_str (), cloud2);
  }

  PCLVisualizer* p = new PCLVisualizer (argc, argv, "Initial data");

  int vp1 = 1;
  p->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);

  int vp2 = 2;
  p->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);
  //p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255, vp2);

  //p->setBackgroundColor (25./255, 25.0/255, 25.0/255);
  //p->setBackgroundColor (255, 255, 255, vp2);

  float sd = args->D / 6.; // the point can lie in a 'D/2' m radius. so 6 sigma = D.
  addGaussianNoise (cloud2, sd);

  //CloudPtr sampledcloud1 (new Cloud);
  //CloudPtr sampledcloud2 (new Cloud);

  //if (samplingtype.compare ("random") == 0) {
  //  sampleCloud (cloud1, numPoints1, sampledcloud1, samplingtype);
  //  sampleCloud (cloud2, numPoints2, sampledcloud2, samplingtype);
  //}
  //else if (samplingtype.compare ("uniform") == 0) {
  //  CloudPtr tmp1 (new Cloud);
  //  CloudPtr tmp2 (new Cloud);

  //  sampleCloud (cloud1, numPoints1, tmp1, "random");
  //  sampleCloud (cloud2, numPoints2, tmp2, "random");

  //  sampleCloud (tmp1, numPoints1, sampledcloud1, samplingtype);
  //  sampleCloud (tmp2, numPoints2, sampledcloud2, samplingtype);
  //}

  //cout << "Before makeEven " << sampledcloud2->points.size () << endl;
  //makeEven (sampledcloud1, sampledcloud2);
  //cout << "After makeEven " << sampledcloud2->points.size () << endl;

  //return 0;


  //Extended4PCS e4pcs (args->D, args->abcd_mindist, args->corr_max_range);
  Extended4PCS e4pcs;
  e4pcs.setArgs (argc, argv);
  e4pcs.setSource (cloud1);
  e4pcs.setTarget (cloud2);
  e4pcs.setCongruency (args->congruency);
  e4pcs.setSamplingType (args->sampling_type);
  e4pcs.setOffset (args->offset);
  e4pcs.setD (args->D);
  e4pcs.setABCDMinDist (args->abcd_mindist);
  e4pcs.setCorrMaxRange (args->corr_max_range);
  e4pcs.setVisualizer (p);
  e4pcs.setVisualizationSphereRadius (args->sphere_radius);
  e4pcs.setNumQuads (args->num_quads);
  e4pcs.setKeypointParams (args->keypoint_par);
  e4pcs.initParams ();

  if (args->sampling_type.find ("random") != string::npos) {
    e4pcs.setSamplingRatio1 (args->random_sampling_ratio1);
  }

  if (args->sampling_type.compare ("random") == 0) {
    e4pcs.setSamplingRatio2 (args->random_sampling_ratio2);
  }

  if (args->sampling_type.compare ("keypointsandregionsaround") == 0) {
    e4pcs.setRadiusOfRegionAroundKeypoint (args->region_around_radius);
  }

  if (args->sampling_type.compare ("randomonwindows") == 0) {
    e4pcs.setWindowSize (args->windowsize);
  }

  e4pcs.align (); 

  Eigen::Matrix4f transformation;

  e4pcs.getTransformation (transformation);

  cout << "\n\n";

  //cout << "# of points in cloud 1 = " << cloud1->points.size () << endl;
  //cout << "# of points in cloud 2 = " << cloud2->points.size () << endl;
  //cout << "# of points on the MAX plane = " << e4pcs.getPlanePoints ().size () << endl;

  ofstream ofile ("transform.txt");
  ofile << transformation << "\n";
  ofile.close ();


  cout << "\n\nVisualizing ..\n\n";

  PCLVisualizer* final = new PCLVisualizer (argc, argv, "Final output");

  CloudPtr op_cloud1 (new Cloud);
  CloudPtr op_cloud2 (new Cloud);
  //op_cloud1 = sampledcloud1;
  //op_cloud2 = sampledcloud2;
  sampleCloud (cloud1, args->vis_num_points, op_cloud1, "random");
  sampleCloud (cloud2, args->vis_num_points, op_cloud2, "random");

  cout << "# points in sampled cloud 1 = " << op_cloud1->points.size () << endl;
  cout << "# points in sampled cloud 2 = " << op_cloud2->points.size () << endl;

  final->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  final->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  final->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);


  CloudPtr op_cloud3 (new Cloud);
  Eigen::Matrix3f R = transformation.block <3, 3> (0, 0);
  Eigen::Vector3f T = transformation.block <3, 1> (0, 3);

  BOOST_FOREACH (Point& pt, op_cloud1->points) {
    Eigen::Vector3f v = R * pt.getVector3fMap () + T;
    Point np (v (0), v (1), v (2));
    op_cloud3->points.push_back (np);
  }
  op_cloud3->width = 1;
  op_cloud3->height = op_cloud3->points.size ();


  cout << "# points in cloud 3 = " << op_cloud3->points.size () << endl;

  int vp0 = 0;
  int color[3] = { 255, 0, 0};
  displayPointCloud (final, op_cloud1, color, (char *) "opcloud1", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (final, op_cloud2, color, (char *) "opcloud2", vp1);

  color[0] = 0; color[1] = 255; color[2] = 0;
  displayPointCloud (final, op_cloud2, color, (char *) "opcloud11", vp2);

  color[0] = 255; color[1] = 0; color[2] = 0;
  displayPointCloud (final, op_cloud3, color, (char *) "opcloud12", vp2);


  p->spin ();
  final->spin ();

  vector <PCLVisualizer*>& V = e4pcs.getVisualizers ();

  BOOST_FOREACH (PCLVisualizer* p, V) {
    p->spin ();
  }

  boost::shared_ptr <PCLVisualizer> keypointsviz = e4pcs.getKeypointsVisualizer();
  keypointsviz->spin ();

  }
  catch (std::exception& e) {
    cout << "\n\nException :: " << e.what () << "\n\n";
  }

  return 0;
}
