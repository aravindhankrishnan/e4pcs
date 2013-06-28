#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "io.h"
#include "e4pcs.h"
#include "typedefs.h"

#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

float D;
float sphere_radius;
float abcd_mindist;
float corr_max_range;
string sourcefile;
string targetfile;
string filetype1;
string filetype2;
int numQuads;
int numPoints;

void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
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


void readLine (ifstream& ifile, string& line)
{
    char buff[2048];
    fill_n (buff, 2048, '\0');
    ifile.getline (buff, 2048);
    line = buff;
}

int loadConfigFile (const char* filename)
{
  cout << endl;

  ifstream ifile (filename);
  if (!ifile) {
    cout << "Cannot load config file .. " << filename << " ..\n\n";
    return -1;
  }

  while (ifile) {

    string line;
    readLine (ifile, line);

    if ( (line.length () == 0) or (line[0] == '#')) {
      continue;
    }

    istringstream istr (line);
    string keyword;
    string val;

    istr >> keyword >> val;

    //cout << "keyword = " << keyword << " val = " << val << endl;

    if (keyword.compare ("source") == 0 ) {
      cout << "Source cloud = " << val << endl;
      sourcefile = val;
      
      readLine (ifile, line);
      //cout << "LINE :: " << line << endl;
      istringstream istr1 (line);

      keyword = "", val = "";
      istr1 >> keyword >> val;
      //cout << "keyword = " << keyword << " val = " << val << endl;
      if (keyword.compare ("type") == 0) {
        cout << "Type = *" << val << "*\n";
        filetype1 = val;
      }
      else {
        cout << "File type not entered for source cloud..\n";
        return -1;
      }
      continue;
    }

    if (keyword.compare ("target") == 0 ) {
      cout << "Target cloud = " << val << endl;
      targetfile = val;

      readLine (ifile, line);

      istringstream istr1 (line);

      keyword = "", val = "";
      istr1 >> keyword >> val;
      //cout << "keyword = " << keyword << " val = " << val << endl;
      if (keyword.compare ("type") == 0) {
        cout << "Type = **" << val << "**\n";
        filetype2 = val;
      }
      else {
        cout << "File type not entered for target cloud..\n";
        return -1;
      }
      continue;
    }

    if (keyword.compare ("numquads") == 0 ) {
      cout << "Number of quads = " << val << endl;
      numQuads = atoi (val.c_str ());
      continue;
    }

    if (keyword.compare ("sampling") == 0 ) {
      cout << "Number of points sampled = " << val << endl;
      numPoints = atoi (val.c_str ());
      continue;
    }

    if (keyword.compare ("errorballdiameter") == 0 ) {
      cout << "Error ball diameter = " << val << endl;
      D = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("abcd_mindist") == 0 ) {
      cout << "ABCD min dist = " << val << endl;
      abcd_mindist = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("corr_max_range") == 0 ) {
      cout << "Correspondence max range = " << val << endl;
      corr_max_range = atof (val.c_str ());
      continue;
    }

    if (keyword.compare ("vis_sphereradius") == 0 ) {
      cout << "Visualization Sphere radius = " << val << endl;
      sphere_radius = atof (val.c_str ());
      continue;
    }
  }
  cout << "file type1 = *" << filetype1 << "*\n";
  cout << "file type2 = *" << filetype2 << "*\n";
  cout << endl << endl;
}


int main (int argc, char *argv[])
{

  if (argc < 2) {
    cout << "Enter the configuration file ..\n";
    return -1;
  }

  if (loadConfigFile (argv[1]) == -1) {
    return -1;
  }

  using namespace E4PCS;

  CloudPtr cloud1 ( new Cloud );
  CloudPtr cloud2 ( new Cloud );

  if (filetype1.compare ("ascii") == 0) {
    readASCIIFile (sourcefile.c_str (), cloud1);
  }
  else if (filetype1.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (sourcefile.c_str (), cloud1);
  }

  if (filetype2.compare ("ascii") == 0) {
    readASCIIFile (targetfile.c_str (), cloud2);
  }
  else if (filetype2.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (targetfile.c_str (), cloud2);
  }

  PCLVisualizer* p = new PCLVisualizer (argc, argv, "Choosing plane");

  int vp1 = 1;
  p->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
  //p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255, vp1);

  int vp2 = 2;
  p->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);

  p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);
  //p->setBackgroundColor (113.0/255, 113.0/255, 154.0/255, vp2);

  //p->setBackgroundColor (25./255, 25.0/255, 25.0/255);
  //p->setBackgroundColor (255, 255, 255, vp2);

  float sd = D / 6.; // the point can lie in a 'D/2' m radius. so 6 sigma = D.
  addGaussianNoise (cloud2, sd);

  CloudPtr sampledcloud1 (new Cloud);
  CloudPtr sampledcloud2 (new Cloud);

  sampleCloud (cloud1, numPoints, sampledcloud1);
  sampleCloud (cloud2, numPoints, sampledcloud2);


  Extended4PCS e4pcs (D, abcd_mindist, corr_max_range);
  e4pcs.setSource (sampledcloud1);
  e4pcs.setTarget (sampledcloud2);
  e4pcs.setVisualizer (p);
  e4pcs.setNumQuads (numQuads);

  e4pcs.align (); 

  Eigen::Matrix4f transformation;

  e4pcs.getTransformation (transformation);

  cout << "\n\n";

  cout << "# of points in cloud 1 = " << cloud1->points.size () << endl;
  cout << "# of points in cloud 2 = " << cloud2->points.size () << endl;
  cout << "# of points on the MAX plane = " << e4pcs.getPlanePoints ().size () << endl;

  ofstream ofile ("transform.txt");
  ofile << transformation << "\n";
  ofile.close ();


  cout << "\n\nVisualizing ..\n\n";

  PCLVisualizer* final = new PCLVisualizer (argc, argv, "Final output");


  CloudPtr op_cloud1 (new Cloud);
  CloudPtr op_cloud2 (new Cloud);
  numPoints = 20000;
  sampleCloud (cloud1, numPoints, op_cloud1);
  sampleCloud (cloud2, numPoints, op_cloud2);

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


  final->spin ();

  p->spin ();

  return 0;
}
