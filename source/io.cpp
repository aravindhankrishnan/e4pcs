#include "io.h"

#include <iostream>
#include <fstream>
using namespace std;

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

void readPCDBinaryFile (const char* filename, CloudPtr cloud)
{
  pcl::io::loadPCDFile (filename, *cloud);
  vector <int> ids;
  pcl::removeNaNFromPointCloud (*cloud, *cloud, ids);
}

void readASCIIFile (const char* filename, CloudPtr cloud)
{
  ifstream ifile (filename);

  if (!ifile) {
    ostringstream ostr;
    ostr << "Couldn't open file " << filename << endl;
    throw std::runtime_error (ostr.str ());
  }
  char buffer[1024];
  int n = 0;

  while (ifile) {
    ifile.getline (buffer, 1024);
    string line = buffer;
    if (line.empty ()) {
      break;
    }
    istringstream istr (line);
    double x = 0., y = 0., z = 0.;
    istr >> x >> y >> z;
    cloud->points.push_back (Point (x, y, z));
    n++;
  }
  cloud->width = n;
  cloud->height = 1;
  cout << "Loaded " << filename << " with " << cloud->size () << " points ..\n";
}

void writeASCIIFile (const char* filename, CloudPtr cloud)
{
  ofstream ofile (filename);
  for (int i = 0; i < cloud->points.size (); i++) {
    ofile << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
  }
  ofile.close ();
}
