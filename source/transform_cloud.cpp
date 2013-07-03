#include <iostream>
#include <string>
#include <fstream>
using namespace std;

#include <pcl/registration/transforms.h>
#include "pcl/io/pcd_io.h"

#include "typedefs.h"
#include "io.h"

string inputfile, matfile, outputfile;

int main (int argc, char * argv[]) 
{
  if (argc < 4) {
    cout << "\nEnter\n"
            "\t1) Input cloud\n"
            "\t2) transformation file\n"
            "\t3) output file name\n\n";
    return -1;
  }

  inputfile = argv[1];
  matfile = argv[2];
  outputfile = argv[3];

	ifstream ifile (matfile.c_str ());
	if (!ifile) {
		cout << "File could not be opened - " << matfile << endl;
		return -1;
	}

	vector<float> elements (16);

	for(int i=0; i<16; i++) {
    ifile >> elements[i];
  }

	Eigen::Matrix4f transform;
	transform << elements[0], elements[1], elements[2], elements[3],
		  elements[4], elements[5], elements[6], elements[7],
		  elements[8], elements[9], elements[10], elements[11],
		  elements[12], elements[13], elements[14], elements[15];

	cout << transform << endl;

	ofstream ofile (outputfile.c_str ());
	if (!ofile) {
		cout << "Cannot create output file.. Check the path..\n";
		return -1;
	}
	else {
		remove (outputfile.c_str ());
	}

	CloudPtr cloud1 (new Cloud);

  readASCIIFile (inputfile.c_str (), cloud1);

  float centroid[3] = {0};
  for (int i = 0; i < cloud1->points.size (); i++) {
    centroid[0] += cloud1->points[i].x;
    centroid[1] += cloud1->points[i].y;
    centroid[2] += cloud1->points[i].z;
  }
  
  centroid[0] /= cloud1->points.size ();
  centroid[1] /= cloud1->points.size ();
  centroid[2] /= cloud1->points.size ();

  for (int i = 0; i < cloud1->points.size (); i++) {
    cloud1->points[i].x -= centroid[0];
    cloud1->points[i].y -= centroid[1];
    cloud1->points[i].z -= centroid[2];
  }

	CloudPtr cloud2 (new Cloud);
	transformPointCloud (*cloud1, *cloud2, transform);

  for (int i = 0; i < cloud2->points.size (); i++) {
    cloud2->points[i].x += centroid[0];
    cloud2->points[i].y += centroid[1];
    cloud2->points[i].z += centroid[2];
  }

  writeASCIIFile (outputfile.c_str (), cloud2);

	return 0;
}
