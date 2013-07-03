#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

float tx;
float ty;
float tz;

string outputfile;

int main (int argc, char *argv[])
{
  if ( argc < 8) {
    cout << "Enter all parameters ..\n";
    return -1;
  }
  
  roll = atof (argv[1]);
  pitch = atof (argv[2]);
  yaw = atof (argv[3]);

  tx = atof (argv[4]);
  ty = atof (argv[5]);
  tz = atof (argv[6]);

  outputfile = argv[7];

  // converting to radians;

  roll = roll * M_PI / 180;
  pitch = pitch * M_PI / 180;
  yaw = yaw * M_PI / 180;

  Eigen::Matrix3f rotation_x;
  Eigen::Matrix3f rotation_y;
  Eigen::Matrix3f rotation_z;

  // ROTATION X
  rotation_x (0, 0) = 1;
  rotation_x (0, 1) = 0;
  rotation_x (0, 2) = 0;

  rotation_x (1, 0) = 0;
  rotation_x (1, 1) = cos (roll);
  rotation_x (1, 2) = -sin (roll);

  rotation_x (2, 0) = 0;
  rotation_x (2, 1) = sin (roll);
  rotation_x (2, 2) = cos (roll);

  // ROTATION Y

  rotation_y (0, 0) = cos (pitch);
  rotation_y (0, 1) = 0;
  rotation_y (0, 2) = sin (pitch);

  rotation_y (1, 0) = 0;
  rotation_y (1, 1) = 1;
  rotation_y (1, 2) = 0;

  rotation_y (2, 0) = -sin (pitch);
  rotation_y (2, 1) = 0;
  rotation_y (2, 2) = cos (pitch);

  // ROTATION Z

  rotation_z (0, 0) = cos (yaw);
  rotation_z (0, 1) = -sin (yaw);
  rotation_z (0, 2) = 0;

  rotation_z (1, 0) = sin (yaw);
  rotation_z (1, 1) = cos (yaw);
  rotation_z (1, 2) = 0;

  rotation_z (2, 0) = 0;
  rotation_z (2, 1) = 0;
  rotation_z (2, 2) = 1;


  Eigen::Matrix3f rotation = rotation_x * rotation_y * rotation_z;

  Eigen::Matrix4f transformation;
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      transformation (i, j) = rotation (i,j);
    }
  }
  transformation (3, 0) = transformation (3, 1) = transformation (3, 2) = 0;
  transformation (3, 3) = 1;

  transformation (0, 3) = tx;
  transformation (1, 3) = ty;
  transformation (2, 3) = tz;


  ofstream ofile (outputfile.c_str ());
  ofile << transformation << endl;
  cout << transformation << endl;
  ofile.close ();

  return 0;
}
