#pragma once


#include <pcl/io/pcd_io.h>

#include "typedefs.h"

#include <iostream>
using namespace std;

using namespace E4PCS;

CloudPtr readPointCloud (string filename, string filetype);
void writePointCloud (CloudPtr cloud, string filename, 
                      string filetype="ascii");

void readASCIIFile (const char* filename, CloudPtr cloud);
void readPCDBinaryFile (const char* filename, CloudPtr cloud);

void writeASCIIFile (const char* filename, CloudPtr cloud);
