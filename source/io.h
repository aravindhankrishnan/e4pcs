#pragma once


#include <pcl/io/pcd_io.h>

#include "typedefs.h"

using namespace E4PCS;

void readPCDBinaryFile (char* filename, CloudPtr cloud);

void readASCIIFile (char* filename, CloudPtr cloud);
void writeASCIIFile (char* filename, CloudPtr cloud);
