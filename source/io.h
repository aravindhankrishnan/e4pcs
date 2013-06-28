#pragma once


#include <pcl/io/pcd_io.h>

#include "typedefs.h"

using namespace E4PCS;

void readPCDBinaryFile (const char* filename, CloudPtr cloud);

void readASCIIFile (const char* filename, CloudPtr cloud);
void writeASCIIFile (const char* filename, CloudPtr cloud);
