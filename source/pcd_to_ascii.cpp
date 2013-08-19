#include <iostream>
using namespace std;

#include "typedefs.h"
#include "io.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

using namespace E4PCS;

int main (int argc, char *argv[])
{
  if (argc < 3) {
    cout << "Enter arguments ..\n"
            "\t1) Input file\n"
            "\t2) Outputfile\n";
    return -1;
  }

  CloudIntensityPtr cloud (new CloudIntensity);
  pcl::io::loadPCDFile (argv[1], *cloud);
  vector <int> ids;
  pcl::removeNaNFromPointCloud (*cloud, *cloud, ids);

  CloudPtr ocloud (new Cloud);
  copyPointCloud (*cloud, *ocloud);

  writePointCloud (ocloud, argv[2]);

  return 0;
}
