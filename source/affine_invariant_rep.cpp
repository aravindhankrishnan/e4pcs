#include <iostream>
using namespace std;

#include "io.h"
#include "typedefs.h"
#include "affine_invariant_representation.h"

#include <pcl/visualization/pcl_visualizer.h>
using namespace pcl::visualization;

int main (int argc, char *argv[])
{
  
  if (argc < 5) {
    cout << "Enter arguments..\n"
             "1) Cloud 1\n"
             "2) filetype [ascii / pcdbinary]\n"
             "2) Cloud 2\n"
             "4) filetype [ascii / pcdbinary]\n";
    return -1;
  }


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

  AffineInvariantRepresentation air;
  air.setCloud1 (cloud1);
  air.setCloud2 (cloud2);
  air.compute ();

  CloudPtr aircloud1 (new Cloud);
  CloudPtr aircloud2 (new Cloud);

  aircloud1 = air.getAIRCloud1 ();
  aircloud2 = air.getAIRCloud2 ();

  return 0;
}
