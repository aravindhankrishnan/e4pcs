#include <iostream>
using namespace std;

#include "partial_overlap.h"
#include "io.h"

int main (int argc, char *argv[])
{

  if (argc < 6) {
    cout << "Enter the following arguments\n"
      "\t1) file1\n"
      "\t2) filetype1 (ascii / pcdbinary)\n"
      "\t3) file2\n"
      "\t4) filetype2 (ascii / pcdbinary)\n"
      "\t5) threshold (threshold for closest point)\n"
      "\n\n";
    return -1;
  }

  string filename1 = argv[1];
  string filetype1 = argv[2];
  string filename2 = argv[3];
  string filetype2 = argv[4];
  float threshold = atof (argv[5]);

  CloudPtr cloud1 (new Cloud);
  if (filetype1.compare ("ascii") == 0) {
    readASCIIFile (filename1.c_str (), cloud1);
  }
  else if (filetype1.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (filename1.c_str (), cloud1);
  }
  else {
    cout << "\nUnknown file type provided..\n";
    cout << "Options (ascii / pcdbinary)\n\n";
    return -1;
  }


  CloudPtr cloud2 (new Cloud);
  if (filetype2.compare ("ascii") == 0) {
    readASCIIFile (filename2.c_str (), cloud2);
  }
  else if (filetype2.compare ("pcdbinary") == 0) {
    readPCDBinaryFile (filename2.c_str (), cloud2);
  }
  else {
    cout << "\nUnknown file type provided..\n";
    cout << "Options (ascii / pcdbinary)\n\n";
    return -1;
  }

  PartialOverlap po;
  po.setCloud1 (cloud1);
  po.setCloud2 (cloud2);
  po.setThreshold (threshold);

  cout << "Overlap ratio = " << po.findOverlapRatio () << "\n\n";

  return 0;
}
