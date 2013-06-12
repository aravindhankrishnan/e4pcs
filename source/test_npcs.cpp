#include <iostream>
#include <sstream>
using namespace std;

#include "io.h"
#include "e4pcs.h"

int main (int argc, char *argv[])
{
  if (argc < 2) {
    cout << "Enter arguments\n"
             "1) source cloud\n"
             "2) target cloud\n";
  }

  CloudPtr cloud1 (new Cloud);
  CloudPtr cloud2 (new Cloud);


  readASCIIFile (argv[1], cloud1);
  readASCIIFile (argv[2], cloud2);


  Extended4PCS npcs;
  npcs.setSource (cloud1);
  npcs.setTarget (cloud2);

  npcs.align ();

  return 0;
}
