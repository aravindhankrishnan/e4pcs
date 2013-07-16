#pragma once

#include "keypoints_interface.h"

#include <fstream>
using namespace std;

#include <boost/shared_ptr.hpp>

namespace E4PCS
{

struct InputParams
{
  float D;
  float sphere_radius;
  float abcd_mindist;
  float corr_max_range;
  string sourcefile;
  string targetfile;
  string filetype1;
  string filetype2;
  string sampling_type;
  string keypoint_type;
  KeypointParamsPtr keypoint_par;
  float region_around_radius;
  float random_sampling_ratio1;
  float random_sampling_ratio2;
  float windowsize;
  int num_quads;
  int vis_num_points;
};

typedef boost::shared_ptr <InputParams> InputParamsPtr;

void getSIFTParameters (istringstream& istr);
void getCurvatureParameters (istringstream& istr);
void getISSParameters (istringstream& istr);
int loadConfigFile (const char* filename);
InputParamsPtr getInputParams ();

}
