#pragma once

#include "typedefs.h"
#include <boost/shared_ptr.hpp>

#include <iostream>
using namespace std;

namespace E4PCS
{

struct KeypointParams
{
  string keypoint_type;
};

struct ISSKeypointParams : public KeypointParams
{
  float model_resolution;
  float ratio_21;
  float ratio_32;
  int min_neighbours;
};

struct CurvatureKeypointParams : public KeypointParams
{
  int k_search;
  int num_points;
};

struct SIFTKeypointParams : public KeypointParams
{
  float min_scale;
  int n_octaves;
  int n_scales_per_octave;
  int k_search;
  float min_contrast;
};

typedef boost::shared_ptr <KeypointParams> KeypointParamsPtr;

class KeyPointsInterface
{
  protected:

    CloudPtr cloud;
    CloudPtr keypoints;

    string keypoint_type;

    KeypointParamsPtr keypoint_par;

  public:

    KeyPointsInterface ()
    {
      keypoints.reset (new Cloud);
    }

    KeyPointsInterface (string type)
    {
      keypoint_type = type;
      keypoints.reset (new Cloud);
    }

    void setKeyPointType (string type)
    {
      keypoint_type = type;
    }

    void setParams (KeypointParamsPtr& params) { keypoint_par = params; }

    ~KeyPointsInterface ()
    {
    }

    void compute ();

    void setInputCloud (CloudPtr input) { cloud = input; }

    CloudPtr getKeypoints () { return keypoints; }

  private:

    void computeSIFT ();
    void computeHarris ();
    void computeCurvatureKeypoint ();
    void computeISS ();


    struct CurvatureSortFunctor
    {
      bool operator () (const PointNormal& p1, const PointNormal& p2)
      {
        return p1.curvature > p2.curvature;
        //return p1.curvature < p2.curvature;
      }
    };
};

}
