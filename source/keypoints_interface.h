#pragma once

#include "typedefs.h"

namespace E4PCS
{

class KeyPointsInterface
{
  protected:

    CloudPtr cloud;
    CloudPtr keypoints;

    string keypointType;


  public:

    KeyPointsInterface (string type)
    {
      keypointType = type;
      keypoints.reset (new Cloud);
    }

    ~KeyPointsInterface ()
    {
    }

    void setInputCloud (CloudPtr input) { cloud = input; }
    CloudPtr getKeypoints () { return keypoints; }

  private:
};

}
