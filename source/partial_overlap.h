#pragma once

#include "typedefs.h"

using namespace E4PCS;

namespace E4PCS {

class PartialOverlap
{
  protected:

    CloudPtr cloud1;
    CloudPtr cloud2;

    float overlap_ratio;
    float threshold;

  public:

    void setCloud1 (CloudPtr c) { cloud1 = c; }

    void setCloud2 (CloudPtr c) { cloud2 = c; }

    void setThreshold (float t) { threshold = t; }

    float findOverlapRatio ();
};

}
