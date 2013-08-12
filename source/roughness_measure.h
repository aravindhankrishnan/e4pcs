#pragma once

#include "typedefs.h"

using namespace E4PCS;

class RoughnessMeasure
{
  protected:

    CloudPtr cloud;

  public:

    virtual void computeRoughness () = 0;

    void setInputCloud (CloudPtr c) { cloud = c; }

};

typedef boost::shared_ptr <RoughnessMeasure> RoughnessMeasurePtr;

class RoughnessMeasureRMS : public RoughnessMeasure
{
  public:

    RoughnessMeasureRMS ()
    {
      area = 0.0;
      resolution = 1.0;
      random_tries = 500;
      plane_fit_threshold = 2.0;
    }

    void computeRoughness ();

    void setPlaneFitThreshold (float f) { plane_fit_threshold = f; }

    CloudPtr getProjectedCloud () { return projected_cloud; }

  protected:

    float area;
    float plane_fit_threshold;
    int random_tries;
    CloudPtr projected_cloud; 
    float resolution;


    void findMaxPlane (Eigen::Vector4f& plane_eq);

    void findPlaneEquation (CloudPtr cloud, int a, int b, int c, 
                                             Eigen::Vector4f& eq);

    int countPointsOnPlane (CloudPtr cloud, 
                            Eigen::Vector4f& plane_eq);

    void findCentroidOfPlanePts (Eigen::Vector4f& plane_eq, CloudPtr cloud,
                                                  Eigen::Vector3f& p);

  private:


    void project3dto2d (CloudPtr cloud, CloudPtr projected_cloud,
                        Eigen::Vector4f& plane_eq); 

    CloudPtr removeAffineComponents (CloudPtr projected_cloud);

    float findArea (CloudPtr tmp_cloud);

    void findMinMax (CloudPtr cloud, float& minx, float& miny, 
                                      float& maxx, float& maxy);
};


// Histogram of perpendicular distances from the MAX plane
class RoughnessMeasureHoPD : public RoughnessMeasureRMS
{
  public:

    void computeRoughness ();
};

// Entropy of curvatures
class RoughnessMeasureEoC: public RoughnessMeasure
{
  public:

    void computeRoughness ();
};
