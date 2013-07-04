#pragma once

#include "typedefs.h"

namespace E4PCS {

class AffineInvariantRepresentation
{
  protected:

    CloudPtr cloud1;
    CloudPtr cloud2;
    CloudPtr aircloud1;
    CloudPtr aircloud2;

    Eigen::Vector3f eigenvector1;
    Eigen::Vector3f eigenvector2;
    Eigen::Vector3f eigenvector3;


    Eigen::Vector3f centroid1;
    Eigen::Vector3f centroid2;

  public:

    AffineInvariantRepresentation ()
    {
      aircloud1.reset (new Cloud);
      aircloud2.reset (new Cloud);
    }

    ~AffineInvariantRepresentation ()
    {
    }

    void setCloud1 (CloudPtr cloud) { cloud1 = cloud; }
    void setCloud2 (CloudPtr cloud) { cloud2 = cloud; }

    void compute ();

    CloudPtr getAIRCloud1 () { return aircloud1; }
    CloudPtr getAIRCloud2 () { return aircloud2; }

  private:

    void findPrincipalComponents ();
};

typedef AffineInvariantRepresentation AffInvRep;

}
