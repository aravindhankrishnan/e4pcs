#pragma once

#include "typedefs.h"

#include <vector>
using namespace std;

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace pcl::visualization;


namespace E4PCS {

typedef pcl::KdTreeFLANN <Point> KdTree;

struct Quad
{
  int q[4];
  float r1;
  float r2;
  int quadId;
  Eigen::Vector3f intersection;
  float intersect_angle;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointList
{
  float len;
  vector <int> points;
};


struct QuadMatch
{
  Quad* quad;

  float ab_len;
  float cd_len;

  // (r1_pts[i] & r1_pts[i+1)] are pairs that are of similar
  // length to the pair (quad->q[0] & quad->q[1])
  vector <int>* r1_pts;

  // (r2_pts[i] & r2_pts[i+1)] are pairs that are of similar
  // length to the pair (quad->q[2] & quad->q[3])
  vector <int>* r2_pts;

  // Kdtree object
  KdTree kdtree;

  // List of matching quads
  vector <Quad> matches;

  QuadMatch ()
  {
    best_match = -1;
    ignoreMatch = false;
  }

  int best_match;   // RMS error is computed for transformation between quad and matches[i]
                    // Set to -1 when every RMS error is above a particular threshold

  bool ignoreMatch; // Set when the quad is not on the dominant plane

  double least_error;
  int cnt_best;
};

struct Params
{
  int random_tries;
  float min_dist;
  float length_similarity_threshold;
  float max_range;
  float e_match_threshold;
  float plane_fit_threshold;
  float angle_threshold;
};


class Extended4PCS
{
  protected:


  public:

    Extended4PCS (float D)
    {
      param.random_tries = 1000; // for selecting the plane containing max points

      // this parameter is constant
      param.angle_threshold = 2.0; // angle threshold at quad intersections

      // change this depending on the span of your dataset
      param.min_dist = 2; // ensures min dist between |ab|, |ac|, |ad|, |bc|, |bd|, |cd| so that a, b, c & d are not too close

      // change this depending on the span of your dataset
      param.max_range = 5.0; // maximum threshold for correspondence, used in estimateError ()

      // this depends on the standard deviation of error
      param.length_similarity_threshold = D; // segment length difference while finding matching quads

      // this depends on the standard deviation of error
      param.e_match_threshold = D/2; // matching intersection threshold i.e (e1 - e2)

      // this depends on the standard deviation of error
      param.plane_fit_threshold = D*2; // threshold to find points on the plane


      vp1 = 1;
      vp2 = 2;
    }

    ~Extended4PCS ()
    {
    }

    void setSource (CloudPtr src) { source = src; }

    void setTarget (CloudPtr tgt) { target = tgt; }

    void setVisualizer (PCLVisualizer* v) { viz = v; }

    void setNumQuads (int n) { num_quads = n; }

    vector <int>& getPlanePoints () { return plane_pts; }

    vector <Quad>& getQuads () { return quads; }

    double getRMS () const { return rms; }

    void align ();

    void getTransformation (Eigen::Matrix4f& mat) { mat = transform; }


  private:

    void select3Points (CloudPtr cloud, int& a, int& b, int& c);

    void selectPlane ();

    void findPointsOnPlane (CloudPtr cloud, int a, int b,
                        int c, vector <int>& pts);

    void selectQuads (vector <int>& plane_pts, int N);

    void selectOneQuad (Quad& quad, vector <int>& plane_pts);

    bool checkQuad (Quad& quad);

    void initializeQuadMatchTable ();

    void insertToQuadMatchTable (float length, int i, int j);

    void findSimilarQuads ();

    int findMatchingPoint (KdTree& kdtree, Point pt);

    void plotMatchingQuads (QuadMatch& qmatch, double r,
                            double g, double b);

    void displayPointCloud (CloudPtr cloud, int* color, char* name,
                            int& viewport);

    void findIntersection (CloudPtr cloud, int x1, int x2, 
                           int x3, int x4, Point& intersection);

    bool angleCheck (Quad& quad, Quad& mquad);

    void findBestQuadMatch (QuadMatch& qmatch);

    void estimateRotation (CloudPtr src, CloudPtr tgt,
                                            Eigen::Matrix3f& trans_mat);

    void estimateTranslation (CloudPtr cloud1, CloudPtr cloud2,
                                    Eigen::Matrix3f R, Eigen::Vector3f& T);


    void estimateRigidBodyTransformation (CloudPtr src, CloudPtr tgt,
                                            Eigen::Matrix3f& R,
                                            Eigen::Vector3f& T);

    double estimateError (CloudPtr src, CloudPtr tgt, 
                          Eigen::Matrix3f& R, Eigen::Vector3f& T);

    void demeanPointCloud (CloudPtr &in, Eigen::Vector4f& centroid,
                                 Eigen::MatrixXf &out);

    bool filterMatchingQuads ();

    void getRotationFromCorrelation (Eigen::MatrixXf &cloud_src_demean,
                                 Eigen::Vector4f &centroid_src,
                                 Eigen::MatrixXf &cloud_tgt_demean,
                                 Eigen::Vector4f &centroid_tgt,
                                 Eigen::Matrix3f &transformation_matrix);

    bool samePlaneCheck (Quad& one, Quad& two);

    void findTransformationParameters ();

    void cleanup ();

    int num_quads;

    int vp1;
    int vp2;

    CloudPtr source;
    CloudPtr target;

    vector <int> plane_pts;
    vector <Quad> quads;

    vector <QuadMatch> quadMatchTable;
    vector <PointList> pointListTable;

    Params param;

    double rms;

    Eigen::Matrix4f transform;

    PCLVisualizer* viz;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


}
