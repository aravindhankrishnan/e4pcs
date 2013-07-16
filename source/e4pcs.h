#pragma once

#include "typedefs.h"
#include "keypoints_interface.h"

#include <vector>
using namespace std;

#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;


namespace E4PCS {


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

struct E4PCSParams
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

    int num_quads;

    int median_count;

    int best_quad;

    int vp1;
    int vp2;

    CloudPtr source;
    CloudPtr target;

    CloudPtr sourcefull;
    CloudPtr targetfull;

    vector <int> plane_pts;
    vector <Quad> quads;

    vector <QuadMatch> quadMatchTable;
    vector <PointList> pointListTable;

    E4PCSParams param;

    string sampling_type;

    double rms;

    Eigen::Matrix4f transform;

    PCLVisualizer* pviz;
    vector <PCLVisualizer*> V;

    float random_sampling_ratio1;
    float random_sampling_ratio2;
    float region_around_radius;
    float windowsize;


    KeypointParamsPtr keypoint_par;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int argc_;
    char **argv_;


  public:

    Extended4PCS (float D, float min_dist, float corr_max_range)
    {
      float offset = 1;

      if (fabs (D) < 1e-2) {
        D += offset;
      }

      param.random_tries = 500; // for selecting the plane containing max points

      // this parameter is constant
      param.angle_threshold = 5.0; // angle threshold at quad intersections

      // change this depending on the span of your dataset
      param.min_dist = min_dist; // ensures min dist between |ab|, |ac|, |ad|, |bc|, |bd|, |cd| so that a, b, c & d are not too close

      // change this depending on the span of your dataset
      param.max_range = corr_max_range; // maximum threshold for correspondence, used in estimateError ()

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

    void setArgs (int argc, char **argv) { argc_ = argc; argv_ = argv; }

    void setSource (CloudPtr src) { sourcefull = src; }

    void setTarget (CloudPtr tgt) { targetfull = tgt; }

    void setVisualizer (PCLVisualizer* v) { pviz = v; }

    void setNumQuads (int n) { num_quads = n; }

    void setSamplingType (string& type) { sampling_type = type; }

    void setKeypointParams (KeypointParamsPtr& par) { keypoint_par = par; }

    void setSamplingRatio1 (float ratio) { random_sampling_ratio1 = ratio; }

    void setSamplingRatio2 (float ratio) { random_sampling_ratio2 = ratio; }

    void setWindowSize (float size) { windowsize = size; }

    void setRadiusOfRegionAroundKeypoint (float rad) { region_around_radius = rad; }

    vector <int>& getPlanePoints () { return plane_pts; }

    vector <Quad>& getQuads () { return quads; }

    double getRMS () const { return rms; }

    void align ();

    void getTransformation (Eigen::Matrix4f& mat) { mat = transform; }

    vector <PCLVisualizer*>& getVisualizers () { return V; }



  private:

    void select3Points (CloudPtr cloud, int& a, int& b, int& c);

    void samplingRandom ();

    void samplingRandomOnWindows ();

    void samplingKeypoints ();

    void samplingKeypointsAndRegionsAround ();

    void samplingKeypointsAndRandom ();

    void partitionCloud (CloudPtr& cloud, 
                          vector <vector <CloudPtr> >& partitions,
                          float windowsize, int& nx, int& ny);

    void selectMaxPlane ();

    void findPointsOnPlane (CloudPtr cloud, int a, int b,
                        int c, vector <int>& pts);

    void computeKeypoints ( CloudPtr cloud,
                            CloudPtr& keypoints,
                            string type);

    void addPointsWithinRadius (CloudPtr cloud1, CloudPtr cloud2,
                                          float radius);

    void selectQuads (vector <int>& plane_pts, int N);

    void selectOneQuad (Quad& quad, vector <int>& plane_pts);

    bool checkQuad (Quad& quad);

    void initializeQuadMatchTable ();

    void insertToQuadMatchTable (float length, int i, int j);

    void findSimilarQuads ();

    int findMatchingPoint (KdTree& kdtree, Point pt);

    void plotMatchingQuads (PCLVisualizer* viz, QuadMatch& qmatch, double r,
                            double g, double b, int vp1, int vp2);

    void displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, char* name,
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

    void sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud);

    void findMedianCount ();

    void cleanup ();

    void debugQuadMatch ();


};


}
