#include "e4pcs.h"
#include "keypoints_interface.h"
#include <utils/timer.h>

#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/uniform_sampling.h>

#include <boost/foreach.hpp>

#include <iostream>
#include <functional>
#include <algorithm>
#include <sstream>
#include <limits>
#include <iterator>

using namespace std;

#include "io.h"

#define foreach BOOST_FOREACH

namespace E4PCS {

  void Extended4PCS::align ()
  {
    source.reset (new Cloud);
    target.reset (new Cloud);

    if (sampling_type.compare ("random") == 0) {
      samplingRandom ();
    }
    else if (sampling_type.compare ("randomonwindows") == 0) {
      samplingRandomOnWindows ();
    }
    else if (sampling_type.compare ("keypoints") == 0) {
      samplingKeypoints ();
    }
    else if (sampling_type.compare ("keypointsandregionsaround") == 0) {
      samplingKeypointsAndRegionsAround ();
    }
    else if (sampling_type.compare ("keypointsandrandom") == 0) {
      samplingKeypointsAndRandom ();
    }

    if (source->points.size () == 0) {
      throw std::runtime_error ("Zero points in source after sampling ..");
    }
    if (target->points.size () == 0) {
      throw std::runtime_error ("Zero points in target after sampling ..");
    }

    int C[3] = {255, 255, 255};
    displayPointCloud (pviz, source, C, (char *)"cloud7473", vp1);
    displayPointCloud (pviz, target, C, (char *)"cloud7474", vp2);

    selectMaxPlane ();

    //return ;


    if (congruency.compare ("quad") == 0) {
      alignUsingQuads ();
    }
    else if (congruency.compare ("pyramid") == 0) {
      alignUsingPyramids ();
    }
  }

  void Extended4PCS::alignUsingQuads ()
  {
    int N = num_quads;
    selectQuads (plane_pts, N);

    cout << "\n\nSelected " << quads.size () << " quads in source cloud ..\n";

    initializeQuadMatchTable ();

    cout << "\n\nFinding matching point pairs ..\n";
    utils::Timer timer ("seconds");
    timer.tic ();

    for (int i = 0; i < target->points.size (); i++) {
      for (int j = i+1; j < target->points.size (); j++) {

        float length = (target->points[i].getVector3fMap () - 
            target->points[j].getVector3fMap ()).norm ();

        insertToQuadMatchTable (length, i, j);
      }
    }

    cout << "Time taken = " << timer.toc () << " seconds\n\n\n";

    cout << "# entries in Point List table = " << pointListTable.size () << "\n\n";

    cout << "\n\n";


    if (quadMatchTable.size () == 0) {
      cout << "No quads found with the given parameters .. Stopping ..\n";
      return;
    }

    cout << "<< QUAD TABLE >> \n-------------------------------------------------------\n";
    for (int i = 0; i < quadMatchTable.size (); i++) {
      QuadMatch& qmatch = quadMatchTable[i];
      cout << "Quad " << i+1 << " AB length = " << qmatch.ab_len << ",\t# point pairs = " << (*(qmatch.r1_pts)).size () / 2 << endl;
      cout << "Quad " << i+1 << " CD length = " << qmatch.cd_len << ",\t# point pairs = " << (*(qmatch.r2_pts)).size () / 2 << endl;
      cout << "-------------------------------------------------------\n";
    }

    findSimilarQuads ();

    bool emptyFlag = true;
    for (int i = 0; i < quadMatchTable.size (); i++) {
      QuadMatch& qmatch = quadMatchTable[i];
      cout << "Quad " << i+1 << " # of matching quads = " << qmatch.matches.size () << endl;
      if (qmatch.matches.size ()) {
        emptyFlag = false;
      }
    }

    if (emptyFlag) {
      cout << "\n\n <<< NO MATCHING QUADS FOUND. RELAX SOME PARAMETERS AND TRY AGAIN >>> ..\n";
      return;
    }

    cout << "\n";

    findMedianCountQuad ();
    cout << "MEDIAN COUNT = " << median_count << "\n\n";

    // FIDNING THE BEST MATCHING QUAD
    for (int i = 0; i < quadMatchTable.size (); i++) {
      QuadMatch& qmatch = quadMatchTable[i];
      findBestQuadMatch (qmatch);
      if (qmatch.best_match == -1) {
        continue;
      }
      cout << "Found best match for Quad " << i+1 << "\t";
      cout << qmatch.best_match << "\tRMS error = " << qmatch.least_error << "\n";
    }

    findTransformationParametersFromQuad ();

    debugQuadMatch ();

    for (int i = 0; i < quadMatchTable.size (); i++) {
      double r = 0., g = 0., b = 0.;
      switch (i%3) {
        case 0: r = 1; break;
        case 1: g = 1; break;
        case 2: b = 1; break;
      }

      // only plot quads on the dominant plane
      if (quadMatchTable[i].ignoreMatch) {
        continue;
      }

      if (i == best_quad) {
        plotMatchingQuads (pviz, quadMatchTable[i], r, g, b, vp1, vp2);
      }
    }
  }

  void Extended4PCS::alignUsingPyramids ()
  {
    cout << "Aligning using pyramids ..\n";
    int N = num_quads;

    selectPyramids (plane_pts, N);
    //selectPyramids1 (N);

    cout << "\n\nSelected " << pyramids.size () << " pyramids in source cloud ..\n";

    initializePyramidMatchTable ();

    cout << "\n\nFinding matching point pairs ..\n";
    utils::Timer timer ("seconds");
    timer.tic ();

    for (int i = 0; i < target->points.size (); i++) {
      for (int j = i+1; j < target->points.size (); j++) {

        float length = (target->points[i].getVector3fMap () - 
            target->points[j].getVector3fMap ()).norm ();

        insertToPyramidMatchTable (length, i, j);
      }
    }

    cout << "Time taken = " << timer.toc () << " seconds\n\n\n";

    cout << "# entries in Point List table = " << pointListTable.size () << "\n\n";

    cout << "\n\n";


    if (pyramidMatchTable.size () == 0) {
      cout << "No pyramids found with the given parameters .. Stopping ..\n";
      return;
    }

    cout << "<< PYRAMID TABLE >> \n-------------------------------------------------------\n";
    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      PyramidMatch& pmatch = pyramidMatchTable[i];
      cout << "Quad " << i+1 << " AB length = " << pmatch.ab_len << ",\t# point pairs = " << (*(pmatch.r1_pts)).size () / 2 << endl;
      cout << "Quad " << i+1 << " CD length = " << pmatch.cd_len << ",\t# point pairs = " << (*(pmatch.r2_pts)).size () / 2 << endl;
      cout << "-------------------------------------------------------\n";
    }

    findSimilarPyramids ();

    bool emptyFlag = true;
    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      PyramidMatch& pmatch = pyramidMatchTable[i];
      cout << "Pyramid " << i+1 << " # of matching pyramids = " << pmatch.matches.size () << endl;
      if (pmatch.matches.size ()) {
        emptyFlag = false;
      }
    }

    if (emptyFlag) {
      cout << "\n\n <<< NO MATCHING PYRAMIDS FOUND. RELAX SOME PARAMETERS AND TRY AGAIN >>> ..\n";
      debugPyramidMatch ();
      displaySourceAndTarget ();
      return;
    }

    cout << "\n";

    findMedianCountPyramid ();
    cout << "MEDIAN COUNT = " << median_count << "\n\n";

    // FIND THE BEST MATCHING PYRAMID
    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      PyramidMatch& pmatch = pyramidMatchTable[i];
      findBestPyramidMatch (pmatch);
      if (pmatch.best_match == -1) {
        continue;
      }
      cout << "Found best match for Pyramid " << i+1 << "\t";
      cout << pmatch.best_match << "\tRMS error = " << pmatch.least_error << "\n";
    }

    findTransformationParametersFromPyramid ();

    debugPyramidMatch ();

    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      double r = 0., g = 0., b = 0.;
      switch (i%3) {
        case 0: r = 1; break;
        case 1: g = 1; break;
        case 2: b = 1; break;
      }

      // only plot quads on the dominant plane
      //if (pyramidMatchTable[i].ignoreMatch) {
      //  continue;
      //}

      if (i == best_pyramid) {
        plotMatchingPyramids (pviz, pyramidMatchTable[i], r, g, b, vp1, vp2);
      }
    }

    displaySourceAndTarget ();
  }

  void Extended4PCS::displaySourceAndTarget ()
  {

    keypointsviz.reset (new PCLVisualizer (argc_, argv_ , "Keypoints display"));

    int color[3] = {255, 0, 0};
    displayPointCloud (keypointsviz.get (), source, color, "keypointssource");

    color[0] = 0, color[1] = 255;
    displayPointCloud (keypointsviz.get (), target, color, "keypointstarget");
  }

  void Extended4PCS::samplingRandom ()
  {
    srand (time (NULL));
    for (int i = 0; i < random_sampling_ratio1 * sourcefull->points.size (); i++) {
      int k = rand () % sourcefull->points.size ();
      source->points.push_back (sourcefull->points[k]);
    }

    srand (time (NULL));
    for (int i = 0; i < random_sampling_ratio2 * targetfull->points.size (); i++) {
      int k = rand () % targetfull->points.size ();
      target->points.push_back (targetfull->points[k]);
    }

    source->width = target->width = 1;

    source->height = source->points.size ();
    target->height = target->points.size ();
  }


  void Extended4PCS::samplingRandomOnWindows ()
  {
    vector <vector <CloudPtr> > sourcepartitions;
    vector <vector <CloudPtr> > targetpartitions;

    int nx (0), ny (0);

    partitionCloud (sourcefull, sourcepartitions, windowsize, nx, ny);

    nx = 0, ny = 0;

    partitionCloud (targetfull, targetpartitions, windowsize, nx, ny);

    float ratio = random_sampling_ratio1;

    cout << "Sampling ratio = " << ratio << endl;


    foreach (vector <CloudPtr>& row, sourcepartitions) {
      foreach (CloudPtr& cloud, row) {
        int N = ratio * cloud->points.size ();
        //cout << "Selecting " << N << " out of " << cloud->points.size () << " points ..\n";
        srand (time (NULL));
        for (int i = 0; i < N; i++) {
          int k = rand () % cloud->points.size ();
          source->points.push_back (cloud->points[k]);
        }
      }
    }

    source->width = 1;
    source->height = source->points.size ();

    foreach (vector <CloudPtr>& row, targetpartitions) {
      foreach (CloudPtr& cloud, row) {
        int N = ratio * cloud->points.size ();
        srand (time (NULL));
        for (int i = 0; i < N; i++) {
          int k = rand () % cloud->points.size ();
          target->points.push_back (cloud->points[k]);
        }
      }
    }

    target->width = 1;
    target->height = target->points.size ();

    cout << "# of points in source = " << source->points.size () << endl;
    cout << "# of points in target = " << target->points.size () << endl;


  }


  void Extended4PCS::samplingKeypoints ()
  {
    computeKeypoints (sourcefull, source, keypoint_par->keypoint_type);
    cout << "Computed keypoints for source .. " << 
      source->points.size () << endl;

    if (source->points.size () == 0) {
      cout << "Not enough keypoints in source, check keypoint parameters ..\n";
      return;
    }

    //writePointCloud (source, "keypoints-source.asc");

    computeKeypoints (targetfull, target, keypoint_par->keypoint_type);
    cout << "Computed keypoints for target .. " << 
      target->points.size () << endl;

    if (target->points.size () == 0) {
      cout << "Not enough keypoints in target, check keypoint parameters ..\n";
      return;
    }

    //writePointCloud (target, "keypoints-target.asc");
  }

  void Extended4PCS::samplingKeypointsAndRegionsAround ()
  {

    computeKeypoints (sourcefull, source, keypoint_par->keypoint_type);
    cout << "Computed keypoints for source .. " << 
      source->points.size () << endl;

    if (source->points.size () == 0) {
      cout << "Not enough keypoints in source, check keypoint parameters ..\n";
      return;
    }

    //addPointsWithinRadius (source, sourcefull, radius);

    computeKeypoints (targetfull, target, keypoint_par->keypoint_type);
    cout << "Computed keypoints for target .. " << 
      target->points.size () << endl;

    if (target->points.size () == 0) {
      cout << "Not enough keypoints in target, check keypoint parameters ..\n";
      return;
    }

    addPointsWithinRadius (target, targetfull, region_around_radius);
  }

  void Extended4PCS::samplingKeypointsAndRandom ()
  {
    computeKeypoints (sourcefull, source, keypoint_par->keypoint_type);
    cout << "Computed keypoints for source .. " << 
      source->points.size () << endl;

    if (source->points.size () == 0) {
      cout << "Not enough keypoints in source, check keypoint parameters ..\n";
      return;
    }

    float ratio = random_sampling_ratio1;

    srand (time (NULL));
    for (int i = 0; i < ratio * sourcefull->points.size (); i++) {
      int k = rand () % sourcefull->points.size ();
      source->points.push_back (sourcefull->points[k]);
    }

    computeKeypoints (targetfull, target, keypoint_par->keypoint_type);
    cout << "Computed keypoints for target .. " << 
      target->points.size () << endl;

    if (target->points.size () == 0) {
      cout << "Not enough keypoints in target, check keypoint parameters ..\n";
      return;
    }


    srand (time (NULL));
    for (int i = 0; i < ratio * targetfull->points.size (); i++) {
      int k = rand () % targetfull->points.size ();
      target->points.push_back (targetfull->points[k]);
    }

    source->width = target->width = 1;

    source->height = source->points.size ();
    target->height = target->points.size ();

  }


  void Extended4PCS::partitionCloud (CloudPtr& cloud, 
      vector <vector <CloudPtr> >& partitions,
      float windowsize, int& nx, int& ny)
  {
    float minx = numeric_limits <float>::max ();
    float miny = numeric_limits <float>::max ();
    float maxx = numeric_limits <float>::min ();
    float maxy = numeric_limits <float>::min ();


    foreach (Point& pt, cloud->points) {

      if (pt.x < minx) {
        minx = pt.x;
      }
      if (pt.y < miny) {
        miny = pt.y;
      }

      if (pt.x > maxx) {
        maxx = pt.x;
      }
      if (pt.y > maxy) {
        maxy = pt.y;
      }
    }

    nx = ceil ((maxx-minx) / windowsize);
    ny = ceil ((maxy-miny) / windowsize);
    cout << "nx = " << nx << " ny = " << ny << endl;

    partitions.reserve (nx);

    for (int i = 0; i < nx; i++) {
      partitions.push_back (vector <CloudPtr> ());
    }

    for (int i = 0; i < nx; i++) {
      vector<CloudPtr> &vec = partitions[i];
      for (int j = 0; j < ny; j++) {
        CloudPtr C (new Cloud);
        vec.push_back (C);
      }
    }

    size_t N = cloud->points.size ();

    for (size_t i = 0; i < N; i++) {
      size_t ix = 0, iy = 0;
      float x = cloud->points[i].x;
      float y = cloud->points[i].y;
      ix = int (floor ( (x-minx) / windowsize ) );
      iy = int (floor ( (y-miny) / windowsize ) );

      CloudPtr &C = partitions[ix][iy];
      C->points.push_back (cloud->points[i]);
    }

    for (int i = 0; i < nx; i++) {
      for (int j = 0; j < ny; j++) {
        partitions[i][j]->height = partitions[i][j]->points.size ();
        //cout << "# points in " << i << " " << j << " is "
        //  << partitions[i][j]->points.size () << endl;

        partitions[i][j]->width = 1;
      }
    }
  }

  void Extended4PCS::findMedianCountPyramid ()
  {
    vector <int> counts;
    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      PyramidMatch& pmatch = pyramidMatchTable[i];
      int count = pmatch.matches.size ();
      if (count != 0) {
        counts.push_back (count);
      }
    }

    sort (counts.begin (), counts.end (), std::less <int> ());
    int mi =  (4./5) * counts.size ();
    median_count = counts[mi];
  }

  void Extended4PCS::findMedianCountQuad ()
  {
    vector <int> counts;
    for (int i = 0; i < quadMatchTable.size (); i++) {
      QuadMatch& qmatch = quadMatchTable[i];
      int count = qmatch.matches.size ();
      if (count != 0) {
        counts.push_back (count);
      }
    }

    sort (counts.begin (), counts.end (), std::less <int> ());
    int mi =  (4./5) * counts.size ();
    median_count = counts[mi];

  }

  void Extended4PCS::addPointsWithinRadius (CloudPtr cloud1, CloudPtr cloud2,
      float radius)
  {
    KdTreePtr tree (new KdTree);
    tree->setInputCloud (cloud2);

    foreach (Point& pt, cloud1->points) {
      vector <int> ids;
      vector <float> dist;

      if (tree->radiusSearch (pt, radius, ids, dist) > 0) {
        foreach (int& i, ids) {
          cloud1->points.push_back (cloud2->points[i]);
        }
      }
    }
    cloud1->width = 1;
    cloud1->height = cloud1->points.size ();
  }

  void Extended4PCS::computeKeypoints (CloudPtr cloud,
      CloudPtr& keypoints,
      string type)
  {
    KeyPointsInterface kpi (type);
    kpi.setParams (keypoint_par);
    kpi.setInputCloud (cloud);
    kpi.compute ();
    keypoints = kpi.getKeypoints ();
    keypoints->width = 1;
    keypoints->height = keypoints->points.size ();
    cout << "# of keypoints = " << keypoints->points.size () << endl;
  }

  void Extended4PCS::debugPyramidMatch ()
  {
    int validMatches = 0;

    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      if (!pyramidMatchTable[i].ignoreMatch) {
        validMatches++;
      }
    }

    cout << "DEBUG :: # of valid matches = " << validMatches << endl;

    int pyramidsPerVis = 1;
    int n_vis = floor (validMatches / pyramidsPerVis);
    if (validMatches % pyramidsPerVis) {
      n_vis++;
    }

    cout << "DEBUG :: # of visualizers = " << n_vis << endl;

    vector <int> vp (pyramidsPerVis * 2);
    for (int i = 1; i <= pyramidsPerVis * 2; i++) {
      vp.push_back (i);
    }

    int color[3] = {255, 255, 255};

    static int cloud_id = 436488;
    for (int i = 0; i < n_vis; i++) {
      ostringstream ostr;
      ostr << "Matching pyramids set " << i+1;
      PCLVisualizer *v = new PCLVisualizer (argc_, argv_ , ostr.str ().c_str ());
      char *name = NULL;

      float steplen = 1. / pyramidsPerVis;
      for (int k = 0; k < pyramidsPerVis; k++) {
        v->createViewPort (0.0, k*steplen, 1./2, (k+1)*steplen, vp[2*k]);
        ostr.str ("");
        ostr << "cloud" << cloud_id++;
        name = (char *)ostr.str ().c_str ();
        displayPointCloud (v, source, color, name, vp[2*k]);

        v->createViewPort (1./2, k*steplen, 1.0, (k+1)*steplen, vp[2*k+1]);
        ostr.str ("");
        ostr << "cloud" << cloud_id++;
        name = (char *)ostr.str ().c_str ();
        displayPointCloud (v, target, color, name, vp[2*k+1]);

        //v->createViewPort (0.0, 1./3, 1./2, 2./3, vp[2]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, source, color, name, vp[2]);

        //v->createViewPort (1./2, 1./3, 1., 2./3, vp[3]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, target, color, name, vp[3]);

        //v->createViewPort (0.0, 2./3, 1./2, 1.0, vp[4]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, source, color, name, vp[4]);

        //v->createViewPort (1./2, 2./3, 1.0, 1.0, vp[5]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, target, color, name, vp[5]);

      }

      v->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);
      V.push_back (v);
    }

    logfile.open ("pyramid-debug.txt");
    int z = 0;
    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      double r = 0., g = 0., b = 0.;
      switch (i%3) {
        case 0: r = 1; break;
        case 1: g = 1; break;
        case 2: b = 1; break;
      }

      //if (pyramidMatchTable[i].ignoreMatch) {
      //  continue;
      //}

      int k = (int) (z / pyramidsPerVis);

      //cout << "z = " << z << " :: Choosing visualizer " << k << endl;

      int q = z % pyramidsPerVis;
      plotMatchingPyramids (V[k], pyramidMatchTable[i], r, g, b, vp[2*q], vp[2*q+1]);
      z++;
    }
  }

  void Extended4PCS::debugQuadMatch ()
  {

    int validMatches = 0;

    for (int i = 0; i < quadMatchTable.size (); i++) {
      if (!quadMatchTable[i].ignoreMatch) {
        validMatches++;
      }
    }

    cout << "DEBUG :: # of valid matches = " << validMatches << endl;

    int quadsPerVis = 1;
    int n_vis = floor (validMatches / quadsPerVis);
    if (validMatches % quadsPerVis) {
      n_vis++;
    }

    cout << "DEBUG :: # of visualizers = " << n_vis << endl;

    vector <int> vp (quadsPerVis * 2);
    for (int i = 1; i <= quadsPerVis * 2; i++) {
      vp.push_back (i);
    }

    int color[3] = {255, 255, 255};

    static int cloud_id = 36488;
    for (int i = 0; i < n_vis; i++) {
      ostringstream ostr;
      ostr << "Matching quads set " << i+1;
      PCLVisualizer *v = new PCLVisualizer (argc_, argv_ , ostr.str ().c_str ());
      char *name = NULL;

      float steplen = 1. / quadsPerVis;
      for (int k = 0; k < quadsPerVis; k++) {
        v->createViewPort (0.0, k*steplen, 1./2, (k+1)*steplen, vp[2*k]);
        ostr.str ("");
        ostr << "cloud" << cloud_id++;
        name = (char *)ostr.str ().c_str ();
        displayPointCloud (v, source, color, name, vp[2*k]);

        v->createViewPort (1./2, k*steplen, 1.0, (k+1)*steplen, vp[2*k+1]);
        ostr.str ("");
        ostr << "cloud" << cloud_id++;
        name = (char *)ostr.str ().c_str ();
        displayPointCloud (v, target, color, name, vp[2*k+1]);

        //v->createViewPort (0.0, 1./3, 1./2, 2./3, vp[2]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, source, color, name, vp[2]);

        //v->createViewPort (1./2, 1./3, 1., 2./3, vp[3]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, target, color, name, vp[3]);

        //v->createViewPort (0.0, 2./3, 1./2, 1.0, vp[4]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, source, color, name, vp[4]);

        //v->createViewPort (1./2, 2./3, 1.0, 1.0, vp[5]);
        //ostr.str ("");
        //ostr << "cloud" << cloud_id++;
        //name = (char *)ostr.str ().c_str ();
        //displayPointCloud (v, target, color, name, vp[5]);

      }

      v->setBackgroundColor (113.0/255, 113.0/255, 154.0/255);
      V.push_back (v);
    }

    int z = 0;
    for (int i = 0; i < quadMatchTable.size (); i++) {
      double r = 0., g = 0., b = 0.;
      switch (i%3) {
        case 0: r = 1; break;
        case 1: g = 1; break;
        case 2: b = 1; break;
      }

      if (quadMatchTable[i].ignoreMatch) {
        continue;
      }

      int k = (int) (z / quadsPerVis);

      cout << "z = " << z << " :: Choosing visualizer " << k << endl;

      int q = z % quadsPerVis;
      cout << "Plotting matching quad " << i << endl;
      plotMatchingQuads (V[k], quadMatchTable[i], r, g, b, vp[2*q], vp[2*q+1]);
      z++;
    }

    //foreach (PCLVisualizer* v, V) {
    //  v->spin ();
    //}
  }

  void Extended4PCS::cleanup ()
  {
    plane_pts.clear ();
    quads.clear ();
    pointListTable.clear ();

    foreach (QuadMatch& qm, quadMatchTable) {
      qm.quad = NULL;
    }

    quadMatchTable.clear ();
  }

  void Extended4PCS::findTransformationParametersFromPyramid ()
  {
    double best_error = numeric_limits <double>::max ();
    int index = -1;

    for (int i = 0; i < pyramidMatchTable.size (); i++) {
      PyramidMatch& pmatch = pyramidMatchTable[i];

      if (pmatch.ignoreMatch or pmatch.matches.size () == 0
          or pmatch.best_match == -1) {
        continue;
      }

      if (pmatch.least_error < best_error) {
        index = i;
        best_error = pmatch.least_error;
      }
    }

    if (index == -1) {
      cout << "Couldn't find matching quads .. Stopping ..\n";
      transform.setZero ();
      transform.block <3, 3> (0, 0) = Eigen::Matrix3f::Identity ();
      return;
    }

    best_pyramid = index;

    cout << "\nChoosing pyramid " << index+1 << " for finding the "
      "final transformation\n\n";

    PyramidMatch& pmatch = pyramidMatchTable[best_pyramid];
    CloudPtr cloud1 (new Cloud);
    CloudPtr cloud2 (new Cloud);

    Pyramid& p1 = *(pmatch.pyramid);
    Pyramid& p2 = pmatch.matches[pmatch.best_match];

    foreach (int& index, p1.base.q) {
      cloud1->points.push_back (source->points[index]);
    }
    //cloud1->points.push_back (source->points[p1.apex]);
    cloud1->width = 1;
    cloud1->height = cloud1->points.size ();

    foreach (int& index, p2.base.q) {
      cloud2->points.push_back (target->points[index]);
    }
    //cloud2->points.push_back (target->points[p2.apex]);
    cloud2->width = 1;
    cloud2->height = cloud2->points.size ();

    Eigen::Matrix3f R;
    Eigen::Vector3f T;

    estimateRigidBodyTransformation (cloud1, cloud2, R, T);

    transform.block <3, 3> (0, 0) = R;
    transform.block <3, 1> (0, 3) = T;

    transform (3, 3) = 1.0;

    cout << "\n---------- TRANSFORMATION ---------------\n"
      << transform << "\n------------------------------------------\n\n";

  }

  void Extended4PCS::findTransformationParametersFromQuad ()
  {
    double best_error = numeric_limits <double>::max ();
    int index = -1;

    for (int i = 0; i < quadMatchTable.size (); i++) {
      QuadMatch& qmatch = quadMatchTable[i];

      if (qmatch.ignoreMatch or qmatch.matches.size () == 0
          or qmatch.best_match == -1) {
        continue;
      }

      if (qmatch.least_error < best_error) {
        index = i;
        best_error = qmatch.least_error;
      }
    }

    if (index == -1) {
      cout << "Couldn't find matching quads .. Stopping ..\n";
      transform.setZero ();
      transform.block <3, 3> (0, 0) = Eigen::Matrix3f::Identity ();
      return;
    }

    best_quad = index;

    cout << "\nChoosing quad " << index+1 << " for finding the "
      "final transformation\n\n";

    QuadMatch& qmatch = quadMatchTable[best_quad];
    CloudPtr cloud1 (new Cloud);
    CloudPtr cloud2 (new Cloud);

    Quad& q1 = *(qmatch.quad);
    Quad& q2 = qmatch.matches[qmatch.best_match];

    foreach (int& index, q1.q) {
      cloud1->points.push_back (source->points[index]);
    }
    cloud1->width = 1;
    cloud1->height = cloud1->points.size ();

    foreach (int& index, q2.q) {
      cloud2->points.push_back (target->points[index]);
    }
    cloud2->width = 1;
    cloud2->height = cloud2->points.size ();

    Eigen::Matrix3f R;
    Eigen::Vector3f T;

    estimateRigidBodyTransformation (cloud1, cloud2, R, T);

    transform.block <3, 3> (0, 0) = R;
    transform.block <3, 1> (0, 3) = T;

    transform (3, 3) = 1.0;

    cout << "\n---------- TRANSFORMATION ---------------\n"
      << transform << "\n------------------------------------------\n\n";

  }

  bool Extended4PCS::filterMatchingQuads ()
  {
    vector <bool> flags (quadMatchTable.size ());
    vector <int> plane_ids (quadMatchTable.size ());

    int plane_id = 1;

    for (int i = 0; i < quadMatchTable.size (); i++) {

      bool plane_assigned = false;

      if (flags[i]) { 
        continue;
      }

      if (quadMatchTable[i].ignoreMatch) {
        //cout << "\n\nSame plane check : Quad " << i+1 << " and Quad " << j+1 << " ";
        //cout << "\n\nSkipping Quad " << i+1 << " due to ignoreMatch\n";
        continue;
      }

      for (int j = i; j < quadMatchTable.size (); j++) {

        if (flags[j]) {
          continue;
        }


        if (quadMatchTable[j].ignoreMatch) {
          //cout << "\n\nSame plane check : Quad " << i+1 << " and Quad " << j+1 << " ";
          //cout << "Skipping Quad " << j+1 << " due to ignoreMatch\n";
          continue;
        }


        //cout << "\n\nSame plane check : Quad " << i+1 << " and Quad " << j+1 << endl;

        if ( samePlaneCheck (quadMatchTable[i].matches[quadMatchTable[i].best_match],
              quadMatchTable[j].matches[quadMatchTable[j].best_match]) )
        {
          flags[i] = true;
          flags[j] = true;
          plane_ids[i] = plane_id;
          plane_ids[j] = plane_id;
          plane_assigned = true;
        }
      }

      plane_id++;

      if (!plane_assigned) {
        plane_ids[i] = plane_id;
      }
    }

    // contains count of quads on different planes
    vector <int> plane_cnt;

    cout << endl;

    for (int i = 1; i < plane_id; i++) {
      int c = count (plane_ids.begin (), plane_ids.end (), i);
      cout << "# of points on plane " << i << " = " << c << endl;
      plane_cnt.push_back (c);
    }

    // count of quads on the dominant plane (plane containing max quads)
    int max_cnt = *(max_element (plane_cnt.begin (), plane_cnt.end ()));


    // in case there are more planes that have same max_count,
    // we maintain a list
    vector <int> dominant_planes;
    for (int i = 0; i < plane_cnt.size (); i++) {
      if (plane_cnt[i] == max_cnt) {
        dominant_planes.push_back (i+1);
      }
    }

    cout << "\n# of dominant planes = " << dominant_planes.size () << "\n\n";

    for (int i = 0; i < dominant_planes.size (); i++) {
      cout << "# of quads in dominant plane " << i+1 << " is " 
        << plane_cnt[dominant_planes[i]-1] << endl;
    }

    int dom_plane_id = dominant_planes[0];


    for (int i = 0; i < plane_ids.size (); i++) {
      if (plane_ids[i] != dom_plane_id) {
        quadMatchTable[i].ignoreMatch  = true;
      }
    }

    if (plane_cnt.size () == 1) {
      return true;
    }
    cout << "plane_cnt.size () = " << plane_cnt.size () << endl;
    vector <int> tmp;
    foreach (int& i, plane_cnt) {
      tmp.push_back (i);
    }

    sort (tmp.begin (), tmp.end (), greater <int> ());

    int max_cnt1 = tmp[0];
    int max_cnt2 = tmp[1];

    cout << " :: max cnt 1 = " << max_cnt1 << ", max_cnt2 = " << max_cnt2;
    cout << " ::  CONDITION " << (max_cnt1 - max_cnt2)  << " >= " << max_cnt1/2 << endl;

    //if ( (max_cnt1 - max_cnt2) <= 2 ) {
    //  return false;
    //}


    for (int i = 0; i < quadMatchTable.size (); i++) {
      QuadMatch& qmatch = quadMatchTable[i];
      if (qmatch.matches.size () != 0)
        qmatch.ignoreMatch = false;
      else 
        qmatch.ignoreMatch = true;
    }

    if ( (max_cnt1 - max_cnt2) >= (max_cnt1/2) ) {
      return true;
    }

    return false;
  }

  bool Extended4PCS::samePlaneCheck (Quad& one, Quad& two)
  {
    Eigen::Vector3f centroid;
    centroid.setZero ();

    foreach (int& index, one.q) {
      centroid += target->points[index].getVector3fMap ();
    }

    foreach (int& index, two.q) {
      centroid += target->points[index].getVector3fMap ();
    }

    centroid /= 8.0; // two quads contain 8 points

    Eigen::Matrix3f covariance;
    covariance.setZero ();

    foreach (int& index, one.q) {
      Eigen::Vector3f V;
      V.setZero ();
      V = (target->points[index].getVector3fMap () - centroid) ;
      covariance += (V * V.transpose ());
    }

    foreach (int& index, two.q) {
      Eigen::Vector3f V;
      V.setZero ();
      V = (target->points[index].getVector3fMap () - centroid) ;
      covariance += (V * V.transpose ());
    }

    Eigen::EigenSolver <Eigen::Matrix3f> es (covariance);
    //cout << "The eigen values are " << es.eigenvalues () << endl;
    //cout << "\n\n\n" << endl;

    Eigen::Vector3d eigenvalues;
    eigenvalues.setZero ();
    pcl::eigen33 (covariance, eigenvalues);
    //cout << "The eigen values are " << eigenvalues.transpose () << endl << endl;

    Eigen::Vector3f eigenvector1;
    eigenvector1.setZero ();
    pcl::computeCorrespondingEigenVector (covariance, eigenvalues [2], eigenvector1);

    Eigen::Vector3f eigenvector2;
    eigenvector2.setZero ();
    pcl::computeCorrespondingEigenVector (covariance, eigenvalues [1], eigenvector2);

    Eigen::Vector3f eigenvector3;
    eigenvector3.setZero ();
    pcl::computeCorrespondingEigenVector (covariance, eigenvalues [0], eigenvector3);

    //cout << "Eigen values = " << eigenvalues[0] << " " << eigenvalues[1] << " " << eigenvalues[2] << endl;

    float A = eigenvector3 (0);
    float B = eigenvector3 (1);
    float C = eigenvector3 (2);
    float D = -(A * centroid (0) + B * centroid (1) + C * centroid (2));



    foreach (int& index, one.q) {
      float x = target->points[index].x;
      float y = target->points[index].y;
      float z = target->points[index].z;

      float residue = fabs (A*x + B*y + C*z + D);
      //cout << "Same plane check , residue = " << residue << endl;
      if ( residue > param.plane_fit_threshold ) {
        //cout << "Residue check failed.. returning FALSE..\n";
        return false;
      }
    }

    foreach (int& index, two.q) {
      float x = target->points[index].x;
      float y = target->points[index].y;
      float z = target->points[index].z;

      float residue = fabs (A*x + B*y + C*z + D);
      //cout << "Same plane check , residue = " << residue << endl;
      if ( residue > param.plane_fit_threshold ) {
        //cout << "Residue check failed.. returning FALSE..\n";
        return false;
      }
    }

    return true;
  }

  void Extended4PCS::getRotationFromCorrelation (Eigen::MatrixXf &cloud_src_demean,
      Eigen::Vector4f &centroid_src,
      Eigen::MatrixXf &cloud_tgt_demean,
      Eigen::Vector4f &centroid_tgt,
      Eigen::Matrix3f &R)
  {
    R.setIdentity ();

    Eigen::Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

    Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f u = svd.matrixU ();
    Eigen::Matrix3f v = svd.matrixV ();

    if (u.determinant () * v.determinant () < 0) {
      for (int x = 0; x < 3; ++x)
        v (x, 2) *= -1;
    }

    R = v * u.transpose ();
  }

  void Extended4PCS::demeanPointCloud (CloudPtr &in, Eigen::Vector4f& centroid,
      Eigen::MatrixXf &out)
  {
    size_t npts = in->points.size ();

    out = Eigen::MatrixXf::Zero (4, npts);        // keep the data aligned

    for (size_t i = 0; i < npts; ++i) {
      out.block<4, 1> (0, i) = in->points[i].getVector4fMap () - centroid;
    }

    out.block (3, 0, 1, npts).setZero ();
  }

  void Extended4PCS::estimateTranslation (CloudPtr cloud1, CloudPtr cloud2,
      Eigen::Matrix3f R, Eigen::Vector3f& T)
  {
    Eigen::Vector4f centroid_src, centroid_tgt;
    compute3DCentroid (*cloud1, centroid_src);
    compute3DCentroid (*cloud2, centroid_tgt);

    Eigen::Vector3f tmp1;
    Eigen::Vector3f tmp2;

    tmp1 (0) = centroid_src (0);
    tmp1 (1) = centroid_src (1);
    tmp1 (2) = centroid_src (2);

    tmp2 (0) = centroid_tgt (0);
    tmp2 (1) = centroid_tgt (1);
    tmp2 (2) = centroid_tgt (2);

    T = tmp2 - (R * tmp1);
  }

  void Extended4PCS::estimateRotation (CloudPtr src, CloudPtr tgt,
      Eigen::Matrix3f& R)
  {

    Eigen::Vector4f centroid_src, centroid_tgt;
    compute3DCentroid (*src, centroid_src);
    compute3DCentroid (*tgt, centroid_tgt);

    Eigen::MatrixXf cloud_src_demean;
    demeanPointCloud (src, centroid_src, cloud_src_demean);

    Eigen::MatrixXf cloud_tgt_demean;
    demeanPointCloud (tgt, centroid_tgt, cloud_tgt_demean);

    getRotationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, R);
  }

  void Extended4PCS::estimateRigidBodyTransformation (CloudPtr src, CloudPtr tgt,
      Eigen::Matrix3f& R,
      Eigen::Vector3f& T)
  {
    R.setIdentity ();
    T.setZero ();

    if (src->points.size () == 0 &&
        tgt->points.size () == 0) {
      return;
    }

    estimateRotation (src, tgt, R);
    estimateTranslation (src, tgt, R, T);
  }

  void Extended4PCS::findBestPyramidMatch (PyramidMatch& pmatch)
  {
    //if (pmatch.matches.size () == 0 or pmatch.matches.size () > median_count ) {
    //  pmatch.ignoreMatch = true;
    //  return;
    //}

    Pyramid& pyramid = *(pmatch.pyramid);
    CloudPtr cloud1 (new Cloud);
    foreach (int& index, pyramid.base.q) {
      cloud1->points.push_back (source->points[index]);
    }
    cloud1->points.push_back (source->points[pyramid.apex]);
    cloud1->width = 1;
    cloud1->height = cloud1->points.size ();

    //cout << "best pyramid points 1 = " << cloud1->points.size () << endl;

    double rms_best = numeric_limits <double>::max ();

    //cout << "findBestQuadMatch Error : ";

    for (int i = 0; i < pmatch.matches.size (); i++) {
      Pyramid& match = pmatch.matches[i];
      CloudPtr cloud2 (new Cloud);
      foreach (int& index, match.base.q) {
        cloud2->points.push_back (target->points[index]);
      }
      cloud2->points.push_back (target->points[match.apex]);
      cloud2->width = 1;
      cloud2->height = cloud2->points.size ();

      //cout << "best pyramid points 2 = " << cloud2->points.size () << endl;

      Eigen::Matrix3f R;
      Eigen::Vector3f T;
      estimateRigidBodyTransformation (cloud1, cloud2, R, T);
      //cout << "best pyramid , estimated rigid body transformation ..\n";

      double rms_cur = 0;

      CloudPtr sampledsource (new Cloud);
      CloudPtr sampledtarget (new Cloud);

      sampleCloud (sourcefull, 300, sampledsource);
      sampleCloud (targetfull, 300, sampledtarget);

      if ( (rms_cur = estimateError (sampledsource, sampledtarget, R, T) ) < rms_best) {
        //if ( (rms_cur = estimateError (source, target, R, T) ) < rms_best) {
        rms_best = rms_cur;
        pmatch.best_match = i;
        pmatch.least_error = rms_best;
      }
      }
      cout << endl;
    }

    void Extended4PCS::findBestQuadMatch (QuadMatch& qmatch)
    {

      if (qmatch.matches.size () == 0 or qmatch.matches.size () > median_count ) {
        qmatch.ignoreMatch = true;
        return;
      }
      Quad& quad = *(qmatch.quad);
      CloudPtr cloud1 (new Cloud);
      foreach (int& index, quad.q) {
        cloud1->points.push_back (source->points[index]);
      }
      cloud1->width = 1;
      cloud1->height = cloud1->points.size ();

      double rms_best = numeric_limits <double>::max ();

      //cout << "findBestQuadMatch Error : ";

      for (int i = 0; i < qmatch.matches.size (); i++) {
        Quad& match = qmatch.matches[i];
        CloudPtr cloud2 (new Cloud);
        foreach (int& index, match.q) {
          cloud2->points.push_back (target->points[index]);
        }
        cloud2->width = 1;
        cloud2->height = cloud2->points.size ();

        Eigen::Matrix3f R;
        Eigen::Vector3f T;
        estimateRigidBodyTransformation (cloud1, cloud2, R, T);

        double rms_cur = 0;

        CloudPtr sampledsource (new Cloud);
        CloudPtr sampledtarget (new Cloud);

        sampleCloud (sourcefull, 300, sampledsource);
        sampleCloud (targetfull, 300, sampledtarget);

        if ( (rms_cur = estimateError (sampledsource, sampledtarget, R, T) ) < rms_best) {
          //if ( (rms_cur = estimateError (source, target, R, T) ) < rms_best) {
          rms_best = rms_cur;
          qmatch.best_match = i;
          qmatch.least_error = rms_best;
        }

        //cout << rms_cur << "\t";

        //int cnt_best  = 0;
        //int cnt_cur = 0;

        //if ( (cnt_cur = estimateError (source, target, R, T) ) > cnt_best ) {
        //  cnt_best = cnt_cur;
        //  qmatch.best_match = i;
        //  qmatch.cnt_best = cnt_best;
        //}

        }
        cout << endl;
      }

      void Extended4PCS::sampleCloud (CloudPtr cloud, int N, CloudPtr sampledcloud)
      {
        if (cloud->points.size () <= N) {
          sampledcloud = cloud;
        }
        else {
          srand (time (NULL));
          for (int i = 0; i < N; i++) {
            int k = rand () % cloud->points.size ();
            sampledcloud->points.push_back (cloud->points[k]);
          }
          sampledcloud->width = 1;
          sampledcloud->height = sampledcloud->points.size ();
        }

        //pcl::UniformSampling<Point> us;
        //us.setInputCloud (cloud);
        //double radius = 30;
        //us.setRadiusSearch (radius);
        //pcl::PointCloud<int> subsampled_indices;
        //us.compute (subsampled_indices);
        //std::sort (subsampled_indices.points.begin (), subsampled_indices.points.end ());
        //pcl::copyPointCloud (*cloud, subsampled_indices.points, *sampledcloud);

        //cout << "Points = " << cloud->points.size () << " "
        //  << sampledcloud->points.size () << endl;
      }

      double Extended4PCS::estimateError (CloudPtr src, CloudPtr tgt, 
          Eigen::Matrix3f& R, Eigen::Vector3f& T)
      {
        KdTree kdtree;		
        kdtree.setInputCloud (tgt);
        int K = 1;
        vector<int> ids (K);
        vector<float> dist (K);

        double error = 0.0;
        int count = 0;

        for(int i = 0; i < src->points.size(); i++) {

          Eigen::Vector3f v = src->points[i].getVector3fMap ();
          v = R * v + T;
          Point p1 (v (0), v (1), v (2));

          if(kdtree.nearestKSearch(p1, K, ids, dist) > 0) {

            Point p2 = tgt->points[ids[0]];

            double dist = (p1.getVector3fMap () - p2.getVector3fMap ()).norm ();
            if(dist > param.max_range) {
              continue;
            }
            count++;
            error += dist;
          }
        }

        error /= count;

        // Adding penalty if there are not enough corresponding points
        if (count < 0.8 * src->points.size ()) {
          error += 9999;
        }

        //cout << "Number of correspondences found = " << count << 
        //  "\tTotal error = " << error << endl;

        return error;
      }

      void Extended4PCS::plotMatchingPyramids (PCLVisualizer* viz, PyramidMatch& pmatch, double red,
          double green, double blue, int vp1, int vp2)
      {

        //if (pmatch.best_match == -1) {
        //  return;
        //}

        ostringstream ostr;

        static int lineId = 39745;
        static int sphereId = 12445;

        Pyramid& pyramid = *(pmatch.pyramid);
        Quad& quad = pyramid.base;
        for (int k = 0; k <4; k++) {
          ostr.str ("");
          ostr << "sphere" << sphereId++;
          viz->addSphere (source->points[quad.q[k]], sphere_radius, 
              red, green, blue, ostr.str ().c_str (), vp1);
        }
        ostr.str ("");
        ostr << "sphere" << sphereId++;
        viz->addSphere (source->points[pyramid.apex], 2*sphere_radius, 
            1, 0, 1, ostr.str ().c_str (), vp1);

        Eigen::Vector3f a = source->points[quad.q[0]].getVector3fMap ();
        Eigen::Vector3f b = source->points[quad.q[1]].getVector3fMap ();
        Eigen::Vector3f c = source->points[quad.q[2]].getVector3fMap ();
        Eigen::Vector3f d = source->points[quad.q[3]].getVector3fMap ();

        static int intersectId = 13726;

        Eigen::Vector3f e = a + quad.r1 * (b - a);
        Point pt (e (0), e (1), e (2));

        Eigen::Vector3f te = c + quad.r2 * (d - c);

        if ( (e-te).norm () > 1e-2) {
          //cout << "(e-te).norm () = " << (e-te).norm () << endl;
          //throw std::runtime_error ("\nIntersection ratio doesn't match ..\n\n");
        }

        ostr.str ("");
        ostr << "intersect" << intersectId++;
        viz->addSphere (pt, 2*sphere_radius, 1, 1, 0, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[0]], source->points[quad.q[1]], 
            1, 1, 0, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[0]], source->points[quad.q[2]], 
            0, 1, 1, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[0]], source->points[quad.q[3]], 
            0, 1, 1, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[1]], source->points[quad.q[2]], 
            0, 1, 1, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[1]], source->points[quad.q[3]], 
            0, 1, 1, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[2]], source->points[quad.q[3]], 
            1, 1, 0, ostr.str ().c_str (), vp1);

        //cout << "APEX - CORNER LENGTHS :: ";
        for (int i = 0; i < 4; i++) {
          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (source->points[quad.q[i]], source->points[pyramid.apex], 
              0, 1, 1, ostr.str ().c_str (), vp1);
          //cout << (source->points[quad.q[i]].getVector3fMap () - 
          //         source->points[pyramid.apex].getVector3fMap ()).norm () << "\t";
          if (pmatch.best_match != -1) {
            logfile << source->points[quad.q[i]].getVector3fMap ().transpose () << " ";
          }
        }
        if (pmatch.best_match != -1) {
          logfile << source->points[pyramid.apex].getVector3fMap ().transpose () << " ";
        }
        //cout << endl;

        //for (int i = 0; i < pmatch.matches.size (); i++) 
        {
          Pyramid& mpyramid = pmatch.matches[pmatch.best_match];

          if (pmatch.best_match == -1) {
            return;
          }

          Quad& mquad = mpyramid.base;
          for (int k = 0; k < 4; k++) {
            ostr.str ("");
            ostr << "sphere" << sphereId++;
            viz->addSphere (target->points[mquad.q[k]], sphere_radius, 
                red, green, blue, ostr.str ().c_str (), vp2);
          }
          ostr.str ("");
          ostr << "sphere" << sphereId++;
          viz->addSphere (target->points[mpyramid.apex], 2*sphere_radius, 
              1, 0, 1, ostr.str ().c_str (), vp2);

          Eigen::Vector3f a = target->points[mquad.q[0]].getVector3fMap ();
          Eigen::Vector3f b = target->points[mquad.q[1]].getVector3fMap ();

          Eigen::Vector3f e = a + mquad.r1 * (b - a);
          Point pt (e (0), e (1), e (2));

          ostr.str ("");
          ostr << "intersect" << intersectId++;
          viz->addSphere (pt, 2*sphere_radius, 1, 1, 0, ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[mquad.q[0]], target->points[mquad.q[1]], 
              1, 1, 0, ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[mquad.q[0]], target->points[mquad.q[2]], 
              0, 1, 1, ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[mquad.q[0]], target->points[mquad.q[3]], 
              0, 1, 1, ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[mquad.q[1]], target->points[mquad.q[2]], 
              0, 1, 1, ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[mquad.q[1]], target->points[mquad.q[3]], 
              0, 1, 1, ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[mquad.q[2]], target->points[mquad.q[3]], 
              1, 1, 0, ostr.str ().c_str (), vp2);

          //cout << "CONGRUENT APEX - CORNER LENGTHS :: ";
          for (int i = 0; i < 4; i++) {
            ostr.str ("");
            ostr << "line" << lineId++;
            viz->addLine (target->points[mquad.q[i]], target->points[mpyramid.apex], 
                0, 1, 1, ostr.str ().c_str (), vp2);
            //cout << (target->points[mquad.q[i]].getVector3fMap () - 
            //         target->points[mpyramid.apex].getVector3fMap ()).norm () << "\t";
            logfile << target->points[mquad.q[i]].getVector3fMap ().transpose () << " ";
          }
          logfile << target->points[mpyramid.apex].getVector3fMap ().transpose () << "\n";
          logfile.flush ();
          if (pyramidEdgeLengthCheck (pmatch, mpyramid)) {
            //cout << "Edge length check passed ..\n";
          }
          else {
            cout << "Edge length check failed ..\n";
          }
          //cout << "\n\n";
        }
      }


      void Extended4PCS::plotMatchingQuads (PCLVisualizer* viz, QuadMatch& qmatch, double red,
          double green, double blue, int vp1, int vp2)
      {

        if (qmatch.best_match == -1) {
          return;
        }

        ostringstream ostr;

        static int lineId = 98745;
        static int sphereId = 23445;

        Quad& quad = *(qmatch.quad);
        for (int k = 0; k <4; k++) {
          ostr << "sphere" << sphereId++;
          viz->addSphere (source->points[quad.q[k]], sphere_radius, 
              red, green, blue, ostr.str ().c_str (), vp1);
        }

        Eigen::Vector3f a = source->points[quad.q[0]].getVector3fMap ();
        Eigen::Vector3f b = source->points[quad.q[1]].getVector3fMap ();
        Eigen::Vector3f c = source->points[quad.q[2]].getVector3fMap ();
        Eigen::Vector3f d = source->points[quad.q[3]].getVector3fMap ();

        static int intersectId = 1;
        Eigen::Vector3f e = a + quad.r1 * (b - a);
        Eigen::Vector3f te = c + quad.r2 * (d - c);

        if ( (e-te).norm () > 1e-2) {
          cout << "(e-te).norm () = " << (e-te).norm () << endl;
          //throw std::runtime_error ("\nIntersection ratio doesn't match ..\n\n");
        }
        Point pt (e (0), e (1), e (2));

        ostr.str ("");
        ostr << "intersect" << intersectId++;
        viz->addSphere (pt, 2*sphere_radius, 1, 1, 0, ostr.str ().c_str (), vp1);

        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[0]], source->points[quad.q[1]], 
            1, 1, 0, ostr.str ().c_str (), vp1);
        ostr.str ("");
        ostr << "line" << lineId++;
        viz->addLine (source->points[quad.q[2]], source->points[quad.q[3]], 
            1, 1, 0, ostr.str ().c_str (), vp1);

        //for (int i = 0; i < qmatch.matches.size (); i++) 
        {
          if (qmatch.best_match == -1) {
            return;
          }
          Quad& quad = qmatch.matches[qmatch.best_match];
          //Quad& quad = qmatch.matches[i];

          for (int k = 0; k <4; k++) {
            ostr << "sphere" << sphereId++;
            Point& pt = target->points[quad.q[k]];
            viz->addSphere (pt, sphere_radius, red, green, blue, 
                ostr.str ().c_str (), vp2);
          }

          ostr.str ("");
          ostr << "line" << lineId++;
          int x1 = quad.q[0], x2 = quad.q[1], x3 = quad.q[2], x4 = quad.q[3];
          viz->addLine (target->points[x1], target->points[x2], 1, 1, 0, 
              ostr.str ().c_str (), vp2);

          ostr.str ("");
          ostr << "line" << lineId++;
          viz->addLine (target->points[x3], target->points[x4], 1, 1, 0,
              ostr.str ().c_str (), vp2);

          Point intersection;
          findIntersection (target, x1, x2, x3, x4, intersection);

          Eigen::Vector3f e = intersection.getVector3fMap ();

          ostr.str ("");
          ostr << "intersect" << intersectId++;
          viz->addSphere (intersection, 2*sphere_radius, 1, 1, 0,
              ostr.str ().c_str (), vp2);
        }
      }

      void Extended4PCS::displayPointCloud (PCLVisualizer* viz, CloudPtr cloud, int* color, 
          char* name, int viewport)
      {
        PointCloudColorHandlerCustom <Point> tgt_h (cloud, color[0], color[1], color[2]);
        viz->addPointCloud (cloud, tgt_h, name, viewport);
      }

      bool Extended4PCS::findCongruentApex (PyramidMatch& pmatch, Pyramid& mpyramid, CloudPtr target) 
      {
        Pyramid& pyramid = *(pmatch.pyramid);
        Eigen::Vector4f  plane_eq1;
        Eigen::Vector4f  plane_eq2;
        Eigen::Vector4f  plane_eq3;

        findPlaneEquation (source, pyramid.base.q[0], pyramid.base.q[2], pyramid.apex, plane_eq1);
        findPlaneEquation (source, pyramid.base.q[1], pyramid.base.q[2], pyramid.apex, plane_eq2);
        findPlaneEquation (source, pyramid.base.q[0], pyramid.base.q[3], pyramid.apex, plane_eq3);

        CloudPtr src (new Cloud);
        CloudPtr tgt (new Cloud);

        for (int i = 0; i < 4; i++) {
          src->points.push_back (source->points[pyramid.base.q[i]]);
        }
        src->width = 1;
        src->height = src->points.size ();

        for (int i = 0; i < 4; i++) {
          tgt->points.push_back (target->points[mpyramid.base.q[i]]);
        }
        tgt->width = 1;
        tgt->height = tgt->points.size ();

        Eigen::Matrix3f R;
        estimateRotation (src, tgt, R);

        adjustPyramid2PlaneEqn (R, plane_eq1, target->points[mpyramid.base.q[0]]);
        adjustPyramid2PlaneEqn (R, plane_eq2, target->points[mpyramid.base.q[1]]);
        adjustPyramid2PlaneEqn (R, plane_eq3, target->points[mpyramid.base.q[0]]);

        Eigen::Vector3f e;
        find3PlaneIntersection (plane_eq1, plane_eq2, plane_eq3, e);

        Point pt (e (0), e (1), e (2));

        KdTreePtr tree (new KdTree);
        tree->setInputCloud (target);

        int K = 1;
        vector <int> ids (K);
        vector <float> dists (K);

        fill (ids.begin (), ids.end (), -1);
        fill (dists.begin (), dists.end (), 0.);

        if (tree->nearestKSearch (pt, K, ids, dists) > 0) {
          if (dists[0] < param.length_similarity_threshold*2) {
            mpyramid.apex = ids[0];
          }
          else {
            return false;
          }
        }
        else {
          return false;
        }

        if ( pyramidEdgeLengthCheck (pmatch, mpyramid) and
            pyramidApexBaseDistanceCheck (pmatch, mpyramid) ) {
          return true;
        }

        return false;
      }

      void Extended4PCS::find3PlaneIntersection (Eigen::Vector4f& plane_eq1,
          Eigen::Vector4f& plane_eq2,
          Eigen::Vector4f& plane_eq3,
          Eigen::Vector3f& e)
      {
        // Refer to Eq 8 in http://mathworld.wolfram.com/Plane-PlaneIntersection.html
        Eigen::Vector3f n1 = plane_eq1.block <3, 1> (0, 0);
        Eigen::Vector3f n2 = plane_eq2.block <3, 1> (0, 0);
        Eigen::Vector3f n3 = plane_eq3.block <3, 1> (0, 0);

        float x1 = -plane_eq1 (3);
        float x2 = -plane_eq2 (3);
        float x3 = -plane_eq3 (3);

        cout << "\n";
        cout << "Normal 1 = " << n1.transpose () << endl;
        cout << "Normal 2 = " << n2.transpose () << endl;
        cout << "Normal 3 = " << n3.transpose () << endl;

        Eigen::Matrix3f mat;
        mat.row (0) = n1;
        mat.row (1) = n2;
        mat.row (2) = n3;

        float K = mat.determinant ();

        cout << "\nDeterminant = " << K << endl;

        e.setZero ();
        e = (1./K) * (x1 * (n2.cross (n3)) +
            x2 * (n3.cross (n1)) +
            x3 * (n1.cross (n2)));
        cout << "\nIntersection is " << e.transpose () << endl;

      }

      void Extended4PCS::adjustPyramid2PlaneEqn (Eigen::Matrix3f& R, 
          Eigen::Vector4f& plane_eq, 
          Point& pt)
      {
        plane_eq.block <3, 1> (0, 0) = R * plane_eq.block <3, 1> (0, 0);
        plane_eq (3) = -pt.getVector3fMap ().dot (plane_eq.block <3, 1> (0, 0));
      }

      void Extended4PCS::findSimilarPyramids ()
      {
        for (int i = 0; i < pyramidMatchTable.size (); i++) {
          cout << "\n[[[ ----  Finding matching pyramid for Pyramid " 
            << i+1 << " --- ]]]\n";
          PyramidMatch& pmatch = pyramidMatchTable[i];

          float radius = (source->points[pmatch.pyramid->apex].getVector3fMap () - 
              pmatch.pyramid->base.intersection).norm ();

          float D = param.length_similarity_threshold*3;

          KdTree& kdtree = pmatch.kdtree;
          Quad& quad  = pmatch.pyramid->base;
          float r1 (quad.r1);
          float r2 (quad.r2);

          vector <int>& r1_pts = *(pmatch.r1_pts);

          //  Creating a point cloud containing r1 intersections
          //  A KdTree is then created for this point cloud
          CloudPtr intersections (new Cloud);

          for (int i = 0; i < r1_pts.size (); i += 2) {
            Eigen::Vector3f a = target->points[r1_pts[i]].getVector3fMap ();
            Eigen::Vector3f b = target->points[r1_pts[i+1]].getVector3fMap ();

            if ( ( (a-b).norm () - pmatch.ab_len)  > param.length_similarity_threshold){
              //cout << "*** Point pairs NOT MATCHING AB length criteria ..\n";
            }
            else {
              //cout << "*** Point pairs MATCHING AB length criteria ..\n";
            }

            Eigen::Vector3f e1_1 = a + r1 * (b-a);
            Point pt1 (e1_1 (0), e1_1 (1), e1_1 (2));
            intersections->points.push_back (pt1);


            Eigen::Vector3f e1_2 = b + r1 * (a-b);
            Point pt2 (e1_2 (0), e1_2 (1), e1_2 (2));
            intersections->points.push_back (pt2);
          }

          intersections->width = 1;
          intersections->height = intersections->points.size ();

          if (intersections->points.size () == 0) {
            cout << "Intersection->points.size () = 0 \n\n";
            continue;
            //return;
          }

          //cout << "# of point pairs matching AB = " << r1_pts.size () << endl;
          kdtree.setInputCloud (intersections);
          //cout << "# of r1 intersections = " << intersections->points.size () << endl;


          vector <int>& r2_pts = *(pmatch.r2_pts);

          for (int i = 0; i < r2_pts.size (); i += 2) {
            Eigen::Vector3f a = target->points[r2_pts[i]].getVector3fMap ();
            Eigen::Vector3f b = target->points[r2_pts[i+1]].getVector3fMap ();

            if ( ( (a-b).norm () - pmatch.cd_len)  > param.length_similarity_threshold) {
              //cout << "*** Point pairs NOT MATCHING CD length criteria ..\n";
            }
            else {
              //cout << "*** Point pairs MATCHING CD length criteria ..\n";
            }

            Eigen::Vector3f e2_1 = a + r2 * (b-a);
            Point pt1 (e2_1 (0), e2_1 (1), e2_1 (2));

            int index = findMatchingPoint (kdtree, pt1); // returns index from cloud created in the above loop
            if (index != -1) {
              if ( (index % 2) == 0) {
                // Matching quad pairs are {r2_pts[i], r2_pts[i+1], r1_pts[index], r1_pts[index+1]}
                // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
                Quad mquad;
                mquad.q[0] = r1_pts[index];
                mquad.q[1] = r1_pts[index+1];
                mquad.q[2] = r2_pts[i];
                mquad.q[3] = r2_pts[i+1];

                if (angleCheck (quad, mquad)) {

                  Pyramid mpyramid;
                  mquad.r1 = r1;
                  mquad.r2 = r2;
                  mpyramid.base = mquad;

                  if (findCongruentApex (pmatch, mpyramid, target)) {
                    pmatch.matches.push_back (mpyramid);
                    cout << "\t<< Pyramid apex check PASSED >> ..\n";
                  }
                  else {
                    cout << "\t<< Pyramid apex check FAILED >> ..\n";
                  }
                }
                else {
                  cout << "\n<<< Base check failed >> ..\n";
                }
              }
              else { // index % 2 == 0
                // Matching quad pairs are {r2_pts[i], r2_pts[i+1], r1_pts[index], r1_pts[index-1]}
                // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
                Quad mquad;
                mquad.q[0] = r1_pts[index];
                mquad.q[1] = r1_pts[index-1];
                mquad.q[2] = r2_pts[i];
                mquad.q[3] = r2_pts[i+1];

                if (angleCheck (quad, mquad)) {
                  Pyramid mpyramid;
                  mquad.r1 = r1;
                  mquad.r2 = r2;
                  mpyramid.base = mquad;

                  if (findCongruentApex (pmatch, mpyramid, target)) {
                    pmatch.matches.push_back (mpyramid);
                    cout << "\t<< Pyramid apex check PASSED >> ..\n";
                  }
                  else {
                    cout << "\t<< Pyramid apex check FAILED >> ..\n";
                  }
                }
                else {
                  cout << "\t<< Base check failed >> ..\n";
                }
              } // end of index % 2 == 0
            } // end of index != -1
            else {
              cout << "\t<< Intersection not found for ratio r1 >> ..\n";
            }

            Eigen::Vector3f e2_2 = b + r2 * (a-b);
            Point pt2 (e2_2 (0), e2_2 (1), e2_2 (2));

            index = findMatchingPoint (kdtree, pt2);
            if (index != -1) {
              if ( (index % 2) == 0) {
                // Matching quad pairs are (r2_pts[i+1], r2_pts[i], r1_pts[index], r1_pts[index+1])
                // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }

                Quad mquad;
                mquad.q[0] = r1_pts[index];
                mquad.q[1] = r1_pts[index+1];
                mquad.q[2] = r2_pts[i+1];
                mquad.q[3] = r2_pts[i];
                //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
                //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
                //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

                if (angleCheck (quad, mquad)) {
                  Pyramid mpyramid;
                  mquad.r1 = r1;
                  mquad.r2 = r2;
                  mpyramid.base = mquad;

                  if (findCongruentApex (pmatch, mpyramid, target)) {
                    pmatch.matches.push_back (mpyramid);
                    cout << "\t<< Pyramid apex check PASSED >> ..\n";
                  }
                  else {
                    cout << "\t<< Pyramid apex check FAILED >> ..\n";
                  }
                }
                else {
                  cout << "\t<< Base check failed >> ..\n";
                }
              }
              else {
                // Matching quad pairs are (r2_pts[i+1], r2_pts[i], r1_pts[index], r1_pts[index-1])
                // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }

                Quad mquad;
                mquad.q[0] = r1_pts[index];
                mquad.q[1] = r1_pts[index-1];
                mquad.q[2] = r2_pts[i+1];
                mquad.q[3] = r2_pts[i];
                //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
                //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
                //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

                if (angleCheck (quad, mquad)) {
                  Pyramid mpyramid;
                  mquad.r1 = r1;
                  mquad.r2 = r2;
                  mpyramid.base = mquad;

                  if (findCongruentApex (pmatch, mpyramid, target)) {
                    pmatch.matches.push_back (mpyramid);
                    cout << "\t<< Pyramid apex check PASSED >> ..\n";
                  }
                  else {
                    cout << "\t<< Pyramid apex check FAILED >> ..\n";
                  }
                }
                else {
                  cout << "\t<< Base check failed >> ..\n";
                }
              }
            }
            else {
              cout << "\t<< Intersection not found for ratio r2 >> ..\n";
            }
          }
        }
      }

          bool Extended4PCS::pyramidApexBaseDistanceCheck (PyramidMatch& pmatch, 
              Pyramid& mpyramid)
          {
            Pyramid& pyramid = *(pmatch.pyramid);
            Eigen::Vector4f plane_eq1;
            plane_eq1.setZero ();
            findPlaneEquation (source, pyramid.base.q[0], pyramid.base.q[1], pyramid.base.q[2], plane_eq1);

            Eigen::Vector4f plane_eq2;
            plane_eq2.setZero ();
            findPlaneEquation (target, mpyramid.base.q[0], mpyramid.base.q[1], mpyramid.base.q[2], plane_eq2);

            Eigen::Vector4f apex1 = source->points[pyramid.apex].getVector4fMap ();
            Eigen::Vector4f apex2 = target->points[mpyramid.apex].getVector4fMap ();

            float d1 = fabs (plane_eq1.dot (apex1));
            float d2 = fabs (plane_eq2.dot (apex2));

            cout << "\n\nApex base distance = (" << d1 << " " << d2 << ")\n";

            if (fabs (d1 - d2) < param.length_similarity_threshold) {
              return true;
            }

            return false;
          }

              bool Extended4PCS::pyramidEdgeLengthCheck (PyramidMatch& pmatch, 
                  Pyramid& mpyramid)
              {
                bool status = true;
                Quad& quad = mpyramid.base;

                Eigen::Vector3f a = target->points[quad.q[0]].getVector3fMap ();
                Eigen::Vector3f b = target->points[quad.q[1]].getVector3fMap ();
                Eigen::Vector3f c = target->points[quad.q[2]].getVector3fMap ();
                Eigen::Vector3f d = target->points[quad.q[3]].getVector3fMap ();
                Eigen::Vector3f e = target->points[mpyramid.apex].getVector3fMap ();

                float base_apex_len[4];

                base_apex_len[0] = (a-e).norm ();
                base_apex_len[1] = (b-e).norm ();
                base_apex_len[2] = (c-e).norm ();
                base_apex_len[3] = (d-e).norm ();

                //cout << "Pyramid Edge Length Check ::  ";

                for (int i = 0; i < 4; i++) {
                  //cout << "( " << pmatch.base_apex_len[i] << " , " << base_apex_len[i] << " )  ";
                  if (fabs (base_apex_len[i] - pmatch.base_apex_len[i]) 
                      > param.length_similarity_threshold * 2) {
                    status = false;
                  }
                }
                cout << endl;
                return status;
              }

              void Extended4PCS::findSimilarQuads ()
              {
                for (int i = 0; i < quadMatchTable.size (); i++) {
                  cout << "\n[[[ ----  Finding matching quad for Quad " 
                    << i+1 << " --- ]]]\n";
                  QuadMatch& qmatch = quadMatchTable[i];

                  KdTree& kdtree = qmatch.kdtree;
                  Quad* quad = qmatch.quad;
                  float r1 (quad->r1);
                  float r2 (quad->r2);

                  vector <int>& r1_pts = *(qmatch.r1_pts);

                  //  Creating a point cloud containing r1 intersections
                  //  A KdTree is then created for this point cloud
                  CloudPtr intersections (new Cloud);

                  for (int i = 0; i < r1_pts.size (); i += 2) {
                    Eigen::Vector3f a = target->points[r1_pts[i]].getVector3fMap ();
                    Eigen::Vector3f b = target->points[r1_pts[i+1]].getVector3fMap ();

                    if ( ( (a-b).norm () - qmatch.ab_len)  > param.length_similarity_threshold){
                      //cout << "*** Point pairs NOT MATCHING AB length criteria ..\n";
                    }
                    else {
                      //cout << "*** Point pairs MATCHING AB length criteria ..\n";
                    }

                    Eigen::Vector3f e1_1 = a + r1 * (b-a);
                    Point pt1 (e1_1 (0), e1_1 (1), e1_1 (2));
                    intersections->points.push_back (pt1);


                    Eigen::Vector3f e1_2 = b + r1 * (a-b);
                    Point pt2 (e1_2 (0), e1_2 (1), e1_2 (2));
                    intersections->points.push_back (pt2);
                  }

                  intersections->width = 1;
                  intersections->height = intersections->points.size ();

                  if (intersections->points.size () == 0) {
                    return;
                  }

                  //cout << "# of point pairs matching AB = " << r1_pts.size () << endl;
                  kdtree.setInputCloud (intersections);
                  //cout << "# of r1 intersections = " << intersections->points.size () << endl;


                  vector <int>& r2_pts = *(qmatch.r2_pts);

                  for (int i = 0; i < r2_pts.size (); i += 2) {
                    Eigen::Vector3f a = target->points[r2_pts[i]].getVector3fMap ();
                    Eigen::Vector3f b = target->points[r2_pts[i+1]].getVector3fMap ();

                    if ( ( (a-b).norm () - qmatch.cd_len)  > param.length_similarity_threshold) {
                      //cout << "*** Point pairs NOT MATCHING CD length criteria ..\n";
                    }
                    else {
                      //cout << "*** Point pairs MATCHING CD length criteria ..\n";
                    }

                    Eigen::Vector3f e2_1 = a + r2 * (b-a);
                    Point pt1 (e2_1 (0), e2_1 (1), e2_1 (2));

                    int index = findMatchingPoint (kdtree, pt1); // returns index from cloud created in the above loop
                    if (index != -1) {
                      if ( (index % 2) == 0) {
                        // Matching quad pairs are {r2_pts[i], r2_pts[i+1], r1_pts[index], r1_pts[index+1]}
                        // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
                        Quad mquad;
                        mquad.q[0] = r1_pts[index];
                        mquad.q[1] = r1_pts[index+1];
                        mquad.q[2] = r2_pts[i];
                        mquad.q[3] = r2_pts[i+1];
                        //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
                        //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
                        //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

                        if (angleCheck (*quad, mquad)) {
                          vector <int> pl_pts;
                          vector <int> npl_pts;
                          findPointsOnPlane (target, mquad.q[0], mquad.q[1], 
                              mquad.q[2], pl_pts, npl_pts);
                          //cout << "# of non plane points = " << npl_pts.size () << endl;
                          qmatch.matches.push_back (mquad);
                        }
                      }
                      else {
                        // Matching quad pairs are {r2_pts[i], r2_pts[i+1], r1_pts[index], r1_pts[index-1]}
                        // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }
                        Quad mquad;
                        mquad.q[0] = r1_pts[index];
                        mquad.q[1] = r1_pts[index-1];
                        mquad.q[2] = r2_pts[i];
                        mquad.q[3] = r2_pts[i+1];
                        //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
                        //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
                        //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

                        if (angleCheck (*quad, mquad)) {
                          vector <int> pl_pts;
                          vector <int> npl_pts;
                          findPointsOnPlane (target, mquad.q[0], mquad.q[1], 
                              mquad.q[2], pl_pts, npl_pts);
                          //cout << "# of non plane points = " << npl_pts.size () << endl;
                          qmatch.matches.push_back (mquad);
                        }
                      }
                    }

                    Eigen::Vector3f e2_2 = b + r2 * (a-b);
                    Point pt2 (e2_2 (0), e2_2 (1), e2_2 (2));

                    index = findMatchingPoint (kdtree, pt2);
                    if (index != -1) {
                      if ( (index % 2) == 0) {
                        // Matching quad pairs are (r2_pts[i+1], r2_pts[i], r1_pts[index], r1_pts[index+1])
                        // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }

                        Quad mquad;
                        mquad.q[0] = r1_pts[index];
                        mquad.q[1] = r1_pts[index+1];
                        mquad.q[2] = r2_pts[i+1];
                        mquad.q[3] = r2_pts[i];
                        //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
                        //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
                        //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

                        if (angleCheck (*quad, mquad)) {
                          vector <int> pl_pts;
                          vector <int> npl_pts;
                          findPointsOnPlane (target, mquad.q[0], mquad.q[1], 
                              mquad.q[2], pl_pts, npl_pts);
                          //cout << "# of non plane points = " << npl_pts.size () << endl;
                          qmatch.matches.push_back (mquad);
                        }
                      }
                      else {
                        // Matching quad pairs are (r2_pts[i+1], r2_pts[i], r1_pts[index], r1_pts[index-1])
                        // and { quad.q[0], quad.q[1], quad.q[2], quad.q[3] }

                        Quad mquad;
                        mquad.q[0] = r1_pts[index];
                        mquad.q[1] = r1_pts[index-1];
                        mquad.q[2] = r2_pts[i+1];
                        mquad.q[3] = r2_pts[i];
                        //cout << "Matching quads " << " ( " << quad->q[0] << " " << quad->q[1] 
                        //  << " " << quad->q[2] << " " << quad->q[3] << " ) ( " <<
                        //  mquad.q[0] << " " << mquad.q[1] << " " << mquad.q[2] << " " << mquad.q[3] << ")\n";

                        if (angleCheck (*quad, mquad)) {
                          vector <int> pl_pts;
                          vector <int> npl_pts;
                          findPointsOnPlane (target, mquad.q[0], mquad.q[1], 
                              mquad.q[2], pl_pts, npl_pts);
                          //cout << "# of non plane points = " << npl_pts.size () << endl;
                          qmatch.matches.push_back (mquad);
                        }
                      }
                    }
                  }
                }
              }

              bool Extended4PCS::angleCheck (Quad& quad, Quad& mquad)
              {

                Eigen::Vector3f a1 = source->points[quad.q[0]].getVector3fMap ();
                Eigen::Vector3f c1 = source->points[quad.q[2]].getVector3fMap ();
                Point e1_pt;
                findIntersection (source, quad.q[0], quad.q[1], quad.q[2], quad.q[3], e1_pt);
                Eigen::Vector3f e1 = e1_pt.getVector3fMap ();
                quad.intersection = e1;

                Eigen::Vector3f u1 = (a1-e1);
                Eigen::Vector3f v1 = (c1-e1);

                Eigen::Vector3f a2 = target->points[mquad.q[0]].getVector3fMap ();
                Eigen::Vector3f c2 = target->points[mquad.q[2]].getVector3fMap ();
                Point e2_pt;
                findIntersection (target, mquad.q[0], mquad.q[1], mquad.q[2], mquad.q[3], e2_pt);
                Eigen::Vector3f e2 = e2_pt.getVector3fMap ();
                mquad.intersection = e2;

                Eigen::Vector3f u2 = (a2-e2);
                Eigen::Vector3f v2 = (c2-e2);

                float angle1 = atan2 (u1.cross (v1).norm (), u1.dot (v1)) * 180. / M_PI;
                float angle2 = atan2 (u2.cross (v2).norm (), u2.dot (v2)) * 180. / M_PI;

                quad.intersect_angle = angle1;
                mquad.intersect_angle = angle2;


                if (fabs (angle1 - angle2) < param.angle_threshold){
                  return true;
                }

                return false;
              }

              int Extended4PCS::findMatchingPoint (KdTree& kdtree, Point pt)
              {
                int K = 1;
                vector <int> ids (K);
                vector <float> dist (K);

                if (kdtree.nearestKSearch (pt, K, ids, dist) > 0) {
                  cout << "Intersection search query ( " 
                       << pt.x << " , " << pt.y << " , " << pt.z 
                       << " ) ";
                  cout << "Intersection distance = " << dist[0] << " ";
                  if (dist[0] <= param.e_match_threshold) {
                    //cout << "dist = " << dist[0] << endl;
                    cout << ids[0] << endl;
                    return ids[0];
                  }
                  cout << -1 << endl;
                }
                else {
                  ostringstream ostr;
                  ostr << "Intersection search query ( " 
                       << pt.x << " , " << pt.y << " , " << pt.z 
                       << " ) ";
                  ostr << " ::: " << "Cannot find closest intersection point";
                  throw std::runtime_error (ostr.str ().c_str ());
                }

                return -1;
              }

              void Extended4PCS::insertToPyramidMatchTable (float length, int p, int q)
              {
                for (int i = 0; i < pyramidMatchTable.size (); i++) {
                  PyramidMatch& pmatch = pyramidMatchTable[i];
                  if ( fabs (pmatch.ab_len - length) < param.length_similarity_threshold) {
                    pmatch.r1_pts->push_back (p);
                    pmatch.r1_pts->push_back (q);
                    break;
                  }
                  if ( fabs (pmatch.cd_len - length) < param.length_similarity_threshold) {
                    pmatch.r2_pts->push_back (p);
                    pmatch.r2_pts->push_back (q);
                    break;
                  }
                }
              }

              void Extended4PCS::insertToQuadMatchTable (float length, int p, int q)
              {
                for (int i = 0; i < quadMatchTable.size (); i++) {
                  QuadMatch& qmatch = quadMatchTable[i];
                  if ( fabs (qmatch.ab_len - length) < param.length_similarity_threshold) {
                    qmatch.r1_pts->push_back (p);
                    qmatch.r1_pts->push_back (q);
                    break;
                  }
                  if ( fabs (qmatch.cd_len - length) < param.length_similarity_threshold) {
                    qmatch.r2_pts->push_back (p);
                    qmatch.r2_pts->push_back (q);
                    break;
                  }
                }
              }

              void Extended4PCS::initializePyramidMatchTable ()
              {
                pyramidMatchTable.resize (pyramids.size ());

                for (int i = 0; i < pyramids.size (); i++) {

                  pyramidMatchTable[i].pyramid = &pyramids[i];
                  Quad& quad = pyramids[i].base;

                  Eigen::Vector3f a = source->points[quad.q[0]].getVector3fMap ();
                  Eigen::Vector3f b = source->points[quad.q[1]].getVector3fMap ();
                  Eigen::Vector3f c = source->points[quad.q[2]].getVector3fMap ();
                  Eigen::Vector3f d = source->points[quad.q[3]].getVector3fMap ();
                  Eigen::Vector3f e = source->points[pyramids[i].apex].getVector3fMap ();

                  pyramidMatchTable[i].ab_len = (a-b).norm ();
                  pyramidMatchTable[i].cd_len = (c-d).norm ();
                  pyramidMatchTable[i].base_apex_len[0] = (a-e).norm ();
                  pyramidMatchTable[i].base_apex_len[1] = (b-e).norm ();
                  pyramidMatchTable[i].base_apex_len[2] = (c-e).norm ();
                  pyramidMatchTable[i].base_apex_len[3] = (d-e).norm ();

                  bool found = false;
                  vector <PointList>::iterator itr = pointListTable.begin ();
                  for (; itr != pointListTable.end (); itr++) {
                    if ( fabs (itr->len - pyramidMatchTable[i].ab_len) < 
                        param.length_similarity_threshold) {
                      found = true;
                      break;
                    }
                  }

                  if (!found) {
                    PointList pl;
                    pl.len = pyramidMatchTable[i].ab_len;
                    pointListTable.push_back (pl);
                  }

                  found = false;
                  itr = pointListTable.begin ();
                  for (; itr != pointListTable.end (); itr++) {
                    if ( fabs (itr->len - pyramidMatchTable[i].cd_len) < 
                        param.length_similarity_threshold) {
                      found = true;
                      break;
                    }
                  }
                  if (!found) {
                    PointList pl;
                    pl.len = pyramidMatchTable[i].cd_len;
                    pointListTable.push_back (pl);
                  }
                }

                for (int i = 0; i < pyramidMatchTable.size (); i++) {
                  vector <PointList>::iterator itr = pointListTable.begin ();
                  for (; itr != pointListTable.end (); itr++) {
                    // point within 1 m
                    if ( fabs (itr->len - pyramidMatchTable[i].ab_len) < 
                        param.length_similarity_threshold) {
                      pyramidMatchTable[i].r1_pts = &(itr->points);
                    }
                    if ( fabs (itr->len - pyramidMatchTable[i].cd_len) < 
                        param.length_similarity_threshold) {
                      pyramidMatchTable[i].r2_pts = &(itr->points);
                    }
                  }
                }

              }

              void Extended4PCS::initializeQuadMatchTable ()
              {
                quadMatchTable.resize (quads.size ());

                for (int i = 0; i < quads.size (); i++) {

                  quadMatchTable[i].quad = &quads[i];

                  Eigen::Vector3f a = source->points[quads[i].q[0]].getVector3fMap ();
                  Eigen::Vector3f b = source->points[quads[i].q[1]].getVector3fMap ();
                  Eigen::Vector3f c = source->points[quads[i].q[2]].getVector3fMap ();
                  Eigen::Vector3f d = source->points[quads[i].q[3]].getVector3fMap ();

                  quadMatchTable[i].ab_len = (a-b).norm ();
                  quadMatchTable[i].cd_len = (c-d).norm ();

                  bool found = false;
                  vector <PointList>::iterator itr = pointListTable.begin ();
                  for (; itr != pointListTable.end (); itr++) {
                    // point within 1 m
                    if ( fabs (itr->len - quadMatchTable[i].ab_len) < 
                        param.length_similarity_threshold) {
                      found = true;
                      break;
                    }
                  }

                  if (!found) {
                    PointList pl;
                    pl.len = quadMatchTable[i].ab_len;
                    pointListTable.push_back (pl);
                  }

                  found = false;
                  itr = pointListTable.begin ();
                  for (; itr != pointListTable.end (); itr++) {
                    if ( fabs (itr->len - quadMatchTable[i].cd_len) < 
                        param.length_similarity_threshold) {
                      found = true;
                      break;
                    }
                  }
                  if (!found) {
                    PointList pl;
                    pl.len = quadMatchTable[i].cd_len;
                    pointListTable.push_back (pl);
                  }
                }


                for (int i = 0; i < quadMatchTable.size (); i++) {
                  vector <PointList>::iterator itr = pointListTable.begin ();
                  for (; itr != pointListTable.end (); itr++) {
                    // point within 1 m
                    if ( fabs (itr->len - quadMatchTable[i].ab_len) < 
                        param.length_similarity_threshold) {
                      quadMatchTable[i].r1_pts = &(itr->points);
                    }
                    if ( fabs (itr->len - quadMatchTable[i].cd_len) < 
                        param.length_similarity_threshold) {
                      quadMatchTable[i].r2_pts = &(itr->points);
                    }
                  }
                }
              }

              void Extended4PCS::selectOneQuad (Quad& quad, vector <int>& plane_pts)
              {
                int z = 0;

                while (true) {

                  if (z==4) {
                    break;
                  }

                  int k = rand () % plane_pts.size ();

                  bool too_close = false;
                  for (int p = 0; p < z; p++) {
                    Point p1 = source->points[plane_pts[k]];
                    Point p2 = source->points[quad.q[p]];
                    if ( (p1.getVector3fMap () - p2.getVector3fMap ()).norm () < param.min_dist) {
                      too_close = true;
                      break;
                    }
                  }
                  if (too_close) {
                    continue;
                  }

                  quad.q[z++] = plane_pts[k];
                }
              }


              void Extended4PCS::selectBase (Pyramid& pyramid, vector <int>& plane_pts)
              {
                int z = 0;

                Quad& quad = pyramid.base;

                while (true) {

                  if (z == 4) {
                    break;
                  }

                  int k = rand () % plane_pts.size ();

                  bool too_close = false;
                  for (int p = 0; p < z; p++) {
                    Point p1 = source->points[plane_pts[k]];
                    Point p2 = source->points[quad.q[p]];
                    if ( (p1.getVector3fMap () - p2.getVector3fMap ()).norm () < param.min_dist) {
                      too_close = true;
                      break;
                    }
                  }
                  if (too_close) {
                    continue;
                  }

                  quad.q[z++] = plane_pts[k];

                  //cout << "z = " << z << endl;
                }
              }

              void Extended4PCS::addApex (CloudPtr& cloud, Pyramid& pyramid, 
                  vector <int>& non_plane_pts, float radius)
              {
                cout << "Adding Apex .. # of non plane points = " << non_plane_pts.size () << "\n";
                Quad& quad = pyramid.base;
                Point pt;
                //cout << "cloud size = " << cloud->points.size () << endl;
                findIntersection (cloud, quad.q[0], quad.q[1], quad.q[2], quad.q[3], pt);

                pcl::IndicesPtr indices (new vector <int>);
                //indices->resize (non_plane_pts.size ());
                copy (non_plane_pts.begin (), non_plane_pts.end (), back_inserter (*indices));

                KdTreePtr tree (new KdTree);
                tree->setInputCloud (cloud, indices);

                vector <int> ids;
                vector <float> dists;

                fill (ids.begin (), ids.end (), -1);
                fill (dists.begin (), dists.end (), 0.);

                Eigen::Vector3f a = cloud->points[quad.q[0]].getVector3fMap ();
                Eigen::Vector3f b = cloud->points[quad.q[1]].getVector3fMap ();
                Eigen::Vector3f c = cloud->points[quad.q[2]].getVector3fMap ();
                Eigen::Vector3f d = cloud->points[quad.q[3]].getVector3fMap ();

                float ab_len = (a-b).norm ();
                float cd_len = (c-d).norm ();

                Eigen::Vector4f plane_eq;
                plane_eq.setZero ();
                findPlaneEquation (cloud, quad.q[0], quad.q[1], quad.q[2], plane_eq);

                if (tree->radiusSearch (pt, radius, ids, dists) > 0) {
                  int index = -1;
                  float pd = numeric_limits <float>::min (); // perpendicular disatance
                  foreach (int& i, ids) {
                    Eigen::Vector4f q = cloud->points[i].getVector4fMap ();
                    float l = fabs (plane_eq.dot (q));
                    //cout << "( " << pd << " , " << l << " ) ";
                    if (l > pd) {
                      pd = l;
                      //cout << "changing pd ";
                      index = i;
                    }
                  }
                  cout << "Longest perpendicular distance = " << pd << endl;
                  pyramid.apex = index;
                }
              }
              void Extended4PCS::addApex (CloudPtr& cloud, Pyramid& pyramid, 
                  vector <int>& non_plane_pts)
              {
                cout << "Adding Apex .. # of non plane points = " << non_plane_pts.size () << "\n";
                Quad& quad = pyramid.base;
                Point pt;
                //cout << "cloud size = " << cloud->points.size () << endl;
                findIntersection (cloud, quad.q[0], quad.q[1], quad.q[2], quad.q[3], pt);

                Eigen::Vector4f plane_eq;
                plane_eq.setZero ();
                findPlaneEquation (cloud, quad.q[0], quad.q[1], quad.q[2], plane_eq);

                pcl::IndicesPtr indices (new vector <int>);
                copy (non_plane_pts.begin (), non_plane_pts.end (), back_inserter (*indices));

                KdTreePtr tree (new KdTree);
                CloudPtr tmp_cloud (new Cloud);
                for (int i = 0; i < non_plane_pts.size (); i++) {
                  tmp_cloud->points.push_back (cloud->points[non_plane_pts[i]]);
                }
                tmp_cloud->width = 1;
                tmp_cloud->height = tmp_cloud->points.size ();

                //tree->setInputCloud (tmp_cloud);
                tree->setInputCloud (cloud, indices);

                int K = non_plane_pts.size ();

                vector <int> ids (K);
                vector <float> dists (K);

                if (tree->nearestKSearch (pt, K, ids, dists) > 0) {
                  int index = -1;
                  //index = ids[ids.size ()-1];
                  float pd = numeric_limits <float>::min (); // perpendicular distance
                  int z = 0.2 * ids.size ();
                  while (z < 0.35 * ids.size ()) {
                    Eigen::Vector4f q = cloud->points[ids[z]].getVector4fMap ();
                    float l = fabs (plane_eq.dot (q));
                    if (l > pd) {
                      pd = l;
                      index = ids[z];
                    }
                    z++;
                  }
                  // This is temporary
                  //index = ids[0.7 * ids.size ()];
                  cout << "Longest perpendicular distance = " << pd << endl;
                  pyramid.apex = index;
                  cout << "Pyramid apex index = " << pyramid.apex << endl;
                }
              }

              void Extended4PCS::addApex (CloudPtr& cloud, Pyramid& pyramid, 
                  vector <int>& non_plane_pts, int K)
              {
                cout << "Adding Apex .. # of non plane points = " << non_plane_pts.size () << "\n";
                Quad& quad = pyramid.base;
                Point pt;
                //cout << "cloud size = " << cloud->points.size () << endl;
                findIntersection (cloud, quad.q[0], quad.q[1], quad.q[2], quad.q[3], pt);

                pcl::IndicesPtr indices (new vector <int>);
                //indices->resize (non_plane_pts.size ());
                copy (non_plane_pts.begin (), non_plane_pts.end (), back_inserter (*indices));

                KdTreePtr tree (new KdTree);
                tree->setInputCloud (cloud, indices);

                vector <int> ids (K);
                vector <float> dists (K);

                fill (ids.begin (), ids.end (), -1);
                fill (dists.begin (), dists.end (), 0.);

                Eigen::Vector3f a = cloud->points[quad.q[0]].getVector3fMap ();
                Eigen::Vector3f b = cloud->points[quad.q[1]].getVector3fMap ();
                Eigen::Vector3f c = cloud->points[quad.q[2]].getVector3fMap ();
                Eigen::Vector3f d = cloud->points[quad.q[3]].getVector3fMap ();

                float ab_len = (a-b).norm ();
                float cd_len = (c-d).norm ();

                Eigen::Vector4f plane_eq;
                plane_eq.setZero ();
                findPlaneEquation (cloud, quad.q[0], quad.q[1], quad.q[2], plane_eq);

                if (tree->nearestKSearch (pt, K, ids, dists) > 0) {
                  int index = -1;
                  float pd = numeric_limits <float>::min (); // perpendicular disatance
                  foreach (int& i, ids) {
                    Eigen::Vector4f q = cloud->points[i].getVector4fMap ();
                    float l = fabs (plane_eq.dot (q));
                    //cout << "( " << pd << " , " << l << " ) ";
                    if (l > pd) {
                      pd = l;
                      //cout << "changing pd ";
                      index = i;
                    }
                  }
                  cout << "Longest perpendicular distance = " << pd << endl;
                  pyramid.apex = index;
                  cout << "Pyramid apex index = " << pyramid.apex << endl;
                }
              }

              void Extended4PCS::findIntersection (CloudPtr cloud, int x1, int x2, 
                  int x3, int x4, Point& intersection)
              {
                Eigen::Vector3f a = cloud->points[x2].getVector3fMap () - 
                  cloud->points[x1].getVector3fMap () ;

                Eigen::Vector3f b = cloud->points[x4].getVector3fMap () - 
                  cloud->points[x3].getVector3fMap () ;

                Eigen::Vector3f c = cloud->points[x3].getVector3fMap () - 
                  cloud->points[x1].getVector3fMap () ;

                Eigen::Vector3f cb = c.cross (b);
                Eigen::Vector3f ab = a.cross (b);

                // e is the intersection
                Eigen::Vector3f e = cloud->points[x1].getVector3fMap () + 
                  a * (cb.dot (ab) / ab.dot (ab));

                //cout << "Intersection = " << e.transpose () << endl;

                intersection.x = e (0);
                intersection.y = e (1);
                intersection.z = e (2);
              }

              bool Extended4PCS::checkQuad (Quad& quad)
              {
                // check the quad by verifying if the line segements intersect
                // using method described in "http://mathworld.wolfram.com/Line-LineIntersection.html"
                // to compute 3D line intersection
                int& x1 = quad.q[0];
                int& x2 = quad.q[1];
                int& x3 = quad.q[2];
                int& x4 = quad.q[3];

                Point intersection;
                findIntersection (source, x1, x2, x3, x4, intersection);

                Eigen::Vector3f e = intersection.getVector3fMap ();

                // a, b, c and d are the four points
                Eigen::Vector3f a = source->points[x1].getVector3fMap ();
                Eigen::Vector3f b = source->points[x2].getVector3fMap ();
                Eigen::Vector3f c = source->points[x3].getVector3fMap ();
                Eigen::Vector3f d = source->points[x4].getVector3fMap ();

                float length = (a-b).norm ();
                // e should be on the line segment ab, so |e-a| or |e-b| 
                // should not exceed the length of |a-b|
                if ( ((e-b).norm () > length) || ((e-a).norm () > length) ) {
                  //cout << "Quad check failed..\n";
                  return false;
                }

                length = (c-d).norm ();
                // e should be on the line segment cd, so |e-c| or |e-d| 
                // should not exceed the length of |c-d|
                if ( ((e-c).norm () > length) || ((e-d).norm () > length) ) {
                  //cout << "Quad check failed..\n";
                  return false;
                }

                float r1 = (a-e).norm () / (a-b).norm ();
                float r2 = (c-e).norm () / (c-d).norm ();

                quad.r1 = r1;
                quad.r2 = r2;

                return true;
              }

              void Extended4PCS::selectPyramids1 (int N)
              {
                int pyramidId = 1;
                int K = 10;

                Pyramid pyramid;
                // Make sure that atleast one pyramid gets selected ..
                do {

                  int ta = 0, tb = 0, tc = 0;
                  select3Points (source, ta, tb, tc);
                  plane_pts.clear ();
                  non_plane_pts.clear ();

                  findPointsOnPlane (source, ta, tb, tc, plane_pts, non_plane_pts);
                  selectBase (pyramid, plane_pts);

                  if (checkQuad (pyramid.base)) {
                    cout << "Selected base ..\n";
                    addApex (source, pyramid, non_plane_pts, K);
                    cout << "Added apex ..\n";
                    pyramid.pyramidId = pyramidId++;
                    pyramids.push_back (pyramid);
                    break;
                  }
                  //cout << "Pyramid base check failed ..\n";
                } while (true);


                cout << "\n<< Selected Pyramid 1 >> ..\n";
                // Select the rest N-1 pyramids
                for (int i = 2; i <= N; i++) { // Select N pyramids

                  Pyramid pyramid;

                  int tries = 1;
                  do {
                    int ta = 0, tb = 0, tc = 0;
                    select3Points (source, ta, tb, tc);
                    plane_pts.clear ();
                    non_plane_pts.clear ();

                    findPointsOnPlane (source, ta, tb, tc, plane_pts, non_plane_pts);
                    selectBase (pyramid, plane_pts);

                    if (checkQuad (pyramid.base)) {
                      cout << "Selected base ..\n";
                      addApex (source, pyramid, non_plane_pts, K);
                      cout << "Added apex ..\n";
                      pyramid.pyramidId = pyramidId++;
                      pyramids.push_back (pyramid);
                      break;
                    }
                  } while (tries++ < 10);

                  if (tries == 11) {
                    cout << "<< Could not select pyramid " << i << " tries = " << tries << " >>\n";
                  }
                  else {
                    cout << "<< Selected pyramid " << i << " Number of tries  = " << tries << " >> .. \n";
                    cout << "<< Selected pyramid " << i << " >> .. \n";
                  }
                }

              }

              void Extended4PCS::selectPyramids (vector <int>& plane_pts, int N)
              {
                int pyramidId = 1;
                int K = 10;

                Pyramid pyramid;
                // Make sure that atleast one pyramid gets selected ..
                do {
                  selectBase (pyramid, plane_pts);
                  if (checkQuad (pyramid.base)) {
                    cout << "Selected base ..\n";
                    //addApex (source, pyramid, non_plane_pts, K);
                    addApex (source, pyramid, non_plane_pts);
                    cout << "Added apex ..\n";
                    pyramid.pyramidId = pyramidId++;
                    pyramids.push_back (pyramid);
                    break;
                  }
                  //cout << "Pyramid base check failed ..\n";
                } while (true);

                cout << "\n<< Selected Pyramid 1 >> ..\n";
                // Select the rest N-1 pyramids
                for (int i = 2; i <= N; i++) { // Select N pyramids

                  Pyramid pyramid;

                  int tries = 1;
                  do {
                    selectBase (pyramid, plane_pts);
                    if (checkQuad (pyramid.base)) {
                      cout << "Selected base ..\n";
                      //addApex (source, pyramid, non_plane_pts, K);
                      addApex (source, pyramid, non_plane_pts);
                      cout << "Added apex ..\n";
                      pyramid.pyramidId = pyramidId++;
                      pyramids.push_back (pyramid);
                      break;
                    }
                  } while (tries++ < 10);

                  if (tries == 11) {
                    cout << "<< Could not select pyramid " << i << " tries = " << tries << " >>\n";
                  }
                  else {
                    cout << "<< Selected pyramid " << i << " Number of tries  = " << tries << " >> .. \n";
                    cout << "<< Selected pyramid " << i << " >> .. \n";
                  }
                }
              }

              void Extended4PCS::selectQuads (vector <int>& plane_pts, int N)
              {
                int quadId = 1;

                Quad quad;
                // Make sure that atleast one quad gets selected ..
                do {
                  selectOneQuad (quad, plane_pts);
                  if (checkQuad (quad)) {
                    quad.quadId = quadId++;
                    quads.push_back (quad);
                    break;
                  }
                  //cout << "Quad check failed ..\n";
                } while (true);

                cout << "\n<< Selected quad 1 >> ..\n";
                // Select the rest N-1 quads
                for (int i = 2; i <= N; i++) { // Select N quads

                  Quad quad;

                  int tries = 1;
                  do {
                    selectOneQuad (quad, plane_pts);
                    if (checkQuad (quad)) {
                      quad.quadId = quadId++;
                      quads.push_back (quad);
                      break;
                    }
                    //cout << "Quad check failed ..\n";
                  } while (tries++ < 10);

                  if (tries == 11) {
                    cout << "<< Could not select quad " << i << " tries = " << tries << " >>\n";
                  }
                  else {
                    cout << "<< Selected quad " << i << " Number of tries  = " << tries << " >> .. \n";
                    cout << "<< Selected quad " << i << " >> .. \n";
                  }

                  //cout << "\nPairwise distances\n-----------------\n";
                  //for (int i = 0; i < 4; i++) {
                  //  for (int j = 0; j < 4; j++) {
                  //    if (i == j) continue;
                  //    Point p1 = source->points[quad.q[i]];
                  //    Point p2 = source->points[quad.q[j]];

                  //    float dist = (p1.getVector3fMap () - p2.getVector3fMap ()).norm ();
                  //    cout << dist <<  " ";
                  //  }
                  //}
                  //cout << "\n-----------------\n";
                }
              }

              void Extended4PCS::selectMaxPlane ()
              {
                int a = -1, b = -1, c = -1;

                int z = 0;
                int max_pts = 0;

                srand (time (NULL));

                //cout << "FINDING THE BEST PLANE :: ";
                while (z++ < param.random_tries) {
                  int ta = 0, tb = 0, tc = 0;
                  select3Points (source, ta, tb, tc);
                  vector <int> pts;
                  vector <int> n_pts;
                  findPointsOnPlane (source, ta, tb, tc, pts, n_pts);
                  //cout << pts.size () << " ";

                  if (pts.size () > max_pts) {
                    max_pts = pts.size ();
                    a = ta, b = tb, c = tc;
                  }
                  //cout << "z = " << z << " max points = " << max_pts << endl;
                }
                cout << endl;

                findPointsOnPlane (source, a, b, c, this->plane_pts, this->non_plane_pts);
                cout << "# of points on the plane = " << plane_pts.size () << endl;
                cout << "# of points not the plane = " << non_plane_pts.size () << endl;

                cloud_plane.reset (new Cloud);
                cloud_non_plane.reset (new Cloud);

                foreach (int& i, plane_pts) {
                  cloud_plane->points.push_back (source->points[i]);
                }
                foreach (int& i, non_plane_pts) {
                  cloud_non_plane->points.push_back (source->points[i]);
                }
                cloud_plane->width = cloud_non_plane->width = 1;
                cloud_plane->height = cloud_plane->points.size ();
                cloud_non_plane->height = cloud_non_plane->points.size ();


                Eigen::Vector4f p_eq;
                findPlaneEquation (source, a, b, c, p_eq);

                float pd = numeric_limits <float>::min (); // perpendicular disatance
                for (int i = 0; i < source->points.size (); i++) {
                  Eigen::Vector4f q = source->points[i].getVector4fMap ();
                  float l = fabs (p_eq.dot (q));
                  if (l > pd) {
                    pd = l;
                  }
                }
                cout << "Longest perpendicular distance from Max plane = " << pd << endl;
              }

              void Extended4PCS::findPlaneEquation (CloudPtr cloud, int a, int b, int c, 
                  Eigen::Vector4f& eq)
              {
                Point pa = cloud->points[a];
                Point pb = cloud->points[b];
                Point pc = cloud->points[c];

                Eigen::Vector3f centroid;
                centroid.setZero ();

                centroid (0) = (pa.x + pb.x + pc.x ) / 3.0;
                centroid (1) = (pa.y + pb.y + pc.y ) / 3.0;
                centroid (2) = (pa.z + pb.z + pc.z ) / 3.0;

                Eigen::Matrix3f covariance;
                covariance.setZero ();

                Eigen::Vector3f V;
                V.setZero ();
                V = (pa.getVector3fMap () - centroid) ;
                covariance += (V * V.transpose ());

                V.setZero ();
                V = (pb.getVector3fMap () - centroid);
                covariance += (V * V.transpose ());

                V.setZero ();
                V = (pc.getVector3fMap () - centroid);
                covariance += (V * V.transpose ());


                Eigen::EigenSolver <Eigen::Matrix3f> es (covariance);
                //cout << "The eigen values are " << es.eigenvalues () << endl;
                //cout << "\n\n\n" << endl;

                Eigen::Vector3d eigenvalues;
                eigenvalues.setZero ();
                pcl::eigen33 (covariance, eigenvalues);
                //cout << "The eigen values are " << eigenvalues.transpose () << endl << endl;

                Eigen::Vector3f eigenvector1;
                eigenvector1.setZero ();
                pcl::computeCorrespondingEigenVector (covariance, eigenvalues [2], eigenvector1);

                Eigen::Vector3f eigenvector2;
                eigenvector2.setZero ();
                pcl::computeCorrespondingEigenVector (covariance, eigenvalues [1], eigenvector2);

                Eigen::Vector3f eigenvector3;
                eigenvector3.setZero ();
                pcl::computeCorrespondingEigenVector (covariance, eigenvalues [0], eigenvector3);

                //cout << "Eigen values = " << eigenvalues[0] << " " << eigenvalues[1] << " " << eigenvalues[2] << endl;

                float A = eigenvector3 (0);
                float B = eigenvector3 (1);
                float C = eigenvector3 (2);
                float D = - (A*centroid (0) + B*centroid (1) + C*centroid (2));

                eq.block <3,1> (0, 0) = eigenvector3;
                eq (3) = -(centroid.dot (eigenvector3));
              }

              void Extended4PCS::findPointsOnPlane (CloudPtr cloud, int a, int b,
                  int c, vector <int>& plane_pts,
                  vector <int>& non_plane_pts)
              {
                plane_pts.clear ();
                non_plane_pts.clear ();

                Eigen::Vector4f plane_eq;
                plane_eq.setZero ();

                findPlaneEquation (cloud, a, b, c, plane_eq);
                float A = plane_eq (0);
                float B = plane_eq (1);
                float C = plane_eq (2);
                float D = plane_eq (3);

                plane_pts.push_back (a), plane_pts.push_back (b), plane_pts.push_back (c);

                for (int i = 0; i < cloud->points.size (); i++) {

                  if (i == a || i == b || i == c) {
                    continue;
                  }

                  float x = cloud->points[i].x;
                  float y = cloud->points[i].y;
                  float z = cloud->points[i].z;

                  float residue = fabs (A*x + B*y + C*z + D);
                  if ( residue < param.plane_fit_threshold) {
                    plane_pts.push_back (i);
                  }
                  else {
                    non_plane_pts.push_back (i);
                  }
                }
              }

              void Extended4PCS::select3Points (CloudPtr cloud, int& a, int& b, int& c)
              {
                int N = cloud->points.size ();


                a = rand () % N;
                b = rand () % N;
                c = rand () % N;

                Point pa = cloud->points[a];
                Point pb = cloud->points[b];
                Point pc = cloud->points[c];

                Eigen::Vector3f u = pa.getVector3fMap () - pb.getVector3fMap ();
                Eigen::Vector3f v = pa.getVector3fMap () - pc.getVector3fMap ();

                if (0.5 * u.cross (v).norm () // area of the traingle formed by
                    // the 3 points should not be zero, this is a check for collinearity
                    && (u.norm () > param.min_dist) // minimum distance between selected points
                    && (v.norm () > param.min_dist) ) {
                  return;
                }
              }


            }
