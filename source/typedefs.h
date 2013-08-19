#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

namespace E4PCS {

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZI PointIntensity;
typedef pcl::PointNormal PointNormal;
typedef pcl::PrincipalCurvatures PrincipalCurvatures;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PointCloud<Point>::Ptr CloudPtr;
typedef pcl::PointCloud<PointIntensity> CloudIntensity;
typedef pcl::PointCloud<PointIntensity>::Ptr CloudIntensityPtr;
typedef pcl::PointCloud<PointNormal> CloudNormal;
typedef pcl::PointCloud<PointNormal>::Ptr CloudNormalPtr;
typedef pcl::PointCloud<PrincipalCurvatures> CloudPC;
typedef pcl::PointCloud<PrincipalCurvatures>::Ptr CloudPCPtr;

//typedef pcl::KdTreeFLANN <Point> KdTree;
//typedef pcl::KdTreeFLANN <Point>::Ptr KdTreePtr;
typedef pcl::search::KdTree <Point> KdTree;
typedef pcl::search::KdTree <Point>::Ptr KdTreePtr;
typedef pcl::search::KdTree <PointNormal> KdTreeNormal;
typedef pcl::search::KdTree <PointNormal>::Ptr KdTreeNormalPtr;

}
