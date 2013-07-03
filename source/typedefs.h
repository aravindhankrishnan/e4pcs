#pragma once

#include <pcl/io/pcd_io.h>


namespace E4PCS {

typedef pcl::PointXYZ Point;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PointCloud<Point>::Ptr CloudPtr;
typedef pcl::PointCloud<PointNormal> CloudNormal;
typedef pcl::PointCloud<PointNormal>::Ptr CloudNormalPtr;
}
