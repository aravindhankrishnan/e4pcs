#pragma once

#include <pcl/io/pcd_io.h>


namespace E4PCS {

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PointCloud<Point>::Ptr CloudPtr;

}
