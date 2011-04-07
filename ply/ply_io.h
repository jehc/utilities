#ifndef PLY_IO_H
#define PLY_IO_H

#include <string>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

union RgbConverter { float rgb; struct { unsigned char r; unsigned char g; unsigned char b; }; };

void savePlyFile (const std::string &, const pcl::PointCloud<pcl::PointXYZRGBNormal> &);

void savePlyFile (const std::string &, const pcl::PointCloud<pcl::PointXYZRGB> &);

#endif
