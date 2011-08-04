#ifndef PLY_IO_H
#define PLY_IO_H

#include <string>
#include <vector>
#include <GL/gl.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

union RgbConverter { float rgb; struct { unsigned char r; unsigned char g; unsigned char b; }; };
union ByteExtractor { uint32_t rgba; uint8_t b[4]; };

void savePlyFile (const std::string &, const pcl::PointCloud<pcl::PointXYZRGBNormal> &);

void savePlyFile (const std::string &, const pcl::PointCloud<pcl::PointXYZRGB> &);

void savePlyFile (const std::string &, const pcl::PointCloud<pcl::PointSurfel> &);

void loadPlyFile (const std::string &, pcl::PointCloud<pcl::PointXYZ> &);

void loadPlyFileGL (const std::string &, std::vector<GLfloat> &, std::vector<GLubyte> &, std::vector<GLuint> &);

#endif
