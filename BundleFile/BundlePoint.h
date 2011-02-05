#ifndef BUNDLE_POINT_H
#define BUNDLE_POINT_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "BundleView.h"

class BundlePoint
{
  cv::Vec3f position;
  cv::Vec3b color;
  std::vector<BundleView> views;
  friend istream & operator>> (istream &, BundlePoint &);
  friend ostream & operator<< (ostream &, const BundlePoint &);
public:
  BundlePoint () { }
  BundlePoint (const cv::Vec3f &, const cv::Vec3b &);
};

#endif
