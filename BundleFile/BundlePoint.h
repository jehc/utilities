#ifndef BUNDLE_POINT_H
#define BUNDLE_POINT_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "BundleView.h"

class BundlePoint
{
  cv::Mat position;
  cv::Vec3b color;
  std::vector<BundleView> views;
  friend istream & operator>> (istream &, BundlePoint &);
  friend ostream & operator<< (ostream &, const BundlePoint &);
public:
  BundlePoint ():position(cv::Mat(3,1,CV_32FC1)) { }
  BundlePoint (const cv::Mat &, const cv::Vec3b &);
  BundlePoint (const BundlePoint & p) { Copy(p); }
  BundlePoint & operator= (const BundlePoint & p) { Copy(p); return *this; }
  void Copy (const BundlePoint & p)
  {
    position = p.position.clone();
    color = p.color;
    views = p.views;
  }
  inline cv::Mat GetPosition() const { return position; }
  inline cv::Vec3b GetColor() const { return color; }
  inline const std::vector<BundleView> & GetViews() const { return views; }
  inline void SetPosition (const cv::Mat & p) { position = p.clone(); }
};

#endif
