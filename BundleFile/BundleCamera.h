#ifndef BUNDLE_CAMERA_H
#define BUNDLE_CAMERA_H

#include <istream>
#include <opencv2/opencv.hpp>

class BundleCamera
{
  bool valid;
  float f, k1, k2;
  cv::Mat R, t;
  friend istream & operator>> (istream &, BundleCamera &);
public:
  inline float GetF() const { return f; }
  inline float GetK1() const { return k1; }
  inline float GetK2() const { return k2; }
  inline const cv::Mat & GetR() const { return R; }
  inline const cv::Mat & GetT() const { return t; }
  inline bool IsValid() const { return valid; }
};

#endif
