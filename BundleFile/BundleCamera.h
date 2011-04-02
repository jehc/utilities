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
  BundleCamera():valid(false),f(0),k1(0),k2(0),R(cv::Mat(3,3,CV_32FC1)),t(cv::Mat(3,1,CV_32FC1)){}
  BundleCamera(const BundleCamera & c) { Copy(c); }
  BundleCamera & operator=(const BundleCamera & c) { Copy(c); return *this; }
  void Copy (const BundleCamera & c)
  {
    f = c.f;
    k1 = c.k1;
    k2 = c.k2;
    valid = c.valid;
    R = c.R.clone();
    t = c.t.clone();
  }
  inline float GetF() const { return f; }
  inline float GetK1() const { return k1; }
  inline float GetK2() const { return k2; }
  inline const cv::Mat & GetR() const { return R; }
  inline const cv::Mat & GetT() const { return t; }
  inline void SetR(const cv::Mat & newR) { R = newR.clone(); }
  inline void SetT(const cv::Mat & newT) { t = newT.clone(); }
  inline bool IsValid() const { return valid; }
};

#endif
