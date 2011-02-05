#ifndef BUNDLE_VIEW_H
#define BUNDLE_VIEW_H

#include <istream>
#include <opencv2/opencv.hpp>

class BundleView
{
  int camera, key;
  float x, y;
  friend istream & operator>> (istream &, BundleView &);
  public:
  inline int GetCamera() const { return camera; }
  inline float GetX() const { return x; }
  inline float GetY() const { return y; }
};

#endif
