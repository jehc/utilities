#ifndef BUNDLE_VIEW_H
#define BUNDLE_VIEW_H

#include <istream>

class BundleView
{
  int camera, key;
  float x, y;
  friend std::istream & operator>> (std::istream &, BundleView &);
  public:
  inline int GetCamera() const { return camera; }
  inline float GetX() const { return x; }
  inline float GetY() const { return y; }
};

#endif
