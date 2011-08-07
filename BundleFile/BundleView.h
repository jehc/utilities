#ifndef BUNDLE_VIEW_H
#define BUNDLE_VIEW_H

#include <istream>

class BundleView
{
  int camera, key;
  float x, y;
  friend std::istream & operator>> (std::istream &, BundleView &);
  friend std::ostream & operator<< (std::ostream &, const BundleView &);
  public:
  BundleView () {}
  BundleView (int camera, int key, float x, float y):camera(camera),key(key),x(x),y(y){}
  inline int GetCamera() const { return camera; }
  inline int GetKey() const { return key; }
  inline float GetX() const { return x; }
  inline float GetY() const { return y; }
};

#endif
