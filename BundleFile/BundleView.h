#ifndef BUNDLE_VIEW_H
#define BUNDLE_VIEW_H

#include <istream>

class BundleView
{
  int camera, key;
  double x, y;
  friend std::istream & operator>> (std::istream &, BundleView &);
  friend std::ostream & operator<< (std::ostream &, const BundleView &);
  public:
  BundleView () {}
  BundleView (int camera, int key, double x, double y):camera(camera),key(key),x(x),y(y){}
  inline int GetCamera() const { return camera; }
  inline int GetKey() const { return key; }
  inline double GetX() const { return x; }
  inline double GetY() const { return y; }
};

#endif
