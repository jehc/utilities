#ifndef BUNDLE_POINT_H
#define BUNDLE_POINT_H

#include <vector>
#include <Eigen/Dense>
#include "BundleView.h"

class BundlePoint
{
  Eigen::Vector3d position;
  Eigen::Vector3i color;
  std::vector<BundleView> views;
  friend std::istream & operator>> (std::istream &, BundlePoint &);
  friend std::ostream & operator<< (std::ostream &, const BundlePoint &);
public:
  BundlePoint () { }
  BundlePoint (const Eigen::Vector3d &, const Eigen::Vector3i &, const std::vector<BundleView> &);
  inline Eigen::Vector3d GetPosition() const { return position; }
  inline Eigen::Vector3i GetColor() const { return color; }
  inline const std::vector<BundleView> & GetViews() const { return views; }
  inline void SetPosition (const Eigen::Vector3d & p) { position = p; }
  inline void AddView (const BundleView & view) { views.push_back (view); }
};

#endif
