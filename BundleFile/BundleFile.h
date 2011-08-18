#ifndef BUNDLE_FILE_H
#define BUNDLE_FILE_H

#include <vector>
#include <string>
#include "BundleCamera.h"
#include "BundlePoint.h"

class BundleFile
{
  std::vector<BundleCamera> cameras;
  std::vector<BundlePoint> points;
  public:
  BundleFile (const std::string &);
  inline const std::vector<BundleCamera> & GetCameras() const { return cameras; }
  inline const std::vector<BundlePoint> & GetPoints() const { return points; } 
  inline void AddPoint (const BundlePoint & point) { points.push_back (point); }
  inline void AddView (const BundleView & view, size_t pointID) { points [pointID].AddView (view); }
  void SetCamera (const Eigen::Matrix3d & R, const Eigen::Vector3d & t, size_t id) { cameras [id].SetR (R); cameras [id].SetT (t); }
  void InvalidateCamera (size_t id) { cameras [id] = BundleCamera(); }
  void save (const std::string & filename) const;
};

#endif
