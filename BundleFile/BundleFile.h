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
};

#endif