#ifndef BUNDLE_CAMERA_H
#define BUNDLE_CAMERA_H

#include <istream>
#include <Eigen/Dense>

class BundleCamera
{
  double f, k1, k2;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  friend std::istream & operator>> (std::istream &, BundleCamera &);
  friend std::ostream & operator<< (std::ostream &, const BundleCamera &);
public:
  BundleCamera():f(0),k1(0),k2(0){R.setZero(); t.setZero();}
  inline double GetF() const { return f; }
  inline double GetK1() const { return k1; }
  inline double GetK2() const { return k2; }
  inline const Eigen::Matrix3d & GetR() const { return R; }
  inline const Eigen::Vector3d & GetT() const { return t; }
  inline void SetR(const Eigen::Matrix3d & newR) { R = newR; }
  inline void SetT(const Eigen::Vector3d & newT) { t = newT; }
  inline bool IsValid() const { return f > 0; }
};

#endif
