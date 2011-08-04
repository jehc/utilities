#ifndef BUNDLE_CAMERA_H
#define BUNDLE_CAMERA_H

#include <istream>
#include <Eigen/Dense>

class BundleCamera
{
  bool valid;
  float f, k1, k2;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;
  friend std::istream & operator>> (std::istream &, BundleCamera &);
  friend std::ostream & operator<< (std::ostream &, const BundleCamera &);
public:
  BundleCamera():valid(false),f(0),k1(0),k2(0){R.setZero(); t.setZero();}
  inline float GetF() const { return f; }
  inline float GetK1() const { return k1; }
  inline float GetK2() const { return k2; }
  inline const Eigen::Matrix3f & GetR() const { return R; }
  inline const Eigen::Vector3f & GetT() const { return t; }
  inline void SetR(const Eigen::Matrix3f & newR) { R = newR; }
  inline void SetT(const Eigen::Vector3f & newT) { t = newT; }
  inline bool IsValid() const { return valid; }
};

#endif
