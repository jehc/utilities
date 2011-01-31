/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#include "plane.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/eigen_utils.h>
#include <Eigen/Geometry>

using namespace cv;

namespace ntk
{

void orthogonal_basis(Vec3f& v1, Vec3f& v2, const Vec3f& v0)
{
  Eigen::Vector3d ev0 = toEigenVector3d(v0);
  Eigen::Vector3d eorth1 = ev0.unitOrthogonal();
  v1 = toVec3f(eorth1);
  v2 = v0.cross(v1);
  normalize(v1);
  normalize(v2);
}

Point3f Plane :: intersectionWithLine (const Point3f& p1, const Point3f& p2) const
{
  double u = a*p1.x + b*p1.y + c*p1.z + d;
  u /= a*(p1.x-p2.x) + b*(p1.y-p2.y) + c*(p1.z-p2.z);
  Point3f r = p1 + u * (p2-p1);
  return r;
}

}
