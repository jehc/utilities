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

#include "silhouette_rgbd_modeler.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/geometry/pose_3d.h>
#include <ntk/numeric/levenberg_marquart_minimizer.h>
#include <ntk/utils/time.h>

using namespace cv;

namespace ntk
{

  void SilhouetteRGBDModeler :: initialize(const cv::Point3f& sizes,
                                           const cv::Point3f& offsets,
                                           float resolution,
                                           float depth_margin)
  {
    m_resolution = resolution;
    m_depth_margin = depth_margin;
    m_offsets = offsets;
    m_sizes = sizes;
    const int c_sizes[3] = {sizes.z/m_resolution, sizes.y/m_resolution, sizes.x/m_resolution};
    m_voxels.create(3, c_sizes);
    for_all_drc(m_voxels) m_voxels(d,r,c) = 1u;
  }

  void SilhouetteRGBDModeler :: computeMesh()
  {
    m_mesh.clear();
    for_all_drc(m_voxels)
    {
      if (!m_voxels(d,r,c))
        continue;

      Point3f p = toRealWorld(Point3f(c,r,d));
      float size_x = m_resolution;
      for (++c; c < m_voxels.size[2] && m_voxels(d,r,c); ++c)
      {
        size_x += m_resolution;
        p.x += m_resolution/2.0;
      }
      m_mesh.addCube(p, Point3f(size_x,m_resolution,m_resolution));
    }
  }

  void SilhouetteRGBDModeler :: computeSurfaceMesh()
  {
    m_mesh.clear();
    for_all_drc(m_voxels)
    {
      if (!m_voxels(d,r,c))
        continue;

      bool is_edge = false;
      for (int dd=-1; !is_edge && dd<=1; ++dd)
      for (int dr=-1; !is_edge && dr<=1; ++dr)
      for (int dc=-1; !is_edge && dc<=1; ++dc)
      {
        if (!m_voxels(d+dd,r+dr,c+dc))
          is_edge = true;
      }
      if (!is_edge)
        continue;

      Point3f p = toRealWorld(Point3f(c,r,d));
      float size_x = m_resolution;
      m_mesh.addCube(p, Point3f(size_x,m_resolution,m_resolution));
    }
  }

  Point3f SilhouetteRGBDModeler :: toRealWorld(const Point3f& p) const
  {
    return Point3f(p.x*m_resolution + (m_resolution/2.0) + m_offsets.x,
                   p.y*m_resolution + (m_resolution/2.0) + m_offsets.y,
                   p.z*m_resolution + (m_resolution/2.0) + m_offsets.z);
  }

  void SilhouetteRGBDModeler :: addNewView(const RGBDImage& image, Pose3D& relative_pose)
  {
    Pose3D rgb_pose = *image.calibration()->rgb_pose;
    Pose3D depth_pose = *image.calibration()->depth_pose;
    depth_pose.applyTransformBefore(relative_pose);
    rgb_pose.applyTransformBefore(relative_pose);

    Pose3D world_to_camera_normal_pose;
    world_to_camera_normal_pose.applyTransformBefore(Vec3f(0,0,0), relative_pose.cvEulerRotation());
    Pose3D camera_to_world_normal_pose = world_to_camera_normal_pose; camera_to_world_normal_pose.invert();

    const Mat1f& depth_im = image.depth();

    for_all_drc(m_voxels)
    {
      if (!m_voxels(d,r,c))
        continue;

      Point3f p2d = depth_pose.projectToImage(toRealWorld(Point3f(c,r,d)));

      bool to_remove = true;
      int dpix = (depth_pose.meanFocal() * m_resolution) / p2d.z;
      dpix = std::max(dpix, 1);

      for (int dr=-dpix; dr<=dpix; ++dr)
      for (int dc=-dpix; dc<=dpix; ++dc)
      {
        if (!is_yx_in_range(depth_im, p2d.y+dr, p2d.x+dc))
          continue;
        if (!image.depthMask()(p2d.y+dr,p2d.x+dc))
          continue;
        double tof_depth = depth_im(p2d.y+dr, p2d.x+dc);
        if (tof_depth < (p2d.z+m_depth_margin))
        {
          to_remove = false;
          break;
        }
      }

      if (to_remove)
      {
        m_voxels(d,r,c) = 0;
      }
    }
  }

  void SilhouetteRGBDModeler :: reset()
  {
    for_all_drc(m_voxels)
        m_voxels(d,r,c) = 1;
    RGBDModeler::reset();
  }

} // ntk
