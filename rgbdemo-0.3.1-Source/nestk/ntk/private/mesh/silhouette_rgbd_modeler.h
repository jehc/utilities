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

#ifndef NTK_MESH_SILHOUETTE_RGBD_MODELER_H
#define NTK_MESH_SILHOUETTE_RGBD_MODELER_H

#include <ntk/core.h>
#include <ntk/camera/calibration.h>
#include <ntk/mesh/mesh.h>
#include <ntk/mesh/rgbd_modeler.h>

namespace ntk
{

class SilhouetteRGBDModeler : public RGBDModeler
{
public:
  SilhouetteRGBDModeler() : RGBDModeler()
  {}

  void initialize(const cv::Point3f& sizes, const cv::Point3f& offsets,
                  float resolution, float depth_margin);

public:
  virtual void addNewView(const RGBDImage& image, Pose3D& relative_pose);
  virtual void computeMesh();
  virtual void computeSurfaceMesh();
  virtual void reset();
  virtual void setResolution(float resolution) { reset(); initialize(m_sizes, m_offsets, resolution, m_depth_margin); }
  virtual void setDepthMargin(float depth_margin) { m_depth_margin = depth_margin; }

private:
  cv::Point3f toRealWorld(const cv::Point3f& p) const;

private:
  cv::Mat_<uchar> m_voxels;
  float m_resolution;
  float m_depth_margin;
  cv::Point3f m_offsets;
  cv::Point3f m_sizes;
};

} // ntk

#endif // NTK_MESH_SILHOUETTE_RGBD_MODELER_H
