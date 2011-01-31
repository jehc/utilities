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
 * Author: Jorge Garcia Bueno <jgarcia@ing.uc3m.es>, (C) 2010
 */

#ifndef NTK_DETECTION_PEDESTRIANSDETECTOR_H
#define NTK_DETECTION_PEDESTRIANSDETECTOR_H

#include <ntk/camera/calibration.h>

namespace ntk
{

  class PedestriansDetector
  {
  public:
    struct Detection
    {
      cv::Rect face_rectangle;
      cv::Rect body_rectangle;
    };

  public:
    void detect(std::vector<Detection>& detections,
                cv::Mat3b& output_image,
                const RGBDImage& image);
  };

} // ntk

#endif // NTK_DETECTION_PEDESTRIANSDETECTOR_H
