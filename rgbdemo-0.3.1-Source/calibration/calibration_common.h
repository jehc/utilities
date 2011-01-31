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

#ifndef CALIBRATION_COMMON_H
#define CALIBRATION_COMMON_H

# include <ntk/core.h>
# include <vector>

void estimate_checkerboard_pose(const std::vector<cv::Point3f>& model,
                                const std::vector<cv::Point2f>& img_points,
                                const cv::Mat1d& calib_matrix,
                                cv::Mat1f& H);
double computeError(const cv::Mat& F,
                    const std::vector<std::vector<cv::Point2f> >& rgb_corners,
                    const std::vector<std::vector<cv::Point2f> >& depth_corners);

void show_corners(const cv::Mat3b& image, const std::vector<cv::Point2f>& corners, int wait_time = 10);

// Apply translation coeffs, see http://www.ros.org/wiki/kinect_calibration/technical
void kinect_shift_ir_to_depth(cv::Mat3b& im);

#endif // CALIBRATION_COMMON_H
