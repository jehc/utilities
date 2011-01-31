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

#include "calibration_common.h"

#include <ntk/ntk.h>

using namespace ntk;
using namespace cv;

void show_corners(const cv::Mat3b& image, const std::vector<Point2f>& corners, int wait_time)
{
  cv::Mat3b debug_img;
  image.copyTo(debug_img);
  foreach_idx(i, corners)
  {
    Point2i p = corners[i];
    Rect r (p+Point2i(-2,-2),cv::Size(4,4));
    cv::rectangle(debug_img, r, Scalar(255,0,0,255));
    cv::circle(debug_img, p, 8, Scalar(0,0,255,255));
  }
  imshow("corners", debug_img);
  cv::waitKey(wait_time);
}

double computeError(const cv::Mat& F,
                    const std::vector<std::vector<Point2f> >& rgb_corners,
                    const std::vector<std::vector<Point2f> >& depth_corners)
{
  std::vector<cv::Point2f> points_in_rgb;
  for (int i = 0; i < rgb_corners.size(); ++i)
    for (int j = 0; j < rgb_corners[i].size(); ++j)
      points_in_rgb.push_back(rgb_corners[i][j]);

  std::vector<cv::Point2f> points_in_depth;
  for (int i = 0; i < depth_corners.size(); ++i)
    for (int j = 0; j < depth_corners[i].size(); ++j)
      points_in_depth.push_back(depth_corners[i][j]);

  std::vector<Vec3f> lines_in_depth;
  std::vector<Vec3f> lines_in_rgb;

  cv::computeCorrespondEpilines(cv::Mat(points_in_rgb), 1, F, lines_in_depth);
  cv::computeCorrespondEpilines(cv::Mat(points_in_depth), 2, F, lines_in_rgb);

  double avgErr = 0;
  for(int i = 0; i < points_in_rgb.size(); ++i)
  {
    double err = fabs(points_in_rgb[i].x*lines_in_rgb[i][0] +
                      points_in_rgb[i].y*lines_in_rgb[i][1] + lines_in_rgb[i][2]);
    avgErr += err;
  }

  for(int i = 0; i < points_in_depth.size(); ++i)
  {
    double err = fabs(points_in_depth[i].x*lines_in_depth[i][0] +
                      points_in_depth[i].y*lines_in_depth[i][1] + lines_in_depth[i][2]);
    avgErr += err;
  }

  return avgErr / (points_in_rgb.size() + points_in_depth.size());
}

void estimate_checkerboard_pose(const std::vector<Point3f>& model,
                                const std::vector<Point2f>& img_points,
                                const cv::Mat1d& calib_matrix,
                                cv::Mat1f& H)
{
  cv::Mat1f to_open_cv (4,4);
  setIdentity(to_open_cv);
  to_open_cv(1,1) = -1;
  to_open_cv(2,2) = -1;
  cv::Mat1f from_open_cv = to_open_cv.inv();

  Mat3f model_mat(model.size(), 1); CvMat c_model_mat = model_mat;
  for_all_rc(model_mat) model_mat(r, 0) = Vec3f(model[r].x, -model[r].y, -model[r].z);

  // First image, for model pose.

  Mat2f point_mat(img_points.size(), 1); CvMat c_point_mat = point_mat;
  for_all_rc(point_mat) point_mat(r, 0) = Vec2f(img_points[r].x, img_points[r].y);

  Mat1f rvec (3,1); rvec = 0; CvMat c_rvec = rvec;
  Mat1f tvec (3,1); tvec = 0; CvMat c_tvec = tvec;

  CvMat c_calib_mat = calib_matrix;
  cvFindExtrinsicCameraParams2(&c_model_mat,
                               &c_point_mat,
                               &c_calib_mat,
                               0, &c_rvec, &c_tvec);

  cv::Mat1f rot(3,3); CvMat c_rot = rot;
  cvRodrigues2(&c_rvec, &c_rot);

  H = cv::Mat1f(4,4);
  setIdentity(H);
  cv::Mat1f H_rot = H(Rect(0,0,3,3));
  rot.copyTo(H_rot);
  H(0,3) = tvec(0,0);
  H(1,3) = tvec(1,0);
  H(2,3) = tvec(2,0);
  ntk_dbg_print(H, 1);

  H = from_open_cv * H * to_open_cv;
}

void kinect_shift_ir_to_depth(cv::Mat3b& im)
{
  imwrite("/tmp/before.png", im);
  cv::Mat1f t (2, 3);
  t = 0.f;
  t(0,0) = 1;
  t(1,1) = 1;
  t(0,2) = -4.8;
  t(1,2) = -3.9;
  cv::Mat3b tmp;
  warpAffine(im, tmp, t, im.size());
  im = tmp;
  imwrite("/tmp/after.png", im);
}
