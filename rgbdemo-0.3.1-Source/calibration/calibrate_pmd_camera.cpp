//
// Author: Jorge Garcia Bueno <jgbueno@ing.uc3m.es>, (C) 2010
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <ntk/ntk.h>
#include <ntk/camera/calibration.h>
#include <opencv/cv.h>

#include <QDir>
#include <QDebug>

using namespace ntk;
using namespace cv;

// example command line (for copy-n-paste):
// calibrate_one_camera -w 8 -h 6 -o camera.yml images

namespace global
{
  arg<const char*> opt_image_directory(0, "Image directory", 0);
  arg<int> opt_pattern_width("-w", "Pattern width (number of inner squares)", 8);
  arg<int> opt_pattern_height("-h", "Pattern height (number of inner squares)", 6);
  arg<float> opt_square_size("-s", "Square size in used defined scale", 1.0);
  arg<const char*> opt_output_file("-o", "Output YAML filename", "pmd_calibration.yml");
}

int main(int argc, char** argv)
{
  arg_base::set_help_option("--help");
  arg_parse(argc, argv);
  ntk::ntk_debug_level = 1;

  QDir image_dir(global::opt_image_directory());
  ntk_ensure(image_dir.exists(), (image_dir.absolutePath() + " is not a directory.").toAscii());

  std::vector< std::vector<Point2f> > corners;

  QStringList file_list = image_dir.entryList(QStringList("view????"), QDir::Dirs, QDir::Name);

  namedWindow("corners", 1);

  // First loop for distorsion coeffs.
  cv::Mat image;
  foreach (QString filename, file_list)
  {
    QDir cur_image_dir (image_dir.absoluteFilePath(filename));
    std::string full_filename = cur_image_dir.absoluteFilePath("raw/intensity.png").toStdString();
    ntk_dbg_print(full_filename, 1);
    image = imread(full_filename);
    //erode(image, image, getStructuringElement(MORPH_RECT, Size(5,5)));
    if (!image.data)
    {
      ntk_dbg(1) << "Could not load image, skipping!";
      continue;
    }
    preprocess_pmd(image);
    std::vector<Point2f> view_corners;
    calibrationCorners(full_filename, "corners",
                       global::opt_pattern_width(), global::opt_pattern_height(),
                       view_corners, image, 1);

    if (view_corners.size() == global::opt_pattern_height()*global::opt_pattern_width())
      corners.push_back(view_corners);
  }
  ntk_dbg_print(corners.size(), 1);
  std::vector<Mat> rvecs, tvecs; // not used.
  std::vector< std::vector<Point3f> > pattern_points;
  calibrationPattern(pattern_points,
                     global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                     corners.size());
  cv::Mat matrix, dist;
  calibrateCamera(pattern_points, corners, image.size(),
                  matrix, dist,
                  rvecs, tvecs);

#if 0
  // Second loop with resized images.
  corners.clear();
  foreach (QString filename, file_list)
  {
    QDir cur_image_dir (image_dir.absoluteFilePath(filename));
    std::string full_filename = cur_image_dir.absoluteFilePath("raw/intensity.png").toStdString();
    ntk_dbg_print(full_filename, 1);
    image = imread(full_filename);
    if (!image.data)
    {
      ntk_dbg(1) << "Could not load image, skipping!";
      continue;
    }
    preprocess_pmd(image);
    std::vector<Point2f> view_corners;
    calibrationCorners(full_filename, "corners",
                       global::opt_pattern_width(), global::opt_pattern_height(),
                       view_corners, image, 1, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (view_corners.size() == global::opt_pattern_height()*global::opt_pattern_width())
      corners.push_back(view_corners);
  }
  ntk_dbg_print(corners.size(), 1);

  calibrationPattern(pattern_points,
                     global::opt_pattern_width(),  global::opt_pattern_height(), global::opt_square_size(),
                     corners.size());
  ntk_dbg_print((Mat1d)dist, 1);
  cv::Mat old_dist = dist.clone();
  calibrateCamera(pattern_points, corners, image.size(),
                  matrix, dist,
                  rvecs, tvecs /*, CV_CALIB_ZERO_TANGENT_DIST*/);
  ntk_dbg_print((Mat1d)dist, 1);
  // dist = old_dist;
#endif

  cv::Mat undistorted;
#if 0
  cv::Mat new_matrix = getOptimalNewCameraMatrix(matrix, dist,
                                                 image.size(), 1.0, Size(800,800), 0);
#else
  cv::Mat new_matrix = matrix;
#endif
//  new_matrix.at<double>(0,2) = 400;
//  new_matrix.at<double>(1,2) = 400;
  cv::undistort(image, undistorted, matrix, dist, new_matrix);
//  cv::Mat new_matrix = getOptimalNewCameraMatrix(matrix, dist,
//                                                 image.size(), 1.0, Size(), 0);
//  cv::undistort(image, undistorted, matrix, dist, new_matrix);

  FileStorage output_file (image_dir.absoluteFilePath(global::opt_output_file()).toStdString(),
                           CV_STORAGE_WRITE);
  writeMatrix(output_file, "matrix", matrix);
  writeMatrix(output_file, "optimal_matrix_after_undistort", new_matrix);
  writeMatrix(output_file, "dist", dist);
  output_file.release();

  return 0;
}
