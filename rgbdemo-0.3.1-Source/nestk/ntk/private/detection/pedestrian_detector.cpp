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

#include "pedestrian_detector.h"
#include <ntk/utils/opencv_utils.h>
#include <ntk/utils/stl.h>
#include <ntk/geometry/pose_3d.h>

using namespace cv;

namespace ntk
{

  void PedestriansDetector :: detect(std::vector<Detection>& detections,
                                     cv::Mat3b& output_image,
                                     const RGBDImage& image)
  {
    const int min_rect_area = 300;
    const double haar_scale_factor = 1.3;
    const static Scalar people_colors[] =  { CV_RGB(0,0,255),
                                             CV_RGB(0,128,255),
                                             CV_RGB(0,255,255),
                                             CV_RGB(0,255,0),
                                             CV_RGB(255,128,0),
                                             CV_RGB(255,255,0),
                                             CV_RGB(255,0,0),
                                             CV_RGB(255,0,255)} ;

    const cv::Mat3b& rgb_image = image.rgb();
    rgb_image.copyTo(output_image);
    const cv::Mat1f& distance_image = image.depth();
    cv::Mat1b gray_image = image.rgbAsGray();
    ntk_ensure(gray_image.data, "Gray image required!");
    cv::Mat1b mask_image = image.depthMask().clone();

    cv::morphologyEx(mask_image, mask_image,
                     cv::MORPH_OPEN,
                     getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(3,3)));
    cv::morphologyEx(mask_image, mask_image,
                     cv::MORPH_CLOSE,
                     getStructuringElement(cv::MORPH_RECT,
                                           cv::Size(3,3)));

    std::vector< std::vector<cv::Point> > candidate_contours;
    cv::Mat1b contour_image = mask_image.clone();
    cv::findContours(contour_image, candidate_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::vector<cv::Rect> candidate_rects;
    for (int i = 0; i < candidate_contours.size(); ++i)
    {
      cv::Rect rect = cv::boundingRect(cv::Mat(candidate_contours[i]));
      if (rect.area() < min_rect_area)
        continue;

      Point3f top_left_3d = image.calibration()->depth_pose->unprojectFromImage(
          Point2f(rect.x,rect.y),
          distance_image((rect.x+ rect.width)/2,
                         (rect.y+ rect.height)/2)
          );
      Point3f bottom_right_3d = image.calibration()->depth_pose->unprojectFromImage(Point2f(rect.x+ rect.width,rect.y+ rect.height), distance_image((rect.x+ rect.width)/2,(rect.y+ rect.height)/2));

      Point3f top_left_rgb = image.calibration()->rgb_pose->projectToImage(top_left_3d);
      Point3f bottom_right_rgb = image.calibration()->rgb_pose->projectToImage(bottom_right_3d);

      rectangle(output_image, cv::Point(top_left_rgb.x,top_left_rgb.y), cv::Point(bottom_right_rgb.x,bottom_right_rgb.y),CV_RGB(255,0,0),2);
      cv::Rect r (cv::Point(top_left_rgb.x,top_left_rgb.y), cv::Point(bottom_right_rgb.x,bottom_right_rgb.y));
      adjustRectToImage(r, rgb_image.size());
      candidate_rects.push_back(r);
    }


    cv::CascadeClassifier cascade;
    //cascade.load( string("&cascade_frontalface_alt2.xml") );
    ntk_ensure(is_file("haarcascade_frontalface_alt2.xml"), "Cascade file not available!");
    cascade.load( std::string("haarcascade_frontalface_alt2.xml") );

    std::vector<cv::Rect> haar_detections;
    for( int i_cand = 0; i_cand < candidate_rects.size(); ++i_cand)
    {
      gray_image.adjustROI(candidate_rects.at(i_cand).x, candidate_rects.at(i_cand).y, candidate_rects.at(i_cand).height,candidate_rects.at(i_cand).width);
      cascade.detectMultiScale( gray_image, haar_detections,
                                1.1, 2, 0
                                //|CV_HAAR_FIND_BIGGEST_OBJECT
                                //|CV_HAAR_DO_ROUGH_SEARCH
                                |CV_HAAR_SCALE_IMAGE,
                                Size(30, 30) );
      for (int i_detections = 0; i_detections < haar_detections.size(); ++i_detections)
      {
        const cv::Rect& r = haar_detections[i_detections];
        Detection detection;
        detection.face_rectangle = cv::Rect(r.x,
                                            r.y,
                                            r.width*haar_scale_factor,
                                            r.height*haar_scale_factor);

        Scalar color = people_colors[i_detections%8];
        rectangle(output_image, detection.face_rectangle, color, 2);

        cv::Point2i centerface = Point( (r.x*1 + r.width*haar_scale_factor + candidate_rects.at(i_cand).x)/2,
                                        (r.y*1 + r.height*haar_scale_factor + candidate_rects.at(i_cand).y)/2);

        // check if the face center is into the candidate rectangle.
        bool face_center_inside_candidate_rect =
            (centerface.x > candidate_rects.at(i_cand).x)
            && (centerface.x <  candidate_rects.at(i_cand).x+candidate_rects.at(i_cand).width)
            && (centerface.y > candidate_rects.at(i_cand).y)
            && (centerface.y < candidate_rects.at(i_cand).y+candidate_rects.at(i_cand).height);

        if (face_center_inside_candidate_rect)
        {
          Point top_left = Point(candidate_rects.at(i_cand).x, candidate_rects.at(i_cand). y);
          Point bottom_right = Point(candidate_rects.at(i_cand).x + candidate_rects.at(i_cand).width,
                                     candidate_rects.at(i_cand). y + candidate_rects.at(i_cand).height);
          detection.body_rectangle = Rect(top_left, bottom_right);
          detections.push_back(detection);
          rectangle(output_image, detection.body_rectangle, CV_RGB(0,255,0), 2);
        }
      }
    }//end i_cand
  }

} // ntk
