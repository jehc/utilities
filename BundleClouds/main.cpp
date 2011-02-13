#include "BundleFile.h"

#include <fstream>
#include <iostream>
#include <cassert>
#include <string>

std::vector<BundlePoint> LoadCloud (const std::string & colorFilename, const std::string & depthFilename, const BundleCamera & camera, int scale)
{
  std::vector<BundlePoint> points;

  cv::Mat colorImageDist = cv::imread (colorFilename);

  IplImage * tmp = (IplImage *)cvLoad (depthFilename.c_str());
  cv::Mat depthMapDist (tmp);

  cv::Mat cameraMatrix (3, 3, CV_32FC1);
  cameraMatrix.at<float> (0, 0) = camera.GetF();
  cameraMatrix.at<float> (0, 1) = 0;
  cameraMatrix.at<float> (0, 2) = (float)colorImageDist.cols/2;
  cameraMatrix.at<float> (1, 0) = 0;
  cameraMatrix.at<float> (1, 1) = camera.GetF();
  cameraMatrix.at<float> (1, 2) = (float)colorImageDist.rows/2;
  cameraMatrix.at<float> (2, 0) = 0;
  cameraMatrix.at<float> (2, 1) = 0;
  cameraMatrix.at<float> (2, 2) = 1;

  cv::Mat distCoeffs (5, 1, CV_32FC1);
  distCoeffs.at<float> (0, 0) = camera.GetK1();
  distCoeffs.at<float> (1, 0) = camera.GetK2();
  std::cerr << camera.GetK1() << " " << camera.GetK2() << std::endl;
  distCoeffs.at<float> (2, 0) = 0;
  distCoeffs.at<float> (3, 0) = 0;
  distCoeffs.at<float> (4, 0) = 0;

  cv::Mat depth2 (depthMapDist.rows, depthMapDist.cols, CV_32FC1);
  for (int j = 0; j < depth2.rows; ++j)
  {
    for (int i = 0; i < depth2.cols; ++i)
    {
      if (depthMapDist.at<float>(j, i) < 0.001)
      {
        depth2.at<float>(j, i) = 100000;
      }
      else
      {
        depth2.at<float>(j, i) = depthMapDist.at<float>(j, i);
      }
    }
  }

  cv::Mat colorImage, depthMap, depthMap2;

  cv::undistort (colorImageDist, colorImage, cameraMatrix, distCoeffs);
  cv::undistort (depthMapDist, depthMap, cameraMatrix, distCoeffs);
  cv::undistort (depth2, depthMap2, cameraMatrix, distCoeffs);

  cvReleaseImage (&tmp);

  for (int j = 0; j < depthMap.rows; ++j)
  {
    for (int i = 0; i < depthMap.cols; ++i)
    {
      float depth = depthMap.at<float> (j, i);
      if (depth != depthMap2.at<float>(j, i))
      {
        continue;
      }

      cv::Mat p (cv::Vec4f((float)i, (float)j, depth, 1));
      cv::Mat r (cv::Vec4f(scale*p.at<float>(0, 0)*p.at<float>(2, 0), 
                           scale*p.at<float>(1, 0)*p.at<float>(2, 0), 
                           scale*p.at<float>(2, 0), 1));

      cv::Mat intrinsics = cv::Mat::eye(4, 4, CV_32FC1);
      for (int y = 0; y < cameraMatrix.rows; ++y)
      {
        for (int x = 0; x < cameraMatrix.cols; ++x)
        {
          intrinsics.at<float>(y, x) = cameraMatrix.at<float>(y, x);
        }
      }

	  cv::Mat toOpencv = cv::Mat::eye (4, 4, CV_32FC1);
      toOpencv.at<float>(1, 1) = toOpencv.at<float>(2, 2) = -1;

      cv::Mat cameraTransform = cv::Mat::eye(4, 4, CV_32FC1);
      const cv::Mat & R = camera.GetR();
      const cv::Mat & t = camera.GetT();
	/*  std::cerr << "---" << std::endl;
	  for (int y = 0; y < 3; ++y)
	  {
		  for (int x = 0; x < 3; ++x)
		  {
			  std::cerr << Rinv.at<float>(y, x) << " ";
		  }
		  std::cerr << std::endl;
	  }*/
      cv::Mat z = R * t;
      for (int y = 0; y < R.rows; ++y)
      {
        for (int x = 0; x < R.cols; ++x)
        {
          cameraTransform.at<float>(y, x) = R.at<float>(y, x);
        }
        cameraTransform.at<float>(y, R.cols) = t.at<float>(y, 0);
      }

      cv::Mat projectTransform = intrinsics * toOpencv * cameraTransform;

      p = projectTransform.inv() * r;
      p /= p.at<float>(3, 0);

      BundlePoint point (cv::Vec3f(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0)), colorImage.at<cv::Vec3b> (j, i));
      points.push_back (point);
    }
  }
  return points;
}

void AddCloud (std::vector<BundlePoint> & finalPoints, const std::vector<BundlePoint> & points)
{
  finalPoints.reserve (finalPoints.size() + points.size());
  for (std::vector<BundlePoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    finalPoints.push_back (*i);
  }
}

void SavePly (const std::string & filename, const std::vector<BundlePoint> & points)
{
  std::ofstream output (filename.c_str());

  output << "ply" << std::endl;
  output << "format ascii 1.0" << std::endl;
  output << "element face 0" << std::endl;
  output << "property list uchar int vertex_indices" << std::endl;
  output << "element vertex " << points.size() << std::endl;
  output << "property float x" << std::endl;
  output << "property float y" << std::endl;
  output << "property float z" << std::endl;
  output << "property uchar diffuse_blue" << std::endl;
  output << "property uchar diffuse_green" << std::endl;
  output << "property uchar diffuse_red" << std::endl;
  output << "end_header" << std::endl;

  for (std::vector<BundlePoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    output << *i << std::endl;
  }

  output.close();
}

int
main (int argc, char ** argv)
{
  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << " [bundle.out] [list.txt] [output] {depth_tuning}" << std::endl;
    return -1;
  }

  int scale = 1;
  if (argc == 5)
  {
    scale = atoi (argv[4]);
  }

  BundleFile file (argv[1]);
  const std::vector<BundleCamera> cameras = file.GetCameras();
  std::ifstream images (argv[2]);

  std::vector<BundlePoint> finalPoints;

  int index = 0;
  while (images)
  {
    std::string filename, line;
    if (!(images >> filename))
    {
      break;
    }
    getline (images, line);
    if (!cameras[index].IsValid())
    {
      ++index;
      continue;
    }
    size_t replacement = filename.find (".color.calib.jpg");
    std::string filename2 (filename);
    filename2.replace (replacement, 16, ".depth.calib.yml");
    std::vector<BundlePoint> points = LoadCloud (filename, filename2, cameras[index], scale);
//    AddCloud (finalPoints, points);
    std::stringstream ss;
    ss << argv[3] << "/" << index << ".ply";
    SavePly (ss.str(), points);
    std::cerr << filename << " " << points.size() << "/" << finalPoints.size() << std::endl;
    ++index;
  }

  images.close();
 
//  SavePly (argv[3], finalPoints);

  return 0;
}
