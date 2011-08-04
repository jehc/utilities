#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

int
main (int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cout << "Usage: " << argv[0] << " [in.jpg] [in.raw] [out.jpg] [out.raw]" << std::endl;
    return 1;
  }

  cv::Mat imageDist = cv::imread (argv[1]);
  std::ifstream rawIn (argv[2]);
  uint32_t rows, cols;
  rawIn.read((char*)&rows, sizeof(uint32_t));
  rawIn.read((char*)&cols, sizeof(uint32_t));
  cv::Mat rawDist (rows, cols, CV_32FC1);
  if (!rawIn.read((char*)rawDist.data, rows*cols*sizeof(float)))
  {
    std::cout << "Failed to read " << argv[2] << std::endl;
    return 1;
  }
  rawIn.close();

  cv::Mat image, raw;

  cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_32FC1);
  cameraMatrix.at<float>(0, 0) = 522.79548;
  cameraMatrix.at<float>(1, 1) = 522.78902;
  cameraMatrix.at<float>(0, 2) = 334.99980;
  cameraMatrix.at<float>(1, 2) = 251.89256;
  cameraMatrix.at<float>(2, 2) = 1;

  cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_32FC1);
  distCoeffs.at<float>(0, 0) = 0.24826;
  distCoeffs.at<float>(1, 0) = -0.82034;
  distCoeffs.at<float>(2, 0) = 0.00042;
  distCoeffs.at<float>(3, 0) = 0.00164;
  distCoeffs.at<float>(4, 0) = 0.98894;

  cv::Mat newCameraMatrix = cv::Mat::zeros(3, 3, CV_32FC1);
  newCameraMatrix.at<float>(0, 0) = 522;
  newCameraMatrix.at<float>(1, 1) = 522;
  newCameraMatrix.at<float>(0, 2) = 320;
  newCameraMatrix.at<float>(1, 2) = 240;
  newCameraMatrix.at<float>(2, 2) = 1;

  cv::undistort (imageDist, image, cameraMatrix, distCoeffs, newCameraMatrix);
  
  cv::Mat imageFlip;
  cv::flip (image, imageFlip, 1);
  cv::imwrite (argv[3], imageFlip);

  cv::Mat R, map1, map2;
  cv::initUndistortRectifyMap (cameraMatrix, distCoeffs, R, newCameraMatrix, rawDist.size(), CV_32FC1, map1, map2);
  cv::remap (rawDist, raw, map1, map2, cv::INTER_NEAREST);

  cv::Mat rawFlip;
  cv::flip (raw, rawFlip, 1);
  std::ofstream rawOut (argv[4]);
  rawOut.write ((char*)&rawFlip.rows, sizeof(uint32_t));
  rawOut.write ((char*)&rawFlip.cols, sizeof(uint32_t));
  rawOut.write ((char*)rawFlip.data, sizeof(float)*rawFlip.rows*rawFlip.cols);
  rawOut.close ();
   
  return 0;
}
