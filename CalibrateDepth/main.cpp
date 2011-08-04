#include "BundleFile.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <omp.h>

std::pair<float,float>
depthFromRaw (const std::vector<std::string> & depthFiles, const BundleView & view, std::vector<cv::Mat> & depthCache, std::vector<bool> & cached, const cv::Mat & a, const cv::Mat & b, const cv::Mat & c, const BundleCamera & camera)
{
  const std::string & depthFile = depthFiles [view.GetCamera()];
#pragma omp critical
  if (!cached [view.GetCamera()])
  {
    if (depthFile != "")
    {
    std::ifstream input (depthFile.c_str());
    if (!input)
    {
      std::cerr << "Failed to open file " << depthFile << std::endl;
      throw std::exception();
    }
    uint32_t rows, cols;
    input.read((char*)&rows, sizeof(uint32_t));
    input.read((char*)&cols, sizeof(uint32_t));
    cv::Mat depthBuffer (rows, cols, CV_32FC1);
    if (!input.read((char*)depthBuffer.data, 640*480*sizeof(float)))
    {
      std::cerr << "Failed to read file " << depthFile << std::endl;
      throw std::exception();
    }

    depthCache [view.GetCamera()] = depthBuffer;
    cached [view.GetCamera()] = true;
    }
  }
  bool found;
#pragma omp critical
  found = cached [view.GetCamera()];
  if (!found)
  {
    return std::make_pair(std::numeric_limits<float>::quiet_NaN(), 0);
  }
  const cv::Mat & map = depthCache [view.GetCamera()];
  int indexI = (int)(view.GetX() + 320);
  int indexJ = (int)(240 - view.GetY());
  assert (indexI >= 0 && indexI < 640 && indexJ >= 0 && indexJ < 480);
  Eigen::Matrix3f K;
  K << camera.GetF(), 0, 320,
       0, camera.GetF(), 240,
       0, 0, 1;
  Eigen::Matrix3f Kinv = K.inverse();
  float sum = 0;
  float sum_sq = 0;
  int n = 0;
  for (int i = std::max(indexI - 5, 0); i <= std::min(indexI + 5, 639); ++i)
  {
    for (int j = std::max(indexJ - 5, 0); j <= std::min(indexJ + 5, 479); ++j)
    {
      float depth = map.at<float>(j, i);
      if (depth == 0)
      {
        continue;
      }
      float value =  a.at<float>(j, i)*pow(depth, 2.0f) + b.at<float>(j, i)*depth + c.at<float>(j, i);
      if (value <= 0)
      {
        continue;
      }
      Eigen::Vector3f point((float)i, 479.0 - (float)j, 1);
      point = value * Kinv * point;
      float result = point.norm();
      sum += result;
      ++n;
    }
  }
  if (n < 2)
  {
    return std::make_pair(std::numeric_limits<float>::quiet_NaN(), 0);
  }
  float mean = sum/n;
  assert (mean >= 0);
  for (int i = std::max(indexI - 5, 0); i <= std::min(indexI + 5, 639); ++i)
  {
    for (int j = std::max(indexJ - 5, 0); j <= std::min(indexJ + 5, 479); ++j)
    {
      float depth = map.at<float>(j, i);
      if (depth == 0)
      {
        continue;
      }
      float value =  a.at<float>(j, i)*pow(depth, 2.0f) + b.at<float>(j, i)*depth + c.at<float>(j, i);
      if (value <= 0)
      {
        continue;
      }
      Eigen::Vector3f point((float)i, 479.0 - (float)j, 1);
      point = value * Kinv * point;
      float result = point.norm();
      sum_sq += pow(result - mean, 2.0f);
    }
  }
 
  float var = sum_sq/(n-1);
  assert (var > 0);
  return std::make_pair (mean, var);
}

float
depthFromBundle (const std::vector<BundleCamera> & cameras, const BundlePoint & point, const BundleView & view)
{
  const BundleCamera & camera = cameras [view.GetCamera()];
  assert (camera.IsValid());
  const Eigen::Matrix3f & R = camera.GetR();
  const Eigen::Vector3f & t = camera.GetT();
  Eigen::Vector3f p = point.GetPosition();
  Eigen::Vector3f center = -R.transpose() * t;
//  Eigen::Vector3f b = p - center;
//  Eigen::Vector3f z (0, 0, -1);
//  z = R.transpose() * z;
//  return z.dot(b);
  return (p - center).norm();
}

int
main (int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: " << argv[0] << " [bundle.out] [list.txt]" << std::endl;
  }

  BundleFile bundle (argv[1]);

  std::vector<std::pair<float, float> > dataPoints;
  std::vector<cv::Mat> depthCache;
  std::vector<bool> cached;

  std::string junk;
  std::vector<std::string> depthFiles;
  std::ifstream list (argv[2]);
  while (list)
  {
    std::string filename;
    if (!(list >> filename))
    {
      break;
    }
    getline (list, junk);
    size_t replacement = filename.find ("color.jpg");
    if (replacement == std::string::npos)
    {
      depthFiles.push_back ("");
      continue;
    }
    filename.replace (replacement, 9, "depth.raw");
    depthFiles.push_back (filename);
  }
  list.close();

  const std::vector<BundleCamera> & cameras = bundle.GetCameras();
  const std::vector<BundlePoint> & points = bundle.GetPoints();

  cv::Mat a (480, 640, CV_32FC1);
  cv::Mat b (480, 640, CV_32FC1);
  cv::Mat c (480, 640, CV_32FC1);
  std::ifstream ain ("/hydra/S2/kmatzen/09July2011_calib/a.txt");
  std::ifstream bin ("/hydra/S2/kmatzen/09July2011_calib/b.txt");
  std::ifstream cin ("/hydra/S2/kmatzen/09July2011_calib/c.txt");
  for (int i = 0; i < 640; ++i)
  {
    for (int j = 0; j < 480; ++j)
    { 
      ain >> a.at<float>(j, i);
      bin >> b.at<float>(j, i);
      cin >> c.at<float>(j, i);
      assert (ain && bin && cin);
    }
  }
  ain.close();
  bin.close();
  cin.close();

  std::vector<int> goodPoints (points.size());
#pragma omp parallel for
  for (int i = 0; i < points.size(); ++i)
  {
    const BundlePoint & point = points [i];
    const std::vector<BundleView> & views = point.GetViews();
    for (std::vector<BundleView>::const_iterator j = views.begin(); j != views.end(); ++j)
    {
      if (depthFiles [j->GetCamera()] == "")
      {
        ++goodPoints[i];
      }
    }
  }

  depthCache.resize (cameras.size());
  cached.resize (cameras.size());
#pragma omp parallel for
  for (int i = 0; i < points.size(); ++i)
  {
    if (goodPoints[i] < 4)
    {
      continue;
    }
    const BundlePoint & point = points [i];
    const std::vector<BundleView> & views = point.GetViews();
    for (std::vector<BundleView>::const_iterator j = views.begin(); j != views.end(); ++j)
    {
      assert (j->GetCamera() < depthFiles.size());
      assert (j->GetCamera() < cameras.size());
      // Fine the depth according to the uncalibrated data
      std::pair<float,float> depthGuess = depthFromRaw (depthFiles, *j, depthCache, cached, a, b, c, cameras[j->GetCamera()]);
      if (std::isnan(depthGuess.first) || std::isinf(depthGuess.first))
      {
        continue;
      }
      //depthGuess = -0.018*depthGuess*depthGuess + 1.0038*depthGuess + 0.0050;
      // Get distance from camera to point
      float depthCorrect = depthFromBundle (cameras, point, *j);
      if (depthCorrect < 0 || depthGuess.first < 0)
      {
        continue;
      }
      assert (depthGuess.second >= 0);
#pragma omp critical
      std::cout << i << " " << j->GetCamera() << " " << depthCorrect << " " << depthGuess.first << " " << depthGuess.second << " " << point.GetPosition().transpose() << " " << point.GetColor().transpose() << std::endl;
    }
  }
}
