#include "BundleFile.h"
#include <iostream>
#include <fstream>
#include <vector>

float
depthFromRaw (const std::vector<std::string> & depthFiles, const BundleView & view, std::vector<std::vector<float> > & depthCache, std::vector<bool> & cached)
{
  const std::string & depthFile = depthFiles [view.GetCamera()];
  if (!cached [view.GetCamera()])
  {
    if (depthFile == "")
    {
      return 0;
    }
    std::ifstream input (depthFile.c_str());
    if (!input)
    {
      std::cerr << "Failed to open file " << depthFile << std::endl;
      throw std::exception();
    }
    std::vector<float> depthBuffer (640*480);
    if (!input.read((char*)&depthBuffer[0], 640*480*sizeof(float)))
    {
      std::cerr << "Failed to read file " << depthFile << std::endl;
      throw std::exception();
    }
    depthCache [view.GetCamera()] = depthBuffer;
    cached [view.GetCamera()] = true;
  }

  const std::vector<float> & map = depthCache [view.GetCamera()];
  
  int indexI = (int)(view.GetX() + 320);
  int indexJ = (int)(240 - view.GetY());
  return map[indexJ*640 + indexI];
}

float
depthFromBundle (const std::vector<BundleCamera> & cameras, const BundlePoint & point, const BundleView & view)
{
  const BundleCamera & camera = cameras [view.GetCamera()];
  assert (camera.IsValid());
  const cv::Mat & R = camera.GetR();
  const cv::Mat & t = camera.GetT();
  cv::Mat p = cv::Mat(point.GetPosition());
  cv::Mat center = -R.inv() * t;
  cv::Mat b = p - center;
  cv::Mat z = cv::Mat(cv::Vec3f(0, 0, -1));
  z = R.inv() * z;
  return z.at<float>(0, 0)*b.at<float>(0, 0) + z.at<float>(1, 0)*b.at<float>(1, 0) + z.at<float>(2, 0)*b.at<float>(2, 0);;
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
  std::vector<std::vector<float> > depthCache;
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

  depthCache.resize (cameras.size());
  cached.resize (cameras.size());
  float num = 0;
  float den = 0;
  float mean = 0;
  float var = 0;
  int n = 0;
  for (std::vector<BundlePoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    const std::vector<BundleView> views = i->GetViews();
    for (std::vector<BundleView>::const_iterator j = views.begin(); j != views.end(); ++j)
    {
      assert (j->GetCamera() < depthFiles.size());
      assert (j->GetCamera() < cameras.size());
      // Fine the depth according to the uncalibrated data
      float depthGuess = depthFromRaw (depthFiles, *j, depthCache, cached);
      if (depthGuess == 0)
      {
        continue;
      }
      depthGuess = -0.018*depthGuess*depthGuess + 1.0038*depthGuess + 0.0050;
      // Get distance from camera to point
      float depthCorrect = depthFromBundle (cameras, *i, *j);
      std::cerr << j->GetCamera() << " " << depthGuess << " " << depthCorrect << std::endl;
      num += depthCorrect*depthGuess;
      den += depthGuess*depthGuess;
      ++n;
      mean += depthCorrect/depthGuess;
      var += (depthCorrect*depthCorrect)/(depthGuess*depthGuess);
      dataPoints.push_back (std::make_pair<float, float> (depthGuess, depthCorrect));
    }
  }
  float scale = num/den;
  mean /= n;
  var /= n;
  var -= mean*mean;
  float min = scale - sqrt(var);
  float max = scale + sqrt(var);
  num = 0;
  den = 0;
  for (size_t i = 0; i < dataPoints.size(); ++i)
  {
    const std::pair<float, float> & p = dataPoints[i];
    float ratio = p.second/p.first;
    if (ratio > min && ratio < max)
    {
      num += p.first*p.second;
      den += p.first*p.first;
    }
  }  
  std::cout << num/den << std::endl;
}
