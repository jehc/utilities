#include "BundleFile.h"
#include <iostream>
#include <fstream>
#include <vector>

float
depthFromYml (const std::vector<std::string> & depthFiles, const BundleView & view, std::vector<cv::Mat> & depthCache, std::vector<bool> & cached)
{
  const std::string & depthFile = depthFiles [view.GetCamera()];
  if (!cached [view.GetCamera()])
  {
    IplImage * tmp  = (IplImage *)cvLoad (depthFile.c_str());
    depthCache [view.GetCamera()] = cv::Mat (tmp);
    cached [view.GetCamera()] = true;
  }

  const cv::Mat & map = depthCache [view.GetCamera()];
  
//  std::cerr << "--- " << 240 - view.GetY() << " " << view.GetX() + 320 << std::endl;
  assert (view.GetY() <= map.rows/2 && view.GetY() >= -map.rows/2);
  assert (view.GetX() <= map.cols/2 && view.GetX() >= -map.cols/2);
  float result = map.at<float>(map.rows/2 - view.GetY(), view.GetX() + map.cols/2);

  return result;
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
    filename.replace (filename.find (".color.calib.jpg"), 16, ".depth.calib.yml");
    depthFiles.push_back (filename);
  }
  list.close();

  const std::vector<BundleCamera> & cameras = bundle.GetCameras();
  const std::vector<BundlePoint> & points = bundle.GetPoints();

  depthCache.resize (cameras.size());
  cached.resize (cameras.size());
  for (std::vector<BundlePoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    const std::vector<BundleView> views = i->GetViews();
    for (std::vector<BundleView>::const_iterator j = views.begin(); j != views.end(); ++j)
    {
      assert (j->GetCamera() < depthFiles.size());
      assert (j->GetCamera() < cameras.size());
      // Fine the depth according to the uncalibrated data
      float depthGuess = depthFromYml (depthFiles, *j, depthCache, cached);

      if (depthGuess == 0)
      {
        continue;
      }
      // Get distance from camera to point
      float depthCorrect = depthFromBundle (cameras, *i, *j);
      std::cerr << depthGuess << " " << depthCorrect << std::endl;
      dataPoints.push_back (std::make_pair<float, float> (depthGuess, depthCorrect));
    }
  }  
}
