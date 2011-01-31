#include "BundleCamera.h"

#include <exception>

istream & operator>> (istream & in, BundleCamera & camera)
{
  if (!(in >> camera.f >> camera.k1 >> camera.k1))
  {
    throw std::exception();
  }

  camera.R = cv::Mat (3, 3, CV_32FC1);
  for (int j = 0; j < camera.R.rows; ++j)
  {
    for (int i = 0; i < camera.R.cols; ++i)
    {
      if (!(in >> camera.R.at<float>(j, i)))
      {
        throw std::exception();
      }
    }
  }

  camera.t = cv::Mat (3, 1, CV_32FC1);
  for (int j = 0; j < camera.t.rows; ++j)
  {
    if (!(in >> camera.t.at<float>(j, 0)))
    {
      throw std::exception();
    }
  }
  camera.valid = camera.f > 0;
  return in;
}
