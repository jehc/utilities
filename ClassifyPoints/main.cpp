#include <fstream>
#include <opencv2/opencv.hpp>
#include <iostream>

int
main (int argc, char ** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: " << argv[0] << " [depthMap.yml]" << std::endl;
    return 1;
  }
  IplImage * tmp = (IplImage*)cvLoad(argv[1]);
  cv::Mat depth (tmp);

  cv::Mat boundary (depth.rows, depth.cols, CV_8UC3);

  enum RegionType {Shadow, Veil, Object};

  for (size_t j = 0; j < boundary.rows; ++j)
  {
    for (size_t i = 0; i < boundary.cols; ++i)
    {
      boundary.at<cv::Vec3b>(j, i)[0] = boundary.at<cv::Vec3b>(j, i)[1] = boundary.at<cv::Vec3b>(j, i)[2] = 0;
      if (depth.at<float>(j, i) <= 0)
      {
        boundary.at<cv::Vec3b>(j, i)[Shadow] = 255;
      }
      else
      {
        boundary.at<cv::Vec3b>(j, i)[Object] = 255; 
      }
    }
  }

  cv::Mat mean (boundary.rows, boundary.cols, CV_32FC1);
  cv::Mat var (boundary.rows, boundary.cols, CV_32FC1);

  int window = 2;
  float sumVar = 0;
  int nVar = 0;
  for (size_t j = 0; j < boundary.rows; ++j)
  {
    for (size_t i = 0; i < boundary.cols; ++i)
    {
      if (boundary.at<cv::Vec3b>(j, i)[Shadow] == 255)
      {
        continue;
      }
      float sum = 0;
      float sum2 = 0;
      int n = 0;
      int left = MAX(0, (int)i - window);
      int right = MIN (boundary.cols - 1, (int)i + window);
      int top = MAX (0, (int)j - window);
      int bottom = MIN (boundary.rows - 1, (int)j + window);

      for (int x = left; x < right; ++x)
      {
        for (int y = top; y < bottom; ++y)
        {
          float horz = abs(depth.at<float>(y, x) - depth.at<float>(y, x + 1));
          float vert = abs(depth.at<float>(y, x) - depth.at<float>(y + 1, x));
          if (boundary.at<cv::Vec3b>(y, x + 1)[Shadow] == 0)
          {
            sum += horz;
            sum2 += horz*horz;
            ++n;
          }
          if (boundary.at<cv::Vec3b>(y + 1, x)[Shadow] == 0)
          {
            sum += vert;
            sum2 += vert*vert;
            ++n;
          }
        }
      }
     
      if (n)
      {
        mean.at<float>(j, i) = sum/n;
        var.at<float>(j, i) = sum2/n - pow (mean.at<float>(j, i), 2.0);
        sumVar += var.at<float>(j, i);
        ++nVar;
      }
      else
      {
        mean.at<float>(j, i) = 0;
        var.at<float>(j, i) = 0;
      }
    }
  }
  float meanVar = sumVar/nVar;
  for (size_t j = 0; j < boundary.rows; ++j)
  {
    for (size_t i = 0; i < boundary.cols; ++i)
    {
      if (var.at<float>(j, i) > meanVar)
      {
        boundary.at<cv::Vec3b>(j, i)[Veil] = 255;
      }
    }
  }
  cv::imwrite("foo.jpg", boundary);
  std::ofstream output ("foo.csv");
  for (size_t j = 0; j < var.rows; ++j)
  {
    for (size_t i = 0; i < var.cols; ++i)
    {
      output << depth.at<float>(j,i) << " ";
    }
    output << std::endl;
  }
  output.close();

  cvReleaseImage (&tmp);
  return 0;
}
