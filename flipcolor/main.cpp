#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

int
main (int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cout << "Usage: " << argv[0] << " [in.jpg] [out.jpg]" << std::endl;
    return 1;
  }

  cv::Mat image = cv::imread (argv[1], 0);

  cv::Mat imageFlip (image.rows, image.cols, CV_8UC1);
  for (int j = 0; j < image.rows; ++j)
  {
    for (int i = 0; i < image.cols; ++i)
    {
      imageFlip.at<uchar>(j, i) = 255 - image.at<uchar>(j, i);
    }
  }

  cv::imwrite (argv[2], imageFlip);

  return 0;
}
