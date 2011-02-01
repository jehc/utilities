#include <cstdlib>
#include <string>
#include <sstream>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cassert>
#include <opencv2/opencv.hpp>

int
main (int argc, char ** argv)
{
  if (argc != 4)
  {
    std::cerr << "Usage: <executable> sift_file depth_file mask_file" << std::endl;
    return -1;
  }

  const float invalid_depth = 0;

  int count = 0;

  std::ifstream inputSift (argv[1]);

  IplImage * tmp = (IplImage *)cvLoad (argv[2]);
  cv::Mat depths (tmp);

  std::vector<std::string> lines;
  std::string firstLine;
  getline(inputSift, firstLine);
  int keypoints = atoi (firstLine.c_str());
  for (int i = 0; i < keypoints; ++i)
  {
    std::string line;
    getline (inputSift, line);
    std::stringstream ss;
    ss << line;
    std::vector<std::string> tokens;
    std::string token;
    while (ss >> token)
    {
      tokens.push_back (token);
    }
    std::stringstream ss_new;
    for (int i = 0; i < 2; ++i)
    {
      ss_new << tokens[i] << " ";
    }
    float x = atof (tokens[1].c_str());
    float y = atof (tokens[0].c_str());
    int xi = x;
    int yi = y;
    float depth = depths.at<float>(yi, xi);
    ss_new << convert_depth(depth) << " ";
    for (int i = 2; i < tokens.size(); ++i)
    {
      ss_new << tokens [i] << " ";
    }
    if (depth != invalid_depth)
    {    
      lines.push_back (ss_new.str());
      ++count;
    }
    for (int i = 0; i < 7; ++i)
    {
      getline (inputSift, line);
      if (depth != invalid_depth)
      {
        lines.push_back (line);
      }
    }
  }
  cvReleaseImage (&tmp);
  inputSift.close();
  std::ofstream output (argv[1]);
  output << count << " 128" << std::endl;
  for (int i = 0; i < lines.size(); ++i)
  {
    output << lines[i] << std::endl;
  }
  output.close();
  std::cerr << count << std::endl;
  return 0;
}
