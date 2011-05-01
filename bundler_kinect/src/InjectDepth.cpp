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
  if (argc != 3)
  {
    std::cerr << "Usage: <executable> sift_file depth_file" << std::endl;
    return -1;
  }
  const float focal = 523;
  const float baseline = 0.07881;
  const float offset = 1093.4;
  const float invalid_depth = 0;

  int count = 0;

  std::ifstream inputSift (argv[1]);

  std::ifstream depthInput(argv[2]);
  bool depthAvailable = true;
  uint32_t rows = 1, cols = 1;
  if (!depthInput)
  {
    std::cout << "Could not find file " << argv[2] << std::endl;
    rows = cols = 1;
    depthAvailable = false;
  }

  if (depthAvailable && !depthInput.read((char*)&rows, sizeof(uint32_t)))
  {
    std::cout << "Could not read rows in file " << argv[2] << std::endl;
    rows = cols = 1;
    depthAvailable = false;
  }
  if(depthAvailable && !depthInput.read((char*)&cols, sizeof(uint32_t)))
  {
    std::cout << "Could not read cols in file " << argv[2] << std::endl;
    rows = cols = 1;
    depthAvailable = false;
  }
  cv::Mat1f depths(rows, cols);
  if(depthAvailable && !depthInput.read((char*)depths.data, depths.rows*depths.cols*sizeof(float)))
  {
    std::cout << "Could not read data in file " << argv[2] << std::endl;
    depthAvailable = false;
  }

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
    std::stringstream ss_orig_new;
    for (int i = 0; i < 2; ++i)
    {
      ss_new << tokens[i] << " ";
      ss_orig_new << tokens[i] << " ";
    }
    float x = atof (tokens[1].c_str());
    float y = atof (tokens[0].c_str());
    int xi = x;
    int yi = y;
    float depth = depthAvailable ? depths.at<float>(yi,xi) : 0;
    ss_new << depth << " ";
    for (size_t i = 2; i < tokens.size(); ++i)
    {
      ss_new << tokens [i] << " ";
      ss_orig_new << tokens[i] << " ";
    }
    if (true)//depth != invalid_depth)
    {    
      lines.push_back (ss_new.str());
      ++count;
    }
    for (size_t i = 0; i < 7; ++i)
    {
      getline (inputSift, line);
      if (true)//depth != invalid_depth)
      {
        lines.push_back (line);
      }
    }
  }
  inputSift.close();
  std::ofstream output (argv[1]);
  output << count << " 128" << std::endl;
  for (size_t i = 0; i < lines.size(); ++i)
  {
    output << lines[i] << std::endl;
  }
  output.close();
  std::cerr << count << std::endl;
  return 0;
}
