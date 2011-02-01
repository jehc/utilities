#include <cstdlib>
#include <string>
#include <sstream>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cassert>

float
convert_depth (uint16_t measurement)
{
  return -334.25811/(measurement - 1087.3163);
}

int
main (int argc, char ** argv)
{
  if (argc != 4)
  {
    std::cerr << "Usage: <executable> sift_file depth_file mask_file" << std::endl;
    return -1;
  }

  const uint16_t invalid_depth = 2047;

  int count = 0;

  std::vector<uint16_t> depths;
  std::vector<bool> masks;
  std::ifstream inputSift (argv[1]);
  std::ifstream inputDepth (argv[2]);
  std::ifstream inputMask (argv[3]);

  depths.reserve (640*480);
  uint16_t depth;
  while (inputDepth >> depth)
  {
    depths.push_back (depth);
  }
  assert (depths.size() == 640*480);
  inputDepth.close();

  masks.reserve(640*480/32/32);
  double mask;
  while (inputMask >> mask)
  {
    masks.push_back (mask > 0.5);
  }
  std::cerr << masks.size() << std::endl;
  //assert (masks.size() == 640*480/32/32);
  inputMask.close();

  std::stringstream debug;
  debug << argv[1] << ".debug";
  std::ofstream debugOutput (debug.str().c_str());

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
    uint16_t depth = depths [ yi * 640 + xi];
    bool mask = masks [ (yi/32)*(640/32) + xi/32];
    ss_new << convert_depth(depth) << " ";
    for (int i = 2; i < tokens.size(); ++i)
    {
      ss_new << tokens [i] << " ";
    }
    if (depth != invalid_depth)// && mask)
    {    
      lines.push_back (ss_new.str());
      ++count;
    }
    else
    {
      debugOutput << x << " " << y << std::endl;
    }
    for (int i = 0; i < 7; ++i)
    {
      getline (inputSift, line);
      if (depth != invalid_depth)// && mask)
      {
        lines.push_back (line);
      }
    }
  }
  inputSift.close();
  debugOutput.close();
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
