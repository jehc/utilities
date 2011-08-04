#include "BundleFile.h"

#include <fstream>
#include <exception>
#include <iostream>

BundleFile::BundleFile (const std::string & filename)
{
  std::ifstream input (filename.c_str());
  std::string junk;
  if (!getline (input, junk))
  {
    throw std::exception();
  }

  int numCameras = 0, numPoints = 0;

  if (!(input >> numCameras >> numPoints))
  {
    throw std::exception();
  }

  for (int i = 0; i < numCameras; ++i)
  {
    BundleCamera c;
    input >> c;
    cameras.push_back (c);
  }

  for (int i = 0; i < numPoints; ++i)
  {
    BundlePoint p;
    input >> p;
    points.push_back (p);
  }
 
  input.close();
}

void
BundleFile::save (const std::string & filename)
{
  std::ofstream output (filename.c_str());
  if (!output)
  {
    throw std::exception();
  }

  output << cameras.size() << " " << points.size() << std::endl;

  for (size_t i = 0; i < cameras.size(); ++i)
  {
    output << cameras [i] << std::endl;
  }

  for (size_t i = 0; i < points.size(); ++i)
  {
    output << points [i] << std::endl;
  }

  output.close();
}
