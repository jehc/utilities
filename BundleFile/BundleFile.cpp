#include "BundleFile.h"

#include <fstream>
#include <exception>
#include <iostream>

BundleFile::BundleFile (const std::string & filename)
{
  std::ifstream input (filename.c_str());
  std::string junk;
  getline (input, junk);

  int numCameras, numPoints;

  if (!(input >> numCameras >> numPoints))
  {
    throw std::exception();
  }

  cameras.resize(numCameras);
  points.resize(numPoints);

  for (std::vector<BundleCamera>::iterator i = cameras.begin(); i != cameras.end(); ++i)
  {
    input >> *i;
  }

  for (std::vector<BundlePoint>::iterator i = points.begin(); i != points.end(); ++i)
  {
    input >> *i;
  }

  while (input)
  {
    std::string s;
    getline (input, s);
    std::cerr << s << std::endl;
  }

  input.close();
}
