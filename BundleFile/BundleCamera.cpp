#include "BundleCamera.h"

#include <exception>

std::istream & operator>> (std::istream & in, BundleCamera & camera)
{
  if (!(in >> camera.f >> camera.k1 >> camera.k2))
  {
    throw std::exception();
  }

  for (int j = 0; j < 3; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (!(in >> camera.R(j, i)))
      {
        throw std::exception();
      }
    }
  }

  for (int j = 0; j < 3; ++j)
  {
    if (!(in >> camera.t[j]))
    {
      throw std::exception();
    }
  }
  camera.valid = camera.f > 0;
  return in;
}

std::ostream & operator<< (std::ostream & out, const BundleCamera & camera)
{
  out << camera.f << " " << camera.k1 << " " << camera.k2 << std::endl;

  out << camera.R << std::endl;

  out << camera.t.transpose();

  return out;
}
