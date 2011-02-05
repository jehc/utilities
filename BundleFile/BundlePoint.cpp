#include "BundlePoint.h"

#include <exception>

BundlePoint::BundlePoint (const cv::Vec3f & position, const cv::Vec3b & color)
: position (position), color (color)
{
}

ostream & operator<< (ostream & out, const BundlePoint & point)
{
  for (int i = 0; i < 3; ++i)
  {
    out << point.position[i] << " ";
  }

  for (int i = 0; i < 3; ++i)
  {
    out << (unsigned int)point.color[i] << " ";
  }

  return out;
}

istream & operator>> (istream & in, BundlePoint & point)
{
  for (int i = 0; i < 3; ++i)
  {
    if (!(in >> point.position[i]))
    {
      throw std::exception();
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    unsigned int temp;
    if (!(in >> temp))
    {
      throw std::exception();
    }

    point.color[i] = temp;
  }

  int numViews;
  if (!(in >> numViews))
  {
    throw std::exception();
  }

  point.views.resize (numViews);

  for (std::vector<BundleView>::iterator i = point.views.begin(); i != point.views.end(); ++i)
  {
    in >> *i;
  }

  return in;
}
