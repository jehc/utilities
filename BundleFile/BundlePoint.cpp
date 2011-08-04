#include "BundlePoint.h"

#include <exception>

BundlePoint::BundlePoint (const Eigen::Vector3f & position, const Eigen::Vector3i & color)
: position (position), color (color)
{
}

std::ostream & operator<< (std::ostream & out, const BundlePoint & point)
{
  out << point.position.transpose() << " ";

  out << point.color.transpose() << " ";

  out << point.views.size() << " ";

  for (size_t i = 0; i < point.views.size(); ++i)
  {
    out << point.views [i] << " ";
  }

  return out;
}

std::istream & operator>> (std::istream & in, BundlePoint & point)
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
