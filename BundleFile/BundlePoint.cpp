#include "BundlePoint.h"

#include <exception>

BundlePoint::BundlePoint (const Eigen::Vector3d & position, const Eigen::Vector3i & color, const std::vector<BundleView> & views)
: position (position), color (color), views (views)
{
}

std::ostream & operator<< (std::ostream & out, const BundlePoint & point)
{
  out.precision (10);
  out << std::scientific;

  out << point.position.transpose() << std::endl;

  out << point.color.transpose() << std::endl;

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
