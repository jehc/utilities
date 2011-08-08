#include "BundleView.h"

#include <exception>

std::istream & operator>> (std::istream & in, BundleView & view)
{
  if (!(in >> view.camera >> view.key >> view.x >> view.y))
  {
    throw std::exception();
  }

  return in;
}

std::ostream & operator<< (std::ostream & out, const BundleView & view)
{
  out.precision (4);
  out << std::fixed;

  out << view.camera << " " << view.key << " " << view.x << " " << view.y;

  return out;
}
