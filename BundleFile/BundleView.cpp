#include "BundleView.h"

#include <exception>

istream & operator>> (istream & in, BundleView & view)
{
  if (!(in >> view.camera >> view.key >> view.x >> view.y))
  {
    throw std::exception();
  }

  return in;
}
