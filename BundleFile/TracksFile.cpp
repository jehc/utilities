#include "TracksFile.h"

#include <fstream>

TracksFile::TracksFile (const std::string & filename)
{
  std::ifstream in (filename.c_str());
  if (!in)
  {
    throw std::exception();
  }

  size_t num;
  if (!(in >> num))
  {
    throw std::exception();
  }

  for (size_t i = 0; i < num; ++i)
  {
    Track t;
    if (!(in >> t))
    {
      throw std::exception();
    }
    tracks.push_back (t);
  }

  in.close();
}

std::istream & operator>> (std::istream & in, Track & t)
{
  size_t num;
  if (!(in >> num))
  {
    throw std::exception();
  }
 
  for (size_t i = 0; i < num; ++i)
  {
    TrackEntry e;
    if (!(in >> e))
    {
      throw std::exception();
    }
    t.entries.push_back (e);
  }

  return in;
}

std::istream & operator>> (std::istream & in, TrackEntry & e)
{
  if (!(in >> e.index >> e.key))
  {
    throw std::exception();
  }

  return in;
}

void TracksFile::save (const std::string & filename) const
{
  std::ofstream out (filename.c_str());
  if (!out)
  {
    throw std::exception();
  }

  out << tracks.size() << std::endl;

  for (size_t i = 0; i < tracks.size(); ++i)
  {
    out << tracks [i] << std::endl;
  }

  out.close();
}

std::ostream & operator<< (std::ostream & out, const Track & t)
{
  out << t.entries.size() << " ";

  for (size_t i = 0; i < t.entries.size(); ++i)
  {
    out << t.entries [i] << " ";
  }

  return out;
}

std::ostream & operator<< (std::ostream & out, const TrackEntry & e)
{
  out << e.index << " " << e.key;

  return out;
}
