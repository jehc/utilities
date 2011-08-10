#include "CoordsFile.h"

#include <fstream>
#include <sstream>

CoordsFile::CoordsFile (const std::string & filename)
{
  std::ifstream in (filename.c_str());
  if (!in)
  {
    throw std::exception();
  }

  while (in)
  {
    CoordsEntry entry;
    if (!(in >> entry))
    {
      break;
    }
    entries [entry.GetIndex()] = entry;
  }

  in.close();
}

std::istream & operator>> (std::istream & in, CoordsEntry & entry)
{
  std::string header;
  while (header == "" && in)
  {
    getline (in, header);
  }

  if (header [0] != '#')
  {
    if (in)
    {
      throw std::exception();
    }
    else
    {
      return in;
    }
  }

  std::stringstream ss_line;
  ss_line << header;

  char hash;
  ss_line.get (hash);

  bool indexSet = false;
  bool nameSet = false;
  bool keysSet = false;
  bool pxSet = false;
  bool pySet = false;
  bool focalSet = false;

  int keys;

  enum TokenMode {None, Index, Name, Keys, Px, Py, Focal} mode = None;
  bool equalFound = false;

  while (ss_line)
  {
    std::string field;
    if (!(ss_line >> field))
    {
      break;
    }

    if (mode == None)
    {
      if (field == "index")
      {
        mode = Index;
        equalFound = false;
      }
      else if (field == "name")
      {
        mode = Name;
        equalFound = false;
      }
      else if (field == "keys")
      {
        mode = Keys;
        equalFound = false;
      }
      else if (field == "px")
      {
        mode = Px;
        equalFound = false;
      }
      else if (field == "py")
      {
        mode = Py;
        equalFound = false;
      }
      else if (field == "focal")
      {
        mode = Focal;
        equalFound = false;
      }
      else
      {
        throw std::exception();
      }
    }
    else if (!equalFound)
    {
      if (field == "=")
      {
        equalFound = true;
      }  
      else
      {
        throw std::exception();
      }
    }
    else
    {
      if (mode == Index)
      {
        std::stringstream ss;
        ss << field;
        ss >> entry.index;
        indexSet = true;
      }
      else if (mode == Name)
      {
        if (field [field.size()-1] == ',')
        {
          field.resize (field.size()-1);
        }
        entry.name = field;
        nameSet = true;
      }
      else if (mode == Keys)
      {
        std::stringstream ss;
        ss << field;
        ss >> keys;
        keysSet = true;
      }
      else if (mode == Px)
      {
        std::stringstream ss;
        ss << field;
        ss >> entry.px;
        pxSet = true;
      }
      else if (mode == Py)
      {
        std::stringstream ss;
        ss << field;
        ss >> entry.py;
        pySet = true;
      }
      else if (mode == Focal)
      {
        std::stringstream ss;
        ss << field;
        ss >> entry.focal;
        focalSet = true;
      }
      else
      {
        throw std::exception();
      }
      mode = None;
    }
  }

  if (!indexSet || !nameSet || !keysSet || !pxSet || !pySet || !focalSet)
  {
    throw std::exception();
  }

  for (int i = 0; i < keys; ++i)
  {
    CoordEntry e;
    if (!(in >> e))
    {
      throw std::exception();
    }
    entry.coords.push_back (e);
  }

  return in;
}

std::istream & operator>> (std::istream & in, CoordEntry & entry)
{
  if (!(in >> entry.key >> entry.x >> entry.y >> entry.scale >> entry.orient >> entry.color[0] >> entry.color[1] >> entry.color[2]))
  {
    throw std::exception();
  }
 
  return in;
}

void CoordsFile::AddEntry (const CoordEntry & entry, size_t index)
{
  entries [index].AddEntry (entry);
}

void CoordsEntry::AddEntry (const CoordEntry & entry)
{
  coords.push_back (entry);
}

void CoordsFile::save (const std::string & filename) const
{
  std::ofstream out (filename.c_str());
  if (!out)
  {
    throw std::exception();
  }

  for (std::map<size_t,CoordsEntry>::const_iterator i = entries.begin(); i != entries.end(); ++i)
  {
    out << i->second;
  }

  out.close();
}

std::ostream & operator<< (std::ostream & out, const CoordsEntry & entry)
{
  out.precision (3);
  out << std::fixed;

  out << "#index = " << entry.index << ", name = " << entry.name << ", keys = " << entry.coords.size() << ", px = " << entry.px << ", py = " << entry.py << ", focal = " << entry.focal << std::endl;

  for (size_t i = 0; i < entry.coords.size(); ++i)
  {
    out << entry.coords[i] << std::endl;
  }
  return out;
}

std::ostream & operator<< (std::ostream & out, const CoordEntry & entry)
{
  out.precision (2);
  out << std::fixed;

  out << entry.key << " " << entry.x << " " << entry.y << " ";

  out.unsetf(std::ios_base::floatfield);

  out << entry.scale << " " << entry.orient << " " << entry.color.transpose();
  return out;
}
