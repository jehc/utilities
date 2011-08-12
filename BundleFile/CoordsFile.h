#ifndef COORDS_FILE_H
#define COORDS_FILE_H

#include <string>
#include <vector>
#include <map>

#include <Eigen/Dense>

class CoordEntry
{
  size_t key;
  double x;
  double y;
  double scale;
  double orient;
  double distance;
  Eigen::Vector3i color;
  friend std::istream & operator>> (std::istream &, CoordEntry &);
  friend std::ostream & operator<< (std::ostream &, const CoordEntry &);
public:
  CoordEntry () {}
  CoordEntry (size_t key, double x, double y, double scale, double orient, const Eigen::Vector3i & color, double distance=0.0):key(key),x(x),y(y),scale(scale),orient(orient),color(color),distance(distance) {}
  inline void SetDistance(double d) { distance = d; }
};

class CoordsEntry
{
  int index;
  std::string name;
  double px;
  double py;
  double focal;
  std::vector<CoordEntry> coords;
  friend std::istream & operator>> (std::istream &, CoordsEntry &);
  friend std::ostream & operator<< (std::ostream &, const CoordsEntry &);
public:
  void AddEntry (const CoordEntry &);
  const std::vector<CoordEntry> & GetEntries () const { return coords; }
  size_t GetIndex () const { return index; } 
  inline void SetDistance (size_t key, double d) { coords [key].SetDistance (d); }
};

class CoordsFile
{
  std::map<size_t,CoordsEntry> entries;
public:
  CoordsFile (const std::string &);
  void AddEntry (const CoordEntry &, size_t);
  size_t GetNextID (size_t i) const { return entries.find(i)->second.GetEntries().size(); }
  inline void SetDistance (size_t file, size_t key, double d) { entries.find (file)->second.SetDistance (key, d); }
  void save (const std::string &) const;
};

#endif
