#define INFORMATION_MATRIX

#ifndef TORO_3D_FILE_H
#define TORO_3D_FILE_H

#include <string>
#include <istream>
#include <ostream>
#include <vector>

class Toro3DVertex
{
  friend std::istream & operator>> (std::istream &, Toro3DVertex &);
  friend std::ostream & operator<< (std::ostream &, const Toro3DVertex &);
public:
  Toro3DVertex () {Toro3DVertex(-1,0,0,0,0,0,0);}
  Toro3DVertex (int, double, double, double, double, double, double);
  int id;
  double x, y, z;
  double phi, theta, psi;
};

class Toro3DEdge
{
  friend std::istream & operator>> (std::istream &, Toro3DEdge &);
  friend std::ostream & operator<< (std::ostream &, const Toro3DEdge &);
public:
  Toro3DEdge () {}
  Toro3DEdge (int, int, double, double, double, double, double, double);
  int observed, observer;
  double x, y, z;
  double roll, pitch, yaw;
#ifdef INFORMATION_MATRIX
  double inf_11, inf_12, inf_13, inf_14, inf_15, inf_16;
  double         inf_22, inf_23, inf_24, inf_25, inf_26;
  double                 inf_33, inf_34, inf_35, inf_36;
  double                         inf_44, inf_45, inf_46;
  double                                 inf_55, inf_56;
  double                                         inf_66;
#endif
};

class Toro3DFile
{
  std::vector<Toro3DVertex> vertices;
  std::vector<Toro3DEdge> edges;
public:
  Toro3DFile () {}
  Toro3DFile (const std::string &);
  void save (const std::string &);
  void AddVertex (const Toro3DVertex & vertex)
  {
    if ((int)vertices.size()-1 < vertex.id)
    {
      vertices.resize (vertex.id + 1);
    }
    vertices [vertex.id] = vertex;
  }
  void AddEdge (const Toro3DEdge & edge) { edges.push_back (edge); }
  const std::vector<Toro3DVertex> & GetVertices () const { return vertices; }
};

#endif
