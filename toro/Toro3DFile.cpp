#include "Toro3DFile.h"

#include <cassert>
#include <fstream>
#include <iostream>

Toro3DVertex::Toro3DVertex (int id, double x, double y, double z, double phi, double theta, double psi)
: id(id)
, x(x)
, y(y)
, z(z)
, phi(phi)
, theta(theta)
, psi(psi)
{
}

Toro3DEdge::Toro3DEdge (int observed, int observer, double x, double y, double z, double phi, double theta, double psi)
: observed (observed)
, observer (observer)
, x(x)
, y(y)
, z(z)
, roll(phi)
, pitch(theta)
, yaw(psi)
, inf_11 (1), inf_12 (0), inf_13 (0), inf_14 (0), inf_15 (0), inf_16 (0)
,             inf_22 (1), inf_23 (0), inf_24 (0), inf_25 (0), inf_26 (0)
,                         inf_33 (1), inf_34 (0), inf_35 (0), inf_36 (0)
,                                     inf_44 (1), inf_45 (0), inf_46 (0)
,                                                 inf_55 (1), inf_56 (0)
,                                                             inf_66 (1)
{
}

std::istream & operator>> (std::istream & in, Toro3DVertex & vertex)
{
  if (!(in >> vertex.id >> vertex.x >> vertex.y >> vertex.z >> vertex.phi >> vertex.theta >> vertex.psi))
  {
    throw std::exception();
  }
  return in;
}

std::ostream & operator<< (std::ostream & out, const Toro3DVertex & vertex)
{
  out << "VERTEX3 " << vertex.id << " ";
  out << vertex.x << " " << vertex.y << " " << vertex.z << " ";
  out << vertex.phi << " " << vertex.theta << " " << vertex.psi;
  return out;
}
 
std::istream & operator>> (std::istream & in, Toro3DEdge & edge)
{
  in >> edge.observed >> edge.observer;
  in >> edge.x >> edge.y >> edge.z;
  in >> edge.roll >> edge.pitch >> edge.yaw;
#ifdef INFORMATION_MATRIX
  in >> edge.inf_11 >> edge.inf_12 >> edge.inf_13 >> edge.inf_14 >> edge.inf_15 >> edge.inf_16;
  in >> edge.inf_22 >> edge.inf_23 >> edge.inf_24 >> edge.inf_25 >> edge.inf_26;
  in >> edge.inf_33 >> edge.inf_34 >> edge.inf_35 >> edge.inf_36;
  in >> edge.inf_44 >> edge.inf_45 >> edge.inf_46;
  in >> edge.inf_55 >> edge.inf_56;
  in >> edge.inf_66;
#endif

  if (!in)
  {
    throw std::exception();
  }
  return in;
}

std::ostream & operator<< (std::ostream & out, const Toro3DEdge & edge)
{
  out << "EDGE3 " << edge.observed << " " << edge.observer << " ";
  out << edge.x << " " << edge.y << " " << edge.z << " ";
  out << edge.roll << " " << edge.pitch << " " << edge.yaw << " ";
#ifdef INFORMATION_MATRIX
  out << edge.inf_11 << " " << edge.inf_12 << " " << edge.inf_13 << " ";
  out << edge.inf_14 << " " << edge.inf_15 << " " << edge.inf_16 << " ";
  out << edge.inf_22 << " " << edge.inf_23 << " " << edge.inf_24 << " ";
  out << edge.inf_25 << " " << edge.inf_26 << " ";
  out << edge.inf_33 << " " << edge.inf_34 << " " << edge.inf_35 << " ";
  out << edge.inf_36 << " ";
  out << edge.inf_44 << " " << edge.inf_45 << " " << edge.inf_46 << " ";
  out << edge.inf_55 << " " << edge.inf_56 << " ";
  out << edge.inf_66;
#endif

  return out;
}

Toro3DFile::Toro3DFile (const std::string & filename)
{
  std::ifstream in (filename.c_str());
  if (!in)
  {
    throw std::exception();
  }

  while (in)
  {
    std::string type;
    if (!(in >> type))
    {
      break;
    }

    if (type == "EDGE3")
    {
      Toro3DEdge edge;
      in >> edge;
      edges.push_back (edge);
    }
    else if (type == "VERTEX3")
    {
      Toro3DVertex vertex;
      in >> vertex;
      if ((int)vertices.size()-1 < vertex.id)
      {
        vertices.resize(vertex.id+1);
      }
      vertices [vertex.id] = vertex;
    }
    else
    {
      throw std::exception();
    }
  }

  in.close();
}

void Toro3DFile::save (const std::string & filename)
{
  std::ofstream out (filename.c_str());
  if (!out)
  {
    throw std::exception();
  }

  std::vector<bool> selected (vertices.size());
  for (size_t i = 0; i < edges.size(); ++i)
  {
    selected [edges[i].observed] = selected [edges[i].observer] = true;
  }

  for (size_t i = 0; i < vertices.size(); ++i)
  {
    if (selected [i])
    {
      out << vertices [i] << std::endl;
    }
  }

  for (size_t i = 0; i < edges.size(); ++i)
  {
    out << edges [i] << std::endl;
  }

  out.close();
}
