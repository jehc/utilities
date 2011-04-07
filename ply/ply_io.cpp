#include "ply_io.h"
#include "ply.h"

void
savePlyFile (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGB> & pointCloud)
{
  typedef struct Vertex {
    float x,y,z; 
    unsigned char r,g,b;
  } Vertex;
  static char *elem_names[] = { "vertex" };
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
    {"diffuse_red", Uint8, Uint8, offsetof(Vertex,r), 0, 0, 0, 0},
    {"diffuse_green", Uint8, Uint8, offsetof(Vertex,g), 0, 0, 0, 0},
    {"diffuse_blue", Uint8, Uint8, offsetof(Vertex,b), 0, 0, 0, 0}
  };
  FILE * file = fopen (filename.c_str(), "w");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for write." << std::endl;
    throw std::exception();
  }
  PlyFile * ply = write_ply (file, 1, elem_names, PLY_BINARY_LE);
  describe_element_ply (ply, "vertex", pointCloud.size());
  for (int i = 0; i < 6; ++i)
  {
    describe_property_ply (ply, &vert_props[i]);
  }
  header_complete_ply (ply);

  put_element_setup_ply (ply, "vertex");
  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator i = pointCloud.begin(); i != pointCloud.end(); ++i)
  {
    Vertex v;
    v.x = i->x;
    v.y = i->y;
    v.z = i->z;
    RgbConverter c;
    c.rgb = i->rgb;
    v.r = c.r;
    v.g = c.g;
    v.b = c.b;
    put_element_ply (ply, (void*)&v);
  }

  close_ply (ply);
  free_ply (ply);
}

void
savePlyFile (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud)
{
  typedef struct Vertex {
    float x,y,z;
    float nx,ny,nz;
    unsigned char r,g,b;
  } Vertex;
  static char *elem_names[] = { "vertex" };
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(Vertex,nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(Vertex,ny), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof(Vertex,nz), 0, 0, 0, 0},
    {"diffuse_red", Uint8, Uint8, offsetof(Vertex,r), 0, 0, 0, 0},
    {"diffuse_green", Uint8, Uint8, offsetof(Vertex,g), 0, 0, 0, 0},
    {"diffuse_blue", Uint8, Uint8, offsetof(Vertex,b), 0, 0, 0, 0}
  };
  FILE * file = fopen (filename.c_str(), "w");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for write." << std::endl;
    throw std::exception();
  }
  PlyFile * ply = write_ply (file, 1, elem_names, PLY_BINARY_LE);
  describe_element_ply (ply, "vertex", pointCloud.size());
  for (int i = 0; i < 9; ++i)
  {
    describe_property_ply (ply, &vert_props[i]);
  }
  header_complete_ply (ply);

  put_element_setup_ply (ply, "vertex");
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = pointCloud.begin(); i != pointCloud.end(); ++i)
  {
    Vertex v;
    v.x = i->x;
    v.y = i->y;
    v.z = i->z;
    RgbConverter c;
    c.rgb = i->rgb;
    v.r = c.r;
    v.g = c.g;
    v.b = c.b;
    v.nx = i->normal_x;
    v.ny = i->normal_y;
    v.nz = i->normal_z;
    put_element_ply (ply, (void*)&v);
  }

  close_ply (ply);
  free_ply (ply);
}

