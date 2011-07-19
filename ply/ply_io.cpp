#include "ply_io.h"
#include "ply.h"
#include <fstream>
#include <cmath>

void savePlyFileRangeImage (const std::string &, const pcl::PointCloud<pcl::PointXYZRGB> &);
void savePlyFileRangeImage (const std::string &, const pcl::PointCloud<pcl::PointXYZRGBNormal> &);
void savePlyFileRangeImage (const std::string &, const pcl::PointCloud<pcl::PointSurfel> &);
void savePlyFilePointCloud (const std::string &, const pcl::PointCloud<pcl::PointXYZRGB> &);
void savePlyFilePointCloud (const std::string &, const pcl::PointCloud<pcl::PointXYZRGBNormal> &);
void savePlyFilePointCloud (const std::string &, const pcl::PointCloud<pcl::PointSurfel> &);

typedef struct Vertex 
{
  float x,y,z;
  unsigned char r,g,b;
  float confidence;
} PlyVertex;

typedef struct VertexNormal 
{
  float x,y,z;
  float nx,ny,nz;
  unsigned char r,g,b;
} PlyVertexNormal;

typedef struct Surfel
{
  float x, y, z;
  float nx, ny, nz;
  unsigned char r, g, b;
  float confidence;
  float curvature;
  float radius;
} PlySurfel;

struct RangeGridPnt
{
  unsigned char num_pts;
  int *pts;
};

typedef struct VertexIn
{
  float x, y, z;
  void * other_props;
} PlyVertexIn;

void
loadPlyFile (const std::string & filename, pcl::PointCloud<pcl::PointXYZ> & pointCloud)
{
  
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(VertexIn,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(VertexIn,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(VertexIn,z), 0, 0, 0, 0},
  };

  FILE * file = fopen (filename.c_str(), "r");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for read." << std::endl;
    throw std::exception();
  }

  PlyFile * ply = read_ply (file);

  for (int i = 0; i < ply->num_elem_types; ++i)
  {
    int elem_count;
    char * elem_name = setup_element_read_ply (ply, i, &elem_count);
    if (equal_strings ("vertex", elem_name))
    {
      setup_property_ply (ply, &vert_props[0]);
      setup_property_ply (ply, &vert_props[1]);
      setup_property_ply (ply, &vert_props[2]);

      get_other_properties_ply (ply, offsetof(VertexIn, other_props));

      for (int j = 0; j < elem_count; ++j)
      {
        PlyVertexIn v;
        get_element_ply (ply, (void *)&v);
        pcl::PointXYZ point;
        // ply lib sucks so I have to fix the endianess manually
        union Flipper { float f; uint8_t b [4]; } to1, to2, to3, from1, from2, from3;
        from1.f = v.x;
        from2.f = v.y;
        from3.f = v.z;
        for (int i = 0; i < 4; ++i)
        {
          if (ply->file_type == PLY_BINARY_BE)
          {
            to1.b[i] = from1.b[3 - i];
            to2.b[i] = from2.b[3 - i];
            to3.b[i] = from3.b[3 - i];
          }
          else
          {
            to1.b[i] = from1.b[i];
            to2.b[i] = from2.b[i];
            to3.b[i] = from3.b[i];
	  }
        }
        point.x = to1.f;
        point.y = to2.f;
        point.z = to3.f;
        pointCloud.push_back (point);
      }
    }
    else
    {
      get_other_element_ply (ply);
    }
  }

  close_ply (ply);
  free_ply (ply);
}

void
savePlyFile (const std::string & filename, const pcl::PointCloud<pcl::PointSurfel> & pointCloud)
{
  if (pointCloud.is_dense)
  {
    savePlyFilePointCloud (filename, pointCloud);
  }
  else
  {
    savePlyFileRangeImage (filename, pointCloud);
  }
}

void
savePlyFileRangeImage (const std::string & filename, const pcl::PointCloud<pcl::PointSurfel> & pointCloud)
{
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(Surfel,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Surfel,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Surfel,z), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(Surfel,nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(Surfel,ny),0,0,0,0},
    {"nz", Float32, Float32, offsetof(Surfel,nz),0,0,0,0},
    {"diffuse_red", Uint8, Uint8, offsetof(Surfel,r), 0, 0, 0, 0},
    {"diffuse_green", Uint8, Uint8, offsetof(Surfel,g), 0, 0, 0, 0},
    {"diffuse_blue", Uint8, Uint8, offsetof(Surfel,b), 0, 0, 0, 0},
    {"confidence", Float32, Float32, offsetof(Surfel,confidence), 0, 0, 0, 0},
    {"radius", Float32, Float32, offsetof(Surfel, radius),0,0,0,0},
    {"curvature", Float32, Float32, offsetof(Surfel, curvature), 0, 0, 0, 0}
  };

  static PlyProperty range_props[] =
  {
    {"vertex_indices", Int32, Int32, offsetof(RangeGridPnt,pts),
      1, Uint8, Uint8, offsetof(RangeGridPnt,num_pts)}
  };

  static char * elem_names[] = {"vertex", "range_grid"};

  FILE * file = fopen (filename.c_str(), "w");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for write." << std::endl;
    throw std::exception ();
  }

  std::vector<PlySurfel> vertices;
  std::vector<std::vector<int> > rangeGrid (pointCloud.height, std::vector<int> (pointCloud.width, -1));
  vertices.reserve (pointCloud.size());
  for (int j = pointCloud.height - 1; j >= 0; --j)
  {
    for (int i = 0; i < pointCloud.width; ++i)
    {
      const pcl::PointSurfel & surfel = pointCloud (i, j);
      if (!std::isnan (surfel.x) && !std::isnan (surfel.y) && !std::isnan (surfel.z))
      {
        PlySurfel s;
        s.x = surfel.x;
        s.y = surfel.y;
        s.z = surfel.z;
        ByteExtractor be;
        be.rgba = surfel.rgba;
        s.r = be.b[0];
        s.g = be.b[1];
        s.b = be.b[2];
        s.nx = surfel.normal_x;
        s.ny = surfel.normal_y;
        s.nz = surfel.normal_z;
        s.radius = surfel.radius;
        s.curvature = surfel.curvature;
        s.confidence = surfel.confidence;
        vertices.push_back (s);
        rangeGrid [j][i] = vertices.size() - 1;
      }
    }
  }

  PlyFile * ply = write_ply (file, 2, elem_names, PLY_BINARY_LE);

  std::stringstream nCols;
  nCols << "num_cols " << pointCloud.width;
  std::vector<char> nColsTemp (nCols.str().size() + 1);
  memcpy (&nColsTemp[0], nCols.str().c_str(), nColsTemp.size());
  append_obj_info_ply (ply, &nColsTemp[0]);
  std::stringstream nRows;
  nRows << "num_rows " << pointCloud.height;
  std::vector<char> nRowsTemp (nRows.str().size() + 1);
  memcpy (&nRowsTemp [0], nRows.str().c_str(), nRowsTemp.size());
  append_obj_info_ply (ply, &nRowsTemp[0]);

  describe_element_ply (ply, "vertex", vertices.size());

  for (int i = 0; i < 12; ++i)
  {
    describe_property_ply (ply, &vert_props[i]);
  }

  describe_element_ply (ply, "range_grid", pointCloud.size());

  for (int i = 0; i < 1; ++i)
  {
    describe_property_ply (ply, &range_props [i]);
  }

  header_complete_ply (ply);

  put_element_setup_ply (ply, "vertex");
  for (int i = 0; i < vertices.size(); ++i)
  {
    put_element_ply (ply, (void*)&vertices[i]);
  }

  put_element_setup_ply (ply, "range_grid");
  for (int i = rangeGrid.size() - 1; i >= 0; --i)
  {
    for (int j = 0; j < rangeGrid[i].size(); ++j)
    {
      RangeGridPnt gridCell;
      int cellValue = rangeGrid[i][j];
      if (cellValue == -1)
      {
        gridCell.num_pts = 0;
        gridCell.pts = 0;
      }
      else
      {
        gridCell.num_pts = 1;
        gridCell.pts = &cellValue;
      }
      put_element_ply (ply, (void*)&gridCell);
    }
  }

  close_ply (ply);
  free_ply (ply);

  const Eigen::Quaternionf & q = pointCloud.sensor_orientation_;
  const Eigen::Vector4f & t = pointCloud.sensor_origin_;
  std::string xfFilename = filename;
  xfFilename.replace (filename.find_last_of (".") + 1, 3, "xf");
  std::ofstream xfOutput (xfFilename.c_str());

  Eigen::Matrix3f r (q);
  Eigen::Matrix4f xf;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      xf(i, j) = r(i, j);
    }
    xf (i, 3) = t[i];
  }
  xf (3, 0) = xf(3, 1) = xf(3, 2) = 0;
  xf (3, 3) = 1;
  xfOutput << xf;

  xfOutput.close();
}

void
savePlyFilePointCloud (const std::string & filename, const pcl::PointCloud<pcl::PointSurfel> & pointCloud)
{
  static char *elem_names[] = { "vertex" };
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(Surfel,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Surfel,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Surfel,z), 0, 0, 0, 0},
    {"diffuse_red", Uint8, Uint8, offsetof(Surfel,r), 0, 0, 0, 0},
    {"diffuse_green", Uint8, Uint8, offsetof(Surfel,g), 0, 0, 0, 0},
    {"diffuse_blue", Uint8, Uint8, offsetof(Surfel,b), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(Surfel, nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(Surfel, ny), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof(Surfel, nz), 0, 0, 0, 0},
    {"confidence", Float32, Float32, offsetof(Surfel, confidence), 0, 0, 0, 0},
    {"radius", Float32, Float32, offsetof (Surfel, radius), 0, 0, 0, 0},
    {"curvature", Float32, Float32, offsetof(Surfel, curvature), 0, 0, 0, 0}
  };
  FILE * file = fopen (filename.c_str(), "w");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for write." << std::endl;
    throw std::exception();
  }
  PlyFile * ply = write_ply (file, 1, elem_names, PLY_BINARY_LE);
  describe_element_ply (ply, "vertex", pointCloud.size());
  for (int i = 0; i < 12; ++i)
  {
    describe_property_ply (ply, &vert_props[i]);
  }
  header_complete_ply (ply);

  put_element_setup_ply (ply, "vertex");
  for (pcl::PointCloud<pcl::PointSurfel>::const_iterator i = pointCloud.begin(); i != pointCloud.end(); ++i)
  {
    Surfel s;
    s.x = i->x;
    s.y = i->y;
    s.z = i->z;
    ByteExtractor be;
    be.rgba = i->rgba;
    s.r = be.b[0];
    s.g = be.b[1];
    s.b = be.b[2];
    s.nx = i->normal_x;
    s.ny = i->normal_y;
    s.nz = i->normal_z;
    s.confidence = i->confidence;
    s.curvature = i->curvature;
    s.radius = i->radius;
    put_element_ply (ply, (void*)&s);
  }

  close_ply (ply);
  free_ply (ply);
}

void
savePlyFile (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGB> & pointCloud)
{
  if (pointCloud.is_dense)
  {
    savePlyFilePointCloud (filename, pointCloud);
  }
  else
  {
    savePlyFileRangeImage (filename, pointCloud);
  }
}

void
savePlyFileRangeImage (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGB> & pointCloud)
{
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(Vertex,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex,z), 0, 0, 0, 0},
    {"diffuse_red", Uint8, Uint8, offsetof(Vertex,r), 0, 0, 0, 0},
    {"diffuse_green", Uint8, Uint8, offsetof(Vertex,g), 0, 0, 0, 0},
    {"diffuse_blue", Uint8, Uint8, offsetof(Vertex,b), 0, 0, 0, 0},
    {"confidence", Float32, Float32, offsetof(Vertex,confidence), 0, 0, 0, 0}
  };

  static PlyProperty range_props[] = 
  {
    {"vertex_indices", Int32, Int32, offsetof(RangeGridPnt,pts),
      1, Uint8, Uint8, offsetof(RangeGridPnt,num_pts)}
  };

  static char * elem_names[] = {"vertex", "range_grid"};

  FILE * file = fopen (filename.c_str(), "w");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for write." << std::endl;
    throw std::exception ();
  }

  std::vector<PlyVertex> vertices;
  std::vector<std::vector<int> > rangeGrid (pointCloud.height, std::vector<int> (pointCloud.width, -1));
  vertices.reserve (pointCloud.size());
  for (int j = pointCloud.height - 1; j >= 0; --j)
  {
    for (int i = 0; i < pointCloud.width; ++i)
    {
      const pcl::PointXYZRGB & vertex = pointCloud (i, j);
      if (!std::isnan (vertex.x) && !std::isnan (vertex.y) && !std::isnan (vertex.z))
      {
        PlyVertex v;
        v.x = vertex.x;
        v.y = vertex.y;
        v.z = vertex.z;
        RgbConverter c;
        c.rgb = vertex.rgb;
        v.r = c.r;
        v.g = c.g;
        v.b = c.b;
        vertices.push_back (v);
        rangeGrid [j][i] = vertices.size() - 1;
        int S_b = std::min (j, std::min ((int)pointCloud.height - j, std::min (i, (int)pointCloud.width - i)));
        int S_max = 8;
        float W_b = S_b >= S_max ? 1.0 : (float)S_b/S_max;
        float W_v = 1.0/sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        v.confidence = W_v*W_b;
      }
    }
  }

  PlyFile * ply = write_ply (file, 2, elem_names, PLY_BINARY_LE);

  std::stringstream nCols;
  nCols << "num_cols " << pointCloud.width;
  std::vector<char> nColsTemp (nCols.str().size() + 1);
  memcpy (&nColsTemp[0], nCols.str().c_str(), nColsTemp.size());
  append_obj_info_ply (ply, &nColsTemp[0]);
  std::stringstream nRows;
  nRows << "num_rows " << pointCloud.height;
  std::vector<char> nRowsTemp (nRows.str().size() + 1);
  memcpy (&nRowsTemp [0], nRows.str().c_str(), nRowsTemp.size());
  append_obj_info_ply (ply, &nRowsTemp[0]);

  describe_element_ply (ply, "vertex", vertices.size());

  for (int i = 0; i < 7; ++i)
  {
    describe_property_ply (ply, &vert_props[i]);
  }

  describe_element_ply (ply, "range_grid", pointCloud.size());

  for (int i = 0; i < 1; ++i)
  {
    describe_property_ply (ply, &range_props [i]);
  }

  header_complete_ply (ply);

  put_element_setup_ply (ply, "vertex");
  for (int i = 0; i < vertices.size(); ++i)
  {
    put_element_ply (ply, (void*)&vertices[i]);
  }

  put_element_setup_ply (ply, "range_grid");
  for (int i = rangeGrid.size() - 1; i >= 0; --i)
  {
    for (int j = 0; j < rangeGrid[i].size(); ++j)
    {
      RangeGridPnt gridCell;
      int cellValue = rangeGrid[i][j];
      if (cellValue == -1)
      {
        gridCell.num_pts = 0;
        gridCell.pts = 0;
      }
      else
      {
        gridCell.num_pts = 1;
        gridCell.pts = &cellValue;
      }
      put_element_ply (ply, (void*)&gridCell);
    }
  }

  close_ply (ply);
  free_ply (ply);

  const Eigen::Quaternionf & q = pointCloud.sensor_orientation_;
  const Eigen::Vector4f & t = pointCloud.sensor_origin_;
  std::string xfFilename = filename;
  xfFilename.replace (filename.find_last_of (".") + 1, 3, "xf");
  std::ofstream xfOutput (xfFilename.c_str());
  
  Eigen::Matrix3f r (q);
  Eigen::Matrix4f xf;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      xf(i, j) = r(i, j);
    }
    xf (i, 3) = t[i];
  }
  xf (3, 0) = xf(3, 1) = xf(3, 2) = 0;
  xf (3, 3) = 1;
  xfOutput << xf;

  xfOutput.close();
}

void
savePlyFilePointCloud (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGB> & pointCloud)
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
  if (pointCloud.is_dense)
  {
    savePlyFilePointCloud (filename, pointCloud);
  }
  else
  {
    savePlyFileRangeImage (filename, pointCloud);
  }
}

void
savePlyFileRangeImage (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud)
{
  static PlyProperty vert_props[] = {
    {"x", Float32, Float32, offsetof(VertexNormal,x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(VertexNormal,y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(VertexNormal,z), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(VertexNormal,nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(VertexNormal,ny), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof(VertexNormal,nz), 0, 0, 0, 0},
    {"diffuse_red", Uint8, Uint8, offsetof(VertexNormal,r), 0, 0, 0, 0},
    {"diffuse_green", Uint8, Uint8, offsetof(VertexNormal,g), 0, 0, 0, 0},
    {"diffuse_blue", Uint8, Uint8, offsetof(VertexNormal,b), 0, 0, 0, 0}
  };

  static PlyProperty range_props[] =
  {
    {"vertex_indices", Int32, Int32, offsetof(RangeGridPnt,pts),
      1, Uint8, Uint8, offsetof(RangeGridPnt,num_pts)},
  };

  static char * elem_names[] = {"vertex", "range_grid"};

  FILE * file = fopen (filename.c_str(), "w");
  if (!file)
  {
    std::cout << "Could not open " << filename << " for write." << std::endl;
    throw std::exception ();
  }

  std::vector<PlyVertexNormal> vertices;
  std::vector<std::vector<int> > rangeGrid (pointCloud.height, std::vector<int> (pointCloud.width, -1));
  vertices.reserve (pointCloud.size());
  for (int j = pointCloud.height - 1; j >= 0; --j)
  {
    for (int i = 0; i < pointCloud.width; ++i)
    {
      const pcl::PointXYZRGBNormal & vertex = pointCloud (i, j);
      if (!std::isnan (vertex.x) && !std::isnan (vertex.y) && !std::isnan (vertex.z))
      {
        PlyVertexNormal v;
        v.x = vertex.x;
        v.y = vertex.y;
        v.z = vertex.z;
        RgbConverter c;
        c.rgb = vertex.rgb;
        v.r = c.r;
        v.g = c.g;
        v.b = c.b;
        v.nx = vertex.normal_x;
        v.ny = vertex.normal_y;
        v.nz = vertex.normal_z;
        vertices.push_back (v);
        rangeGrid [j][i] = vertices.size() - 1;
      }
    }
  }

  PlyFile * ply = write_ply (file, 2, elem_names, PLY_BINARY_LE);

  std::stringstream nCols;
  nCols << "num_cols " << pointCloud.width;
  std::vector<char> nColsTemp (nCols.str().size() + 1);
  memcpy (&nColsTemp[0], nCols.str().c_str(), nColsTemp.size());
  append_obj_info_ply (ply, &nColsTemp[0]);
  std::stringstream nRows;
  nRows << "num_rows " << pointCloud.height;
  std::vector<char> nRowsTemp (nRows.str().size() + 1);
  memcpy (&nRowsTemp [0], nRows.str().c_str(), nRowsTemp.size());
  append_obj_info_ply (ply, &nRowsTemp[0]);

  describe_element_ply (ply, "vertex", vertices.size());

  for (int i = 0; i < 9; ++i)
  {
    describe_property_ply (ply, &vert_props[i]);
  }

  describe_element_ply (ply, "range_grid", pointCloud.size());

  for (int i = 0; i < 1; ++i)
  {
    describe_property_ply (ply, &range_props [i]);
  }

  header_complete_ply (ply);

  put_element_setup_ply (ply, "vertex");
  for (int i = 0; i < vertices.size(); ++i)
  {
    put_element_ply (ply, (void*)&vertices[i]);
  }

  put_element_setup_ply (ply, "range_grid");
  for (int i = rangeGrid.size() - 1; i >= 0; --i)
  {
    for (int j = 0; j < rangeGrid[i].size(); ++j)
    {
      RangeGridPnt gridCell;
      int cellValue = rangeGrid[i][j];
      if (cellValue == -1)
      {
        gridCell.num_pts = 0;
        gridCell.pts = 0;
      }
      else
      {
        gridCell.num_pts = 1;
        gridCell.pts = &cellValue;
      }
      put_element_ply (ply, (void*)&gridCell);
    }
  }

  close_ply (ply);
  free_ply (ply);

  const Eigen::Quaternionf & q = pointCloud.sensor_orientation_;
  const Eigen::Vector4f & t = pointCloud.sensor_origin_;
  std::string xfFilename = filename;
  xfFilename.replace (filename.find_last_of (".") + 1, 3, "xf");
  std::ofstream xfOutput (xfFilename.c_str());

  Eigen::Matrix3f r (q);
  Eigen::Matrix4f xf;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      xf(i, j) = r(i, j);
    }
    xf (i, 3) = t[i];
  }
  xf (3, 0) = xf(3, 1) = xf(3, 2) = 0;
  xf (3, 3) = 1;
  xfOutput << xf;

  xfOutput.close();
}

void
savePlyFilePointCloud (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud)
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

