#include "BundleFile.h"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <sys/time.h>
#include <getopt.h>

#include <fstream>
#include <iostream>
#include <cassert>
#include <string>
#include <exception>
#include <set>

#include "ply_io.h"

#include "kvo.h"

#include <opencv2/opencv.hpp>

#include "omp.h"

#define TIMED struct timeval tic, toc;
#define TIME_BEGIN( msg ) std::cout << msg << std::endl; gettimeofday ( &tic, 0 );
#define TIME_END gettimeofday ( &toc, \
  0 ); std::cout << "Complete in " << toc.tv_sec - tic.tv_sec << " seconds." << std::endl;

#define EIGEN_DONT_PARALLELIZE

struct RayClique
{
  Eigen::Vector3f color;
  std::vector<int> voxels;
  int point;
};

void
printCommand ( int argc, char * * argv )
{
  for ( int i = 0; i < argc; ++i )
  {
    std::cout << argv [i] << " ";
  }
  std::cout << std::endl;
}

void
basis2rotation (Eigen::Matrix3f & basis)
{
  // Hold first row fixed
  Eigen::Vector3f row1 (basis(0,0), basis(0,1), basis(0,2));
    
  // Project 2nd row onto tangent plane
  Eigen::Vector3f row2 (basis(1,0), basis(1,1), basis(1,2));
  row2 -= row1.dot(row2)*row1;
    
  // Take cross product of first two rows and flip sign to match 3rd row
  Eigen::Vector3f row3 (basis(2,0), basis(2,1), basis(2,2));
  Eigen::Vector3f c = row1.cross (row2);
  bool flip = row3.dot(c) < 0;
  row3 = c;
  if (flip)
  {
    row3 *= -1;
  }

  for (int i = 0; i < 3; ++i)
  {
    basis (1, i) = row2[i];
    basis (2, i) = row3[i];
  }
  
  float det = basis.determinant();
  basis (2,0) /= det;
  basis (2,1) /= det;
  basis (2,2) /= det;
  std::cout << "Rotation matrice's det is " << basis.determinant() << std::endl;
}   

std::set<std::pair<int, int> > LoadKeys ( const std::string & keypointFilename )
{
  std::ifstream input ( keypointFilename.c_str () );

  std::set<std::pair<int, int> > keys;
  if ( !input )
  {
    return keys;
  }
  std::string junk;
  getline ( input, junk );
  while ( input )
  {
    float x, y;
    if ( !( input >> x ) )
    {
      break;
    }
    if ( !( input >> y ) )
    {
      break;
    }
    getline ( input, junk );
    keys.insert ( std::make_pair ( ( int )y, ( int )x ) );
  }
  input.close ();
  return keys;
}

std::vector<std::vector<float> >
generate_histogram (  const pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud )
{
  std::vector<std::vector<float> > histogram ( 30, std::vector<float> ( 120 ) );
  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      histogram[i][j] = 0;
    }
  }
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = pointCloud.begin ();
        i != pointCloud.end ();
        ++i )
  {
    float x = i->normal_x;
    float y = i->normal_y;
    float z = i->normal_z;
    float r = sqrt ( x * x + y * y + z * z );
    if ( z < 0 )
    {
      r *= -1;
    }
    if ( r == 0 )
    {
      continue;
    }
    x /= r;
    y /= r;
    z /= r;
    float phi_temp = atan2 ( y, x );
    if ( phi_temp < 0 )
    {
      phi_temp += 2 * M_PI;
    }
    int phi = ( int )( 60 * phi_temp / M_PI );
    int theta = ( int )( 60 * acos ( z ) / M_PI );
    if ( theta == 30 )
    {
      theta = 29;
    }
    if ( phi == 120 )
    {
      phi = 119;
    }
    assert ( phi < 120 && theta < 30 && theta >= 0 && phi >= 0 );
    histogram[theta][phi]++;
  }
  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      double theta_0 = M_PI * i / 60;
      double delta_phi = M_PI / 60;
      double delta_theta = M_PI / 60;
      double normalization = delta_phi * ( cos ( theta_0 ) - cos ( theta_0 + delta_theta ) );
      histogram[i][j] /= normalization;
    }
  }
  return histogram;
}

Eigen::Matrix3f
compute_axis ( const std::vector<std::vector<float> > & histogram )
{
  Eigen::Matrix3f basis;
  float max = 0;
  size_t theta_max = 0, phi_max = 0;

  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      if ( histogram[i][j] >= max )
      {
        max = histogram[i][j];
        theta_max = i;
        phi_max = j;
      }
    }
  }
  basis ( 0, 0 ) = sin ( M_PI * theta_max / 60 ) * cos ( M_PI * phi_max / 60 );
  basis ( 0, 1 ) = sin ( M_PI * theta_max / 60 ) * sin ( M_PI * phi_max / 60 );
  basis ( 0, 2 ) = cos ( M_PI * theta_max / 60 );
  double norm = sqrt ( basis ( 0, 0 ) * basis ( 0, 0 ) + basis ( 0, 1 ) * basis ( 0, 1 ) + basis ( 0,
      2 ) * basis ( 0, 2 ) );
  for ( int i = 0; i < 3; ++i )
  {
    basis ( 0, i ) /= norm;
  }
  max = 0;
  const float slack = 0.06;
  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      double xtemp = sin ( M_PI * i / 60 ) * cos ( M_PI * j / 60 );
      double ytemp = sin ( M_PI * i / 60 ) * sin ( M_PI * j / 60 );
      double ztemp = cos ( M_PI * i / 60 );
      double norm = sqrt ( xtemp * xtemp + ytemp * ytemp + ztemp * ztemp );
      xtemp /= norm;
      ytemp /= norm;
      ztemp /= norm;

      double dot = xtemp * basis ( 0, 0 ) + ytemp * basis ( 0, 1 ) + ztemp * basis ( 0, 2 );
      if ( dot > -slack && dot < slack && histogram[i][j] >= max )
      {
        max = histogram[i][j];
        basis ( 1, 0 ) = xtemp;
        basis ( 1, 1 ) = ytemp;
        basis ( 1, 2 ) = ztemp;
      }
    }
  }
  max = 0;
  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      double xtemp = sin ( M_PI * i / 60 ) * cos ( M_PI * j / 60 );
      double ytemp = sin ( M_PI * i / 60 ) * sin ( M_PI * j / 60 );
      double ztemp = cos ( M_PI * i / 60 );
      double norm = sqrt ( xtemp * xtemp + ytemp * ytemp + ztemp * ztemp );
      xtemp /= norm;
      ytemp /= norm;
      ztemp /= norm;

      double dot1 = xtemp * basis ( 0, 0 ) + ytemp * basis ( 0, 1 ) + ztemp * basis ( 0, 2 );
      double dot2 = xtemp * basis ( 1, 0 ) + ytemp * basis ( 1, 1 ) + ztemp * basis ( 1, 2 );
      if ( dot1 > -slack && dot1 < slack && dot2 > -slack && dot2 < slack && histogram[i][j] >= max )
      {
        max = histogram[i][j];
        basis ( 2, 0 ) = xtemp;
        basis ( 2, 1 ) = ytemp;
        basis ( 2, 2 ) = ztemp;
      }
    }
  }
  
  basis2rotation (basis);

  double projections [6] =
  { basis ( 0, 1 ), -basis ( 0, 1 ), basis ( 1, 1 ), -basis ( 1, 1 ), basis ( 2, 1 ), -basis ( 2, 1 ) };
  double maxValue = 0;
  int maxIndex = 0;

  for ( int i = 0; i < 6; ++i )
  {
    if ( maxValue <= projections[i] )
    {
      maxValue = projections[i];
      maxIndex = i;
    }
  }
  Eigen::Vector3f y;
  switch ( maxIndex )
  {
  case 0:
    y = Eigen::Vector3f ( 1, 0, 0 );
    break;
  case 1:
    y = Eigen::Vector3f ( -1, 0, 0 );
    break;
  case 2:
    y = Eigen::Vector3f ( 0, 1, 0 );
    break;
  case 3:
    y = Eigen::Vector3f ( 0, -1, 0 );
    break;
  case 4:
    y = Eigen::Vector3f ( 0, 0, 1 );
    break;
  case 5:
    y = Eigen::Vector3f ( 0, 0, -1 );
    break;
  }
  Eigen::Vector3f up ( 0, 1, 0 );
  Eigen::Vector3f axis = y.cross ( up );
  double angle = acos ( y.dot ( up ) );
  Eigen::Matrix3f Ex;
  Ex << 0, -axis[2], axis[1],
  axis[2], 0, -axis[0],
  -axis[1], axis[0], 0;
  Eigen::Matrix3f R = cos ( angle ) * Eigen::Matrix3f::Identity () +
                      ( 1 - cos ( angle ) ) * axis * axis.transpose () + sin ( angle ) * Ex;

  basis = R * basis;
  return basis;
}

void
project_onto_basis ( pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud,
                     const Eigen::Matrix3f &                   basis )
{
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator i = pointCloud.begin ();
        i != pointCloud.end ();
        ++i )
  {
    Eigen::Vector3f normal ( i->normal_x, i->normal_y, i->normal_z );
    normal = basis * normal;
    i->normal_x = normal [0];
    i->normal_y = normal [1];
    i->normal_z = normal [2];
    Eigen::Vector3f point ( i->x, i->y, i->z );
    point = basis * point;
    i->x = point [0];
    i->y = point [1];
    i->z = point [2];
  }
}

void
saveHistogram ( const std::string &                      filename,
                const std::vector<std::vector<float> > & histogram,
                const Eigen::Matrix3f &                  basis )
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  float max = 0;
  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      if ( histogram[i][j] > max )
      {
        max = histogram[i][j];
      }
    }
  }
  for ( size_t i = 0; i < histogram.size (); ++i )
  {
    for ( size_t j = 0; j < histogram[i].size (); ++j )
    {
      pcl::PointXYZRGBNormal point;
      point.x = sin ( M_PI * i / 60 ) * cos ( M_PI * j / 60 );
      point.y = sin ( M_PI * i / 60 ) * sin ( M_PI * j / 60 );
      point.z = cos ( M_PI * i / 60 );
      double norm = sqrt ( point.x * point.x + point.y * point.y + point.z * point.z );
      point.x /= norm;
      point.y /= norm;
      point.z /= norm;
      point.normal_x = point.x * histogram[i][j] / max;
      point.normal_y = point.y * histogram[i][j] / max;
      point.normal_z = point.z * histogram[i][j] / max;
      RgbConverter c;
      c.g = ( uchar )( 255 * histogram[i][j] / max );
      c.r = 0;
      c.b = 0;
      point.rgb = c.rgb;
      cloud.push_back ( point );
    }
  }
  project_onto_basis ( cloud, basis );
  savePlyFile ( filename, cloud );
}

Eigen::Matrix3f
reorient ( pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud )
{
  std::vector<std::vector<float> > histogram = generate_histogram ( pointCloud );
  Eigen::Matrix3f basis = compute_axis ( histogram );

  std::cout << "Basis " << std::endl;
  std::cout << basis << std::endl;


  project_onto_basis ( pointCloud, basis );
  saveHistogram ( "/home/kmatzen/NOBACKUP/histogram.ply", histogram, basis );

  return basis;
}

void
findAABB ( const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
           const Eigen::Matrix3f &                         basis,
           const std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> &               clouds,
           Eigen::Vector3f &                               min,
           Eigen::Vector3f &                               max )
{
  min = Eigen::Vector3f ( std::numeric_limits<float>::infinity (),
    std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity () );
  max = Eigen::Vector3f ( -std::numeric_limits<float>::infinity (),
    -std::numeric_limits<float>::infinity (), -std::numeric_limits<float>::infinity () );
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin (); i != points.end (); ++i )
  {
    if ( i->x < min[0] )
    {
      min[0] = i->x;
    }
    if ( i->y < min[1] )
    {
      min[1] = i->y;
    }
    if ( i->z < min[2] )
    {
      min[2] = i->z;
    }
    if ( i->x > max[0] )
    {
      max[0] = i->x;
    }
    if ( i->y > max[1] )
    {
      max[1] = i->y;
    }
    if ( i->z > max[2] )
    {
      max[2] = i->z;
    }
  }

  for ( size_t i = 0; i < clouds.size (); ++i )
  {
    Eigen::Vector3f cameraOrigin (clouds[i]->sensor_origin_[0], clouds[i]->sensor_origin_[1], clouds[i]->sensor_origin_[2]);
    Eigen::Vector3f position = basis * cameraOrigin;
    for ( int j = 0; j < 3; ++j )
    {
      if ( max [j] < position [j] )
      {
        max[j] = position [j];
      }
      if ( min[j] > position [j] )
      {
        min[j] = position [j];
      }
    }
  }
}

struct Options
{
  std::string manifest;
  std::string surface;
  std::string output;
  double      voxelSize;
  double      mlsRadius;

  void
              help () const
  {
    std::cout << "  --manifest [string]" << std::endl;
    std::cout << "  --output [string]" << std::endl;
    std::cout << "  --surface [string]" << std::endl;
    std::cout << "  --voxel_size [double]" << std::endl;
    std::cout << "  --mls_radius [double]" << std::endl;
    std::cout << "  --help" << std::endl;
    exit ( 1 );
  }

  Options ( int argc, char * * argv )
  {
    int c;
    int indexptr;

    struct option longopts [15] =
    { { "manifest",       required_argument,         0,                         'm'                                         },
      { "output",       required_argument,         0,                         'o'                                         },
      { "voxel_size",   required_argument,         0,                         'v'                                         },
      { "mls_radius",   required_argument,         0,                         'r'                                         },
      { "surface" , required_argument, 0, 's'},
      { 0,              0,                         0,                         0                                           } };

    bool manifestFileSet = false;
    bool outputSet = false;
    bool voxelSizeSet = false;
    bool mlsRadiusSet = false;
    bool surfaceSet = false;

    while ( ( c = getopt_long ( argc, argv, "o:v:m:r:", longopts, &indexptr ) ) != -1 )
    {
      std::stringstream ss;
      switch ( c )
      {
      case 's':
        surfaceSet = true;
        surface = std::string (optarg);
        break;
      case 'm':
        manifestFileSet = true;
        manifest = std::string ( optarg );
        break;
      case 'o':
        outputSet = true;
        output = std::string ( optarg );
        break;
      case 'v':
        voxelSizeSet = true;
        ss << optarg;
        ss >> voxelSize;
        if ( !ss )
        {
          std::cout << "Failed to parse voxel size parameter" << std::endl;
          help ();
        }
        break;
      case 'r':
        mlsRadiusSet = true;
        ss << optarg;
        ss >> mlsRadius;
        if ( !ss )
        {
          std::cout << "Failed to parse MLS radius parameter" << std::endl;
          help ();
        }
        break;
      default:
        std::cout << "Unknown option " << c << std::endl;
        help ();
        break;
      }
    }

    if ( !manifestFileSet )
    {
      std::cout << "manifest option is required" << std::endl;
      help ();
    }
    if (!surfaceSet)
    {
      std::cout << "surface option is required" << std::endl;
      help();
    }
    if ( !outputSet )
    {
      std::cout << "output option is required" << std::endl;
      help ();
    }
    if ( !voxelSizeSet )
    {
      std::cout << "voxel size option is required" << std::endl;
      help ();
    }
    if ( !mlsRadiusSet )
    {
      std::cout << "MLS radius option is required" << std::endl;
      help ();
    }
  }
};

Eigen::Vector3i
material2Voxel ( const Eigen::Vector3d & material, float length, const Eigen::Vector3d & translation )
{
  Eigen::Vector3d result = 1.0 / length * ( material - translation );

  return Eigen::Vector3i ( ( int )result [0], ( int )result[1], ( int )result [2] );
}

Eigen::Vector3f
voxel2Material ( const Eigen::Vector3i & voxel, float length, const Eigen::Vector3f & translation )
{
  return length * ( ( voxel.cast<float>() + Eigen::Vector3f ( 0.5, 0.5, 0.5 ) ) ) + translation;
}

void
carveVoxels ( const pcl::PointCloud<pcl::PointXYZRGBNormal> &                 points,
              const std::vector<std::vector<std::vector<uint64_t> > > & surfaceVoxels,
              const Eigen::Matrix3f &                                         basis,
              std::vector<std::vector<std::vector<std::pair<uint64_t,
                                                            uint64_t> > > > & voxels,
              float                                                           length,
              const Eigen::Vector3f &                                         translation,
              const Eigen::Vector3i &                                         dimensions/*,
              std::vector<RayClique> & rayCliques */)
{
  assert ( ( int )voxels.size ( ) == dimensions[0] );
  assert ( ( int )voxels[0].size ( ) == dimensions[1] );
  assert ( ( int )voxels[0][0].size ( ) == dimensions[2] );
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin (); i != points.end (); ++i )
  {
    Eigen::Vector3d cameraPosition (points.sensor_origin_[0], points.sensor_origin_[1], points.sensor_origin_[2]);
    cameraPosition = basis.cast<double>() * cameraPosition;
    Eigen::Vector3d destination ( i->x, i->y, i->z );
    destination = basis.cast<double>() * Eigen::Matrix3f(points.sensor_orientation_).cast<double>() * destination + cameraPosition;
    Eigen::Vector3d direction = destination - cameraPosition;
    Eigen::Vector3d tDelta = direction.normalized ();
    Eigen::Vector3i source = material2Voxel ( cameraPosition, ( double )length, translation.cast<double>() );
    Eigen::Vector3i sink = material2Voxel ( destination, ( double )length, translation.cast<double>() );
    Eigen::Vector3d positionInGrid = 1.0 / length * ( cameraPosition - translation.cast<double>() );
    Eigen::Vector3d destPositionInGrid = 1.0 / length * ( destination - translation.cast<double>() );
    Eigen::Vector3d tMax;
    Eigen::Vector3i increment;
    Eigen::Vector3d diff = destPositionInGrid - positionInGrid;
    unsigned int flip = 0;
    assert (
      source[0] >= 0 && source [1] >= 0 && source [2] >= 0 && source [0] < dimensions [0] && source [1] <
      dimensions [1] && source [2] < dimensions [2] );
    if ( sink[0] < 0 || sink [1] < 0 || sink [2] < 0 || sink [0] >= dimensions [0] || sink [1] >=
         dimensions [1] || sink [2] >= dimensions [2] )
    {
      continue;
    }
    RayClique clique;
    RgbConverter c;
    c.rgb = i->rgb;
    clique.color[0] = c.r;
    clique.color[1] = c.g;
    clique.color[2] = c.b;
    for ( int i = 0; i < 3; ++i )
    {
      tDelta [i] = fabs ( 1.0 / tDelta[i] );
      increment [i] = ( int )( direction [i] / fabs ( direction[i] ) );
      int nextBarrier = increment [i] > 0 ? ( int )ceil ( positionInGrid [i] ) : ( int )floor (
        positionInGrid [i] );
      tMax [i] = fabs ( nextBarrier - positionInGrid [i] );
    }

    bool invisible = true;
    double closest = std::numeric_limits<double>::infinity ();
    clique.point = 0;
    while ( true )
    {
      int minIndex = 0;
      double min = std::numeric_limits<double>::infinity ();
      for ( int i = 0; i < 3; ++i )
      {
        if ( tMax [i] < min )
        {
          minIndex = i;
          min = tMax[i];
        }
      }
      source [minIndex] += increment [minIndex];
      if ( source[minIndex] < 0 || source [minIndex] >= dimensions [minIndex] )
      {
        break;
      }

      tMax[minIndex] += tDelta[minIndex];

      assert (
        source [0] >= 0 && source [1] >= 0 && source [2] >= 0 && source [0] < ( int )voxels.size () &&
        source [1] < ( int )voxels[0].size () && source [2] < ( int )voxels[0][0].size () );
      std::pair<uint64_t, uint64_t> & voxel = voxels [source [0]][source [1]][source [2]];
      if (flip % 2 == 0)
      {
#pragma omp atomic
        ++voxel.first;
      }
      else
      {
#pragma omp atomic
        ++voxel.second;
      }
      if (surfaceVoxels[source[0]][source[1]][source[2]])
      {
        flip += surfaceVoxels[source[0]][source[1]][source[2]];
      }
      if ( invisible )
      {
//#pragma omp atomic
//        ++voxel.first;
        double dist = ( sink - source ).cast<double>().norm ();
        if ( dist < closest )
        {
          closest = dist;
        }
        if ( dist < 1.42 )
        {
          invisible = false;
        }
        ++clique.point;
      }
//#pragma omp atomic
//      ++voxel.second;
      clique.voxels.push_back (source[0]*dimensions[1]*dimensions[2] + source[1]*dimensions[2] + source[2]);
    }
    if ( invisible )
    {
#pragma omp critical
      {
        std::cout << "Missed, but the closest was " << closest << std::endl;
        std::cout << "Dimensions " << std::endl;
        std::cout << dimensions << std::endl;
        std::cout << "Source " << std::endl;
        std::cout << cameraPosition << std::endl;
        std::cout << "Destination " << std::endl;
        std::cout << destination << std::endl;
        std::cout << "Source voxel " << std::endl;
        std::cout << positionInGrid << std::endl;
        std::cout << "Destination voxel " << std::endl;
        std::cout << destPositionInGrid << std::endl;
//      exit (1);
      }
    }
    else
    {
    //  #pragma omp critical
     // rayCliques.push_back (clique);
    }
  }
}

void
fillVoxelColors (const pcl::PointCloud<pcl::PointXYZRGBNormal> & points, const Eigen::Matrix3f & basis, std::vector<std::vector<std::vector<std::pair<uint64_t,uint64_t> > > > & r, std::vector<std::vector<std::vector<std::pair<uint64_t,uint64_t> > > > & g, std::vector<std::vector<std::vector<std::pair<uint64_t,uint64_t> > > > & b, float length, const Eigen::Vector3f & translation, const Eigen::Vector3i & dimensions)
{
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    RgbConverter c;
    c.rgb = i->rgb;
    Eigen::Vector3d material (i->x, i->y, i->z);
    Eigen::Vector3d origin (points.sensor_origin_[0], points.sensor_origin_[1], points.sensor_origin_[2]);
    material = basis.cast<double>() * (Eigen::Matrix3f (points.sensor_orientation_).cast<double>() * material + origin);
    Eigen::Vector3i voxel = material2Voxel (material, length, translation.cast<double>());
    if ( voxel[0] < 0 || voxel [1] < 0 || voxel [2] < 0 || voxel [0] >= dimensions [0] || voxel [1] >=
         dimensions [1] || voxel [2] >= dimensions [2] )
    {
      continue;
    }
    std::pair<uint64_t,uint64_t> & rvoxel = r[voxel[0]][voxel[1]][voxel[2]];
    std::pair<uint64_t,uint64_t> & gvoxel = g[voxel[0]][voxel[1]][voxel[2]];
    std::pair<uint64_t,uint64_t> & bvoxel = b[voxel[0]][voxel[1]][voxel[2]];
#pragma omp atomic
    rvoxel.first += c.r;
#pragma omp atomic
    ++rvoxel.second;
#pragma omp atomic
    gvoxel.first += c.g;
#pragma omp atomic
    ++gvoxel.second;
#pragma omp atomic
    bvoxel.first += c.b;
#pragma omp atomic
    ++bvoxel.second;
  }
}

void
saveRayMRF (const std::string & filename, const Eigen::Vector3i & dimensions, const std::vector<RayClique> & rayCliques)
{
  std::ofstream output (filename.c_str());
  if (!output)
  {
    std::cout << "Failed to open " << filename << " for write." << std::endl;
    exit (1);
  }

  output << dimensions [0] << " " << dimensions [1] << " " << dimensions [2] << std::endl;
  output << rayCliques.size() << std::endl;
  for (size_t i = 0; i < rayCliques.size(); ++i)
  {
    const RayClique & ray = rayCliques [i];
    output << ray.color[0] << " " << ray.color[1] << " " << ray.color[2] << " ";
    output << ray.point << " ";
    output << ray.voxels.size() << " ";
    for (size_t j = 0; j < ray.voxels.size(); ++j)
    {
      output << ray.voxels [j] << " ";
    }
    output << std::endl;
  }
  output.close();
}

void
surfaceToVoxels (const pcl::PointCloud<pcl::PointXYZ> & surfacePoints, const Eigen::Matrix3f & basis, float length, const Eigen::Vector3f & translation, std::vector<std::vector<std::vector<uint64_t> > >& surfaceVoxels)
{
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator i = surfacePoints.begin(); i != surfacePoints.end(); ++i)
  {
    Eigen::Vector3f point (i->x, i->y, i->z);
    point = basis * point;
    Eigen::Vector3i voxel = material2Voxel (point.cast<double>(), length, translation.cast<double>());
    ++surfaceVoxels [voxel[0]][voxel[1]][voxel[2]];
  }
}

int
main ( int argc, char * * argv )
{
  TIMED

  printCommand ( argc, argv );

  TIME_BEGIN ( "Loading options" )
  Options options ( argc, argv );
  TIME_END

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combinedCloud ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );

  TIME_BEGIN ( "Loading manifest file" )
  std::ifstream cloudsFile (options.manifest.c_str());
  if (!cloudsFile)
  {
    std::cout << "Failed to open " << options.manifest << " for read." << std::endl;
    options.help();
  }
  std::vector<std::string> cloudNames;
  size_t slash = options.manifest.find_last_of ("/");
  std::string baseDirectory = options.manifest.substr (0, slash + 1);
  while (cloudsFile)
  {
    std::string filename;
    if (!(cloudsFile >> filename))
    {
      break;
    }
    std::stringstream ss;
    ss << baseDirectory << filename;
    cloudNames.push_back (ss.str());
  }
  cloudsFile.close();
  TIME_END

  TIME_BEGIN ( "Loading all point clouds" )
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clouds (cloudNames.size());
#pragma omp parallel for
  for ( int i = 0; i < ( int )cloudNames.size (); ++i )
  {
    const std::string & filename = cloudNames [i];
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & points = clouds [i];
#pragma omp critical
    {
      std::cout << "Opening " << filename << std::endl;
      points = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    }
    pcl::PointCloud<pcl::PointSurfel> tempPoints;
    pcl::io::loadPCDFile (filename, tempPoints);
    //std::cout << Eigen::Matrix3f(tempPoints.sensor_orientation_) << std::endl;
    //std::cout << tempPoints.sensor_origin_ << std::endl;
    for (size_t w = 0; w < tempPoints.width; w += 2)
    {
    for (size_t h = 0; h < tempPoints.height; h += 2)
    {
      const pcl::PointSurfel p = tempPoints (w, h);
      if (isnan(p.x) || isnan(p.y) || isnan(p.z))
      {
        continue;
      }
      if (isinf(p.x) || isinf(p.y) || isinf(p.z))
      {
        std::cout << filename << " has inf" << std::endl;
        exit (1);
      }
      Eigen::Vector3f point (p.x, p.y, p.z);
      Eigen::Vector3f origin(tempPoints.sensor_origin_[0], tempPoints.sensor_origin_[1], tempPoints.sensor_origin_[2]);
      Eigen::Vector3f normal (p.normal_x, p.normal_y, p.normal_z);
      point = Eigen::Matrix3f(tempPoints.sensor_orientation_) * point + origin;
      pcl::PointXYZRGBNormal copy;
      copy.x = p.x;
      copy.y = p.y;
      copy.z = p.z;
      RgbConverter c;
      ByteExtractor b;
      b.rgba = p.rgba;
      c.r = b.b[0];
      c.g = b.b[1];
      c.b = b.b[2];
      copy.rgb = c.rgb;
      normal.normalize();
      copy.normal_x = normal[0];
      copy.normal_y = normal[1];
      copy.normal_z = normal[2];
      points->push_back (copy);
      copy.x = point[0];
      copy.y = point[1];
      copy.z = point[2];
      normal = Eigen::Matrix3f(tempPoints.sensor_orientation_)*normal;
      copy.normal_x = normal[0];
      copy.normal_y = normal[1];
      copy.normal_z = normal[2];
#pragma omp critical
      combinedCloud->push_back (copy);
    }
    }
    points->sensor_orientation_ = tempPoints.sensor_orientation_;
    points->sensor_origin_ = tempPoints.sensor_origin_;
  }
  TIME_END

  TIME_BEGIN ( "Saving unprocessed ply" )
  savePlyFile ( options.output + ".unprocessed.ply", *combinedCloud );
  TIME_END

  TIME_BEGIN ( "Validating data" )
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = combinedCloud->begin ();
        i != combinedCloud->end ();
        ++i )
  {
    if ( isnan ( i->x ) )
    {
      std::cout << "NaN found in x" << std::endl;
    }
    if ( isnan ( i->y ) )
    {
      std::cout << "NaN found in y" << std::endl;
    }
    if ( isnan ( i->z ) )
    {
      std::cout << "NaN found in z" << std::endl;
    }
    if ( isinf ( i->x ) )
    {
      std::cout << "Inf found in x" << std::endl;
    }
    if ( isinf ( i->y ) )
    {
      std::cout << "Inf found in y" << std::endl;
    }
    if ( isinf ( i->z ) )
    {
      std::cout << "Inf found in z" << std::endl;
    }
  }
  TIME_END

  pcl::VoxelGrid<pcl::PointXYZRGBNormal> downsampler;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reduced ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );
  downsampler.setInputCloud ( combinedCloud );
  downsampler.setLeafSize ( options.voxelSize, options.voxelSize, options.voxelSize );
  std::cout << "Data size is " << combinedCloud->size () << std::endl;
  TIME_BEGIN ( "Downsampling data with voxel grid" )
  downsampler.filter ( *reduced );
  TIME_END

  TIME_BEGIN ( "Saving downsampled ply" )
  savePlyFile ( options.output + ".downsampled.ply", *reduced );
  TIME_END

  pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> ());
  tree->setInputCloud ( reduced );
  pcl::PointCloud <pcl::PointNormal>::Ptr normals ( new pcl::PointCloud<pcl::PointNormal>() );
  pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointNormal> normalEstimation;
  normalEstimation.setInputCloud ( reduced );
  normalEstimation.setOutputNormals ( normals );
  normalEstimation.setSearchRadius ( options.mlsRadius );
  normalEstimation.setSearchMethod ( tree );
  normalEstimation.setPolynomialOrder (1);
  pcl::PointCloud<pcl::PointXYZRGBNormal> cleaned;
  std::cout << "Data size is " << reduced->size () << std::endl;
  TIME_BEGIN ( "Reconstructing normals with MLS" )
  normalEstimation.reconstruct ( cleaned );
  TIME_END

  TIME_BEGIN ( "Putting together point cloud" )
  pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator k;
  pcl::PointCloud<pcl::PointNormal>::const_iterator j;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  for ( i = cleaned.begin (), j = normals->begin (), k = reduced->begin ();
        i != cleaned.end ();
        ++i, ++j, ++k )
  {
    double dot = k->normal_x * j->normal_x + k->normal_y * j->normal_y + k->normal_z * j->normal_z;
    double length = j->normal_x * j->normal_x + j->normal_y * j->normal_y + j->normal_z * j->normal_z;
    double flip = ( dot > 0 ) ? 1.0 / length : -1.0 / length;
    if ( length == 0 )
    {
      continue;
    }
    pcl::PointXYZRGBNormal newPoint = *k;
    newPoint.normal_x = flip * j->normal_x;
    newPoint.normal_y = flip * j->normal_y;
    newPoint.normal_z = flip * j->normal_z;
    newPoint.x = i->x;
    newPoint.y = i->y;
    newPoint.z = i->z;
    final->push_back ( newPoint );
  }
  TIME_END

  TIME_BEGIN ( "Reorienting point cloud" )
  Eigen::Matrix3f basis = reorient ( *final );
  TIME_END

  std::cout << "Basis is " << std::endl;
  std::cout << basis << std::endl;
  std::cout << std::endl;

  TIME_BEGIN ( "Saving ply" )
  savePlyFile ( options.output + ".ply", *final );
  TIME_END

  TIME_BEGIN ( "Saving pcd" )
  pcl::io::savePCDFile ( options.output + ".pcd", *final, true );
  TIME_END

  std::cout << "Beginning visibility voxelization." << std::endl;

  Eigen::Vector3f min, max;

  TIME_BEGIN ( "Finding AABB" )
  findAABB ( *final, basis, clouds, min, max );
  TIME_END

  min -= Eigen::Vector3f (options.voxelSize, options.voxelSize, options.voxelSize);
  max += Eigen::Vector3f (options.voxelSize, options.voxelSize, options.voxelSize);

  final.reset();

  std::cout << "AABB is " << std::endl;
  std::cout << min << std::endl;
  std::cout << "-" << std::endl;
  std::cout << max << std::endl;
  std::cout << std::endl;

  Eigen::Vector3f width = max - min;
  Eigen::Vector3i dimensions = ( 1.0 / options.voxelSize * width + Eigen::Vector3f ( 1, 1, 1 ) ).cast<int>();

  pcl::PointCloud<pcl::PointXYZ> surfacePoints;
  loadPlyFile (options.surface, surfacePoints);
  KVO voxels ( dimensions [0], dimensions [1], dimensions [2], options.voxelSize, min );
  std::vector<std::vector<std::vector<uint64_t > > > surfaceVoxels 
    ( voxels.size ( 0 ), std::vector<std::vector<uint64_t> > ( voxels.size ( 1 ), std::vector<uint64_t> ( voxels.size ( 2 ) ) ) );
  surfaceToVoxels (surfacePoints, basis, options.voxelSize, min, surfaceVoxels);

  KVO unnormalizedVoxels ( dimensions [0], dimensions [1], dimensions [2], options.voxelSize, min );
  std::vector<std::vector<std::vector<std::pair<uint64_t, uint64_t> > > > tempa
    ( voxels.size ( 0 ), std::vector<std::vector<std::pair<uint64_t, uint64_t> > > ( voxels.size (
                                                                                       1 ),
                                                                                     std::vector<std::pair<
                                                                                                   uint64_t, uint64_t> > ( voxels.size ( 2 ) ) ) );
  std::vector<std::vector<std::vector<std::pair<uint64_t, uint64_t> > > > tempr
    ( voxels.size ( 0 ), std::vector<std::vector<std::pair<uint64_t, uint64_t> > > ( voxels.size (

       1 ),

     std::vector<std::pair<

                   uint64_t, uint64_t> > ( voxels.size ( 2 ) ) ) );
  std::vector<std::vector<std::vector<std::pair<uint64_t, uint64_t> > > > tempg
    ( voxels.size ( 0 ), std::vector<std::vector<std::pair<uint64_t, uint64_t> > > ( voxels.size (

       1 ),

     std::vector<std::pair<

                   uint64_t, uint64_t> > ( voxels.size ( 2 ) ) ) );
  std::vector<std::vector<std::vector<std::pair<uint64_t, uint64_t> > > > tempb
    ( voxels.size ( 0 ), std::vector<std::vector<std::pair<uint64_t, uint64_t> > > ( voxels.size (

       1 ),

     std::vector<std::pair<

                   uint64_t, uint64_t> > ( voxels.size ( 2 ) ) ) );

  std::vector <RayClique> rayCliques;

  TIME_BEGIN ( "Visibility ray tracing" )
#pragma omp parallel for
  for ( int i = 0; i < ( int )clouds.size (); ++i )
  {
    carveVoxels ( *clouds[i], surfaceVoxels, basis, tempa, options.voxelSize, min, dimensions);//, rayCliques);
    fillVoxelColors (*clouds[i], basis, tempr, tempg, tempb, options.voxelSize, min, dimensions);
  }
  TIME_END

/*  TIME_BEGIN ("Saving Ray MRF cliques")
  saveRayMRF (options.output + "raymrf", dimensions, rayCliques);
  TIME_END
*/
  TIME_BEGIN ( "Building final voxel grid" )
#pragma omp parallel for
  for ( int i = 0; i < ( int )tempa.size (); ++i )
  {
    for ( int j = 0; j < ( int )tempa[i].size (); ++j )
    {
      for ( int k = 0; k < ( int )tempa[i][j].size (); ++k )
      {
        voxels [i][j][k].a = log(( float )tempa[i][j][k].first) - log((float)tempa[i][j][k].second);
        voxels [i][j][k].r = (float)tempr[i][j][k].first / tempr[i][j][k].second;
        voxels [i][j][k].g = (float)tempg[i][j][k].first / tempg[i][j][k].second;
        voxels [i][j][k].b = (float)tempb[i][j][k].first / tempb[i][j][k].second;
        unnormalizedVoxels [i][j][k].a = ( float )tempa[i][j][k].first;
        unnormalizedVoxels [i][j][k].r = (float)tempr[i][j][k].first / tempr[i][j][k].second;
        unnormalizedVoxels [i][j][k].g = (float)tempg[i][j][k].first / tempg[i][j][k].second;
        unnormalizedVoxels [i][j][k].b = (float)tempb[i][j][k].first / tempb[i][j][k].second;
      }
    }
  }
  TIME_END

  TIME_BEGIN ( "Saving unnormalized voxels" )
  voxels.save ( options.output + ".unnormalized.kvo" );
  TIME_END

  TIME_BEGIN ( "Saving voxels" )
  voxels.save ( options.output + ".kvo" );
  TIME_END
}
