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

void
printCommand ( int argc, char * * argv )
{
  for ( int i = 0; i < argc; ++i )
  {
    std::cout << argv [i] << " ";
  }
  std::cout << std::endl;
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

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr LoadCloud (
  const std::string &     colorFilename,
  const std::string &     depthFilename,
  const std::string &     keypointFilename,
  const BundleCamera &    camera,
  float                   scale,
  int                     downscale,
  const Eigen::Matrix3f & basis = Eigen::Matrix3f::Identity () )
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );

  cv::Mat colorImageDist = cv::imread ( colorFilename );

  std::ifstream depthInput ( depthFilename.c_str () );
  if ( !depthInput )
  {
    std::cout << "Could not load file " << depthFilename << std::endl;
    throw std::exception ();
  }
  uint32_t rows, cols;
  if ( !depthInput.read ( ( char * )&rows, sizeof ( uint32_t ) ) )
  {
    std::cout << "Could not read rows in file " << depthFilename << std::endl;
    throw std::exception ();
  }
  if ( !depthInput.read ( ( char * )&cols, sizeof ( uint32_t ) ) )
  {
    std::cout << "Could not read cols in file " << depthFilename << std::endl;
    throw std::exception ();
  }
  cv::Mat1f depthMapDist ( rows, cols );
  if ( !depthInput.read ( ( char * )depthMapDist.data, depthMapDist.rows * depthMapDist.cols * sizeof ( float ) ) )
  {
    std::cout << "Could not read data in file " << depthFilename << std::endl;
    throw std::exception ();
  }
  depthInput.close ();
  cv::Mat cameraMatrixCV ( 3, 3, CV_32FC1 );
  cameraMatrixCV.at<float> ( 0, 0 ) = camera.GetF ();
  cameraMatrixCV.at<float> ( 0, 1 ) = 0;
  cameraMatrixCV.at<float> ( 0, 2 ) = ( float )colorImageDist.cols / 2;
  cameraMatrixCV.at<float> ( 1, 0 ) = 0;
  cameraMatrixCV.at<float> ( 1, 1 ) = camera.GetF ();
  cameraMatrixCV.at<float> ( 1, 2 ) = ( float )colorImageDist.rows / 2;
  cameraMatrixCV.at<float> ( 2, 0 ) = 0;
  cameraMatrixCV.at<float> ( 2, 1 ) = 0;
  cameraMatrixCV.at<float> ( 2, 2 ) = 1;

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << camera.GetF (), 0, ( float )colorImageDist.cols / 2,
  0, camera.GetF (), ( float )colorImageDist.rows / 2,
  0, 0, 1;

  cv::Mat distCoeffs ( 5, 1, CV_32FC1 );
  distCoeffs.at<float> ( 0, 0 ) = camera.GetK1 ();
  distCoeffs.at<float> ( 1, 0 ) = camera.GetK2 ();
  distCoeffs.at<float> ( 2, 0 ) = 0;
  distCoeffs.at<float> ( 3, 0 ) = 0;
  distCoeffs.at<float> ( 4, 0 ) = 0;

  cv::Mat colorImage, depthMap;

  cv::undistort ( colorImageDist, colorImage, cameraMatrixCV, distCoeffs );

  std::set<std::pair<int, int> > keys = LoadKeys ( keypointFilename );

  //Linear interpolation; do not want
  //cv::undistort (depthMapDist, depthMap, cameraMatrix, distCoeffs);

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap ( cameraMatrixCV, distCoeffs, cv::Mat::eye ( 3,
      3,
      CV_32FC1 ), cameraMatrixCV, depthMapDist.size (), CV_32FC1, map1, map2 );
  cv::remap ( depthMapDist, depthMap, map1, map2, cv::INTER_NEAREST );

  for ( int j = 0; j < depthMap.rows; j += downscale )
  {
    for ( int i = 0; i < depthMap.cols; i += downscale )
    {
      float depth = depthMap.at<float> ( j, i );
      if ( depth == 0 )
      {
        continue;
      }

      depth = -0.018 * depth * depth + 1.0038 * depth + 0.005;
      Eigen::Vector3f p ( ( float )i, ( float )depthMap.rows - 1.0 - ( float )j, 1 );
      p = depth * p;

      p = cameraMatrix.inverse () * p;
      p[2] *= -1;

      const Eigen::Matrix3f & R = camera.GetR ();
      const Eigen::Vector3f & t = camera.GetT ();
      p = p - t / scale;
#pragma omp critical
      p = basis * R.transpose () * p;

      Eigen::Vector3f p_cam ( 0, 0, 0 );
      p_cam = cameraMatrix.inverse () * p_cam;
      p_cam[2] *= -1;

      p_cam = p_cam - t / scale;
#pragma omp critical
      p_cam = basis * R.transpose () * p_cam;
      Eigen::Vector3f diff = p_cam - p;

      pcl::PointXYZRGBNormal point;
      point.x = p[0];
      point.y = p[1];
      point.z = p[2];

      if ( isnan ( point.x ) || isnan ( point.y ) || isnan ( point.z ) )
      {
        std::cout << "NaN found in point cloud" << std::endl;
        std::cout << "Point: " << p << std::endl;
        std::cout << "i: " << i << std::endl;
        std::cout << "j: " << j << std::endl;
        std::cout << "depth: " << depth << std::endl;
        std::cout << "z-buffer: " << depthMap.at<float> ( j, i ) << std::endl;
        std::cout << "R: " << R << std::endl;
        std::cout << "t: " << t << std::endl;
      }

      point.normal_x = diff[0];
      point.normal_y = diff[1];
      point.normal_z = diff[2];

      if ( keys.count ( std::make_pair ( j, i ) ) )
      {
        RgbConverter c;
        c.r = 255;
        c.g = 0;
        c.b = 0;
        point.rgb = c.rgb;
      }
      else
      {
        const cv::Vec3b & color = colorImage.at<cv::Vec3b>( j, i );
        RgbConverter c;
        c.r = color[2];
        c.g = color[1];
        c.b = color[0];
        point.rgb = c.rgb;
      }
      points->push_back ( point );
    }
  }
  return points;
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

  std::cout << "Rotation matrice's det is " << basis.determinant() << std::endl;
}

Eigen::Matrix3f
reorient ( pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud )
{
  std::vector<std::vector<float> > histogram = generate_histogram ( pointCloud );
  Eigen::Matrix3f basis = compute_axis ( histogram );

  std::cout << "Basis " << std::endl;
  std::cout << basis << std::endl;

  basis2rotation (basis);

  project_onto_basis ( pointCloud, basis );
  saveHistogram ( "/home/kmatzen/NOBACKUP/histogram.ply", histogram, basis );

  return basis;
}

void
findAABB ( const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
           const Eigen::Matrix3f &                         basis,
           const std::vector<BundleCamera> &               cameras,
           float                                           scale,
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

  for ( size_t i = 0; i < cameras.size (); ++i )
  {
    if ( cameras[i].IsValid () )
    {
      Eigen::Vector3f position = -basis * cameras[i].GetR ().transpose () * cameras[i].GetT () / scale;
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
}

struct Options
{
  std::string bundleFile;
  std::string listFile;
  std::string output;
  bool        drawKeypoints;
  std::string inputCloud;
  double      depthTuning;
  double      stdThresh;
  double      voxelSize;
  double      mlsRadius;
  bool        reprocess;
  bool        postProcess;
  int         downscale;
  bool        validateData;
  bool        voxelize;

  void
              help () const
  {
    std::cout << "  --bundle [string]" << std::endl;
    std::cout << "  --list [string]" << std::endl;
    std::cout << "  --output [string]" << std::endl;
    std::cout << "  --keypoints" << std::endl;
    std::cout << "  --depth_tuning [double]" << std::endl;
    std::cout << "  --input_cloud [string]" << std::endl;
    std::cout << "  --postprocess" << std::endl;
    std::cout << "  --reprocess" << std::endl;
    std::cout << "  --std_thresh [double]" << std::endl;
    std::cout << "  --voxel_size [double]" << std::endl;
    std::cout << "  --mls_radius [double]" << std::endl;
    std::cout << "  --downscale [int]" << std::endl;
    std::cout << "  --validate_data" << std::endl;
    std::cout << "  --voxelize" << std::endl;
    std::cout << "  --help" << std::endl;
    exit ( 1 );
  }

  Options ( int argc, char * * argv )
  {
    int c;
    int indexptr;

    struct option longopts [15] =
    { { "bundle",       required_argument,         0,                         'b'                                         },
      { "list",         required_argument,         0,                         'l'                                         },
      { "output",       required_argument,         0,                         'o'                                         },
      { "keypoints",    no_argument,               0,                         'k'                                         },
      { "depth_tuning", required_argument,         0,                         'd'                                         },
      { "input_cloud",  required_argument,         0,                         'c'                                         },
      { "std_thresh",   required_argument,         0,                         's'                                         },
      { "voxel_size",   required_argument,         0,                         'v'                                         },
      { "mls_radius",   required_argument,         0,                         'm'                                         },
      { "downscale",    required_argument,         0,                         'x'                                         },
      { 0,              0,                         0,                         0                                           } };

    bool bundleFileSet = false;
    bool listFileSet = false;
    bool outputSet = false;
    bool inputCloudSet = false;
    bool depthTuningSet = false;
    bool stdThreshSet = false;
    bool voxelSizeSet = false;
    bool mlsRadiusSet = false;
    bool downscaleSet = false;

    drawKeypoints = false;

    while ( ( c = getopt_long ( argc, argv, "b:l:o:kd:c:s:v:m:x:", longopts, &indexptr ) ) != -1 )
    {
      std::stringstream ss;
      switch ( c )
      {
      case 'b':
        bundleFileSet = true;
        bundleFile = std::string ( optarg );
        break;
      case 'l':
        listFileSet = true;
        listFile = std::string ( optarg );
        break;
      case 'o':
        outputSet = true;
        output = std::string ( optarg );
        break;
      case 'k':
        drawKeypoints = true;
        break;
      case 'd':
        depthTuningSet = true;
        ss << optarg;
        ss >> depthTuning;
        if ( !ss )
        {
          std::cout << "Failed to parse depth_tuning parameter" << std::endl;
          help ();
        }
        break;
      case 'c':
        inputCloudSet = true;
        inputCloud = std::string ( optarg );
        break;
      case 's':
        stdThreshSet = true;
        ss << optarg;
        ss >> stdThresh;
        if ( !ss )
        {
          std::cout << "Failed to parse stdev mult threshold parameter" << std::endl;
          help ();
        }
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
      case 'm':
        mlsRadiusSet = true;
        ss << optarg;
        ss >> mlsRadius;
        if ( !ss )
        {
          std::cout << "Failed to parse MLS radius parameter" << std::endl;
          help ();
        }
        break;
      case 'x':
        downscaleSet = true;
        ss << optarg;
        ss >> downscale;
        if ( !ss )
        {
          std::cout << "Failed to parse downscale parameter" << std::endl;
          help ();
        }
        break;
      default:
        std::cout << "Unknown option " << c << std::endl;
        help ();
        break;
      }
    }

    if ( !bundleFileSet )
    {
      std::cout << "bundle option is required" << std::endl;
      help ();
    }
    if ( !listFileSet )
    {
      std::cout << "list option is required" << std::endl;
      help ();
    }
    if ( !depthTuningSet )
    {
      depthTuning = 1.0;
    }
    if ( !downscaleSet )
    {
      downscale = 1;
    }

    if ( !outputSet )
    {
      std::cout << "output option is required" << std::endl;
      help ();
    }

    if ( !stdThreshSet )
    {
      std::cout << "std thresh option is required" << std::endl;
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

    if ( !voxelSizeSet )
    {
      std::cout << "voxel size option is required" << std::endl;
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
              const Eigen::Matrix3f &                                         basis,
              const BundleCamera &                                            camera,
              float                                                           scale,
              std::vector<std::vector<std::vector<std::pair<uint64_t,
                                                            uint64_t> > > > & voxels,
              float                                                           length,
              const Eigen::Vector3f &                                         translation,
              const Eigen::Vector3i &                                         dimensions )
{
  assert ( ( int )voxels.size ( ) == dimensions[0] );
  assert ( ( int )voxels[0].size ( ) == dimensions[1] );
  assert ( ( int )voxels[0][0].size ( ) == dimensions[2] );
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin (); i != points.end (); ++i )
  {
    Eigen::Vector3d cameraPosition;
#pragma omp critical
    cameraPosition = -basis.cast<double>() * camera.GetR ().transpose ().cast<double>() *
                     camera.GetT ().cast<double>() / scale;
    Eigen::Vector3d destination ( i->x, i->y, i->z );
    Eigen::Vector3d direction = destination - cameraPosition;
    Eigen::Vector3d tDelta = direction.normalized ();
    Eigen::Vector3i source = material2Voxel ( cameraPosition, ( double )length, translation.cast<double>() );
    Eigen::Vector3i sink = material2Voxel ( destination, ( double )length, translation.cast<double>() );
    Eigen::Vector3d positionInGrid = 1.0 / length * ( cameraPosition - translation.cast<double>() );
    Eigen::Vector3d destPositionInGrid = 1.0 / length * ( destination - translation.cast<double>() );
    Eigen::Vector3d tMax;
    Eigen::Vector3i increment;
    Eigen::Vector3d diff = destPositionInGrid - positionInGrid;
    assert (
      source[0] >= 0 && source [1] >= 0 && source [2] >= 0 && source [0] < dimensions [0] && source [1] <
      dimensions [1] && source [2] < dimensions [2] );
    if ( sink[0] < 0 && sink [1] < 0 && sink [2] < 0 && sink [0] >= dimensions [0] && sink [1] >=
         dimensions [1] && sink [2] >= dimensions [2] )
    {
      continue;
    }
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
      if ( invisible )
      {
#pragma omp atomic
        ++voxel.first;
        double dist = ( sink - source ).cast<double>().norm ();
        if ( dist < closest )
        {
          closest = dist;
        }
        if ( dist < 1.42 )
        {
          invisible = false;
        }
      }
#pragma omp atomic
      ++voxel.second;
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

  TIME_BEGIN ( "Loading bundle file" )
  BundleFile file ( options.bundleFile );
  TIME_END

  const std::vector<BundleCamera> cameras = file.GetCameras ();

  std::ifstream images ( options.listFile.c_str () );
  if ( !images )
  {
    std::cout << "Failed to open list file" << std::endl;
    options.help ();
  }

  TIME_BEGIN ( "Loading files from list" )
  std::vector<std::string> imageList;
  while ( images )
  {
    std::string filename, line;
    if ( !( images >> filename ) )
    {
      break;
    }
    imageList.push_back ( filename );
    getline ( images, line );
  }
  images.close ();
  TIME_END

  size_t list_index = options.listFile.find_last_of ( "/" );

  TIME_BEGIN ( "Loading all poiht clouds" )
#pragma omp parallel for
  for ( int i = 0; i < ( int )imageList.size (); ++i )
  {
    const std::string & filename = imageList [i];
    std::string colorFilename = options.listFile;
    if ( list_index == std::string::npos )
    {
      list_index = 0;
    }
    colorFilename.replace ( list_index + 1, colorFilename.size () - list_index, filename );
    assert ( i < ( int )cameras.size () );
    if ( !cameras[i].IsValid () )
    {
      continue;
    }
    size_t replacement = colorFilename.find ( "color.jpg" );
    if ( replacement == std::string::npos )
    {
      continue;
    }
#pragma omp critical
    {
      std::cout << "Opening " << colorFilename << std::endl;
    }
    std::string depthFilename ( colorFilename );
    assert ( replacement + 9 <= depthFilename.size () );
    depthFilename.replace ( replacement, 9, "depth.raw" );
    std::string keyFilename ( colorFilename );
    assert ( replacement + 9 <= keyFilename.size () );
    keyFilename.replace ( replacement, 9, "color.ks" );
    if ( !options.drawKeypoints )
    {
      keyFilename = "";
    }
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points = LoadCloud (
      colorFilename,
      depthFilename,
      keyFilename,
      cameras[i],
      options.depthTuning,
      options.downscale );
#pragma omp critical
    {
      *combinedCloud += *points;
    }
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

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reduced ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );

  pcl::VoxelGrid<pcl::PointXYZRGBNormal> downsampler;
  downsampler.setInputCloud ( combinedCloud );
  downsampler.setLeafSize ( options.voxelSize, options.voxelSize, options.voxelSize );
  std::cout << "Data size is " << combinedCloud->size () << std::endl;
  TIME_BEGIN ( "Downsampling data with voxel grid" )
  downsampler.filter ( *reduced );
  TIME_END

  TIME_BEGIN ( "Saving downsampled ply" )
  savePlyFile ( options.output + ".downsampled.ply", *reduced );
  TIME_END

  pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree =
    boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> > ();
  tree->setInputCloud ( reduced );
  pcl::PointCloud <pcl::PointNormal>::Ptr normals ( new pcl::PointCloud<pcl::PointNormal>() );
  pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointNormal> normalEstimation;
  normalEstimation.setInputCloud ( reduced );
  normalEstimation.setOutputNormals ( normals );
  normalEstimation.setSearchRadius ( options.mlsRadius );
  normalEstimation.setSearchMethod ( tree );
  pcl::PointCloud<pcl::PointXYZRGBNormal> cleaned;
  std::cout << "Data size is " << reduced->size () << std::endl;
  TIME_BEGIN ( "Reconstructing normals with MLS" )
  normalEstimation.reconstruct ( cleaned );
  TIME_END

  TIME_BEGIN ( "Putting together point cloud" )
  pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator k;
  pcl::PointCloud<pcl::PointNormal>::const_iterator j;
  pcl::PointCloud<pcl::PointXYZRGBNormal> final;
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
    pcl::PointXYZRGBNormal newPoint;
    newPoint.normal_x = flip * j->normal_x;
    newPoint.normal_y = flip * j->normal_y;
    newPoint.normal_z = flip * j->normal_z;
    newPoint.rgb = k->rgb;
    newPoint.x = i->x;
    newPoint.y = i->y;
    newPoint.z = i->z;
    final.push_back ( newPoint );
  }
  TIME_END

  TIME_BEGIN ( "Reorienting point cloud" )
  Eigen::Matrix3f basis = reorient ( final );
  TIME_END

  std::cout << "Basis is " << std::endl;
  std::cout << basis << std::endl;
  std::cout << std::endl;

  TIME_BEGIN ( "Saving ply" )
  savePlyFile ( options.output + ".ply", final );
  TIME_END

  TIME_BEGIN ( "Saving pcd" )
  pcl::io::savePCDFile ( options.output + ".pcd", final, true );
  TIME_END

  std::cout << "Beginning visibility voxelization." << std::endl;

  Eigen::Vector3f min, max;

  TIME_BEGIN ( "Finding AABB" )
  findAABB ( final, basis, cameras, options.depthTuning, min, max );
  TIME_END

  std::cout << "AABB is " << std::endl;
  std::cout << min << std::endl;
  std::cout << "-" << std::endl;
  std::cout << max << std::endl;
  std::cout << std::endl;

  Eigen::Vector3f width = max - min;
  Eigen::Vector3i dimensions = ( 1.0 / options.voxelSize * width + Eigen::Vector3f ( 1, 1, 1 ) ).cast<int>();

  KVO voxels ( dimensions [0], dimensions [1], dimensions [2], options.voxelSize, min );
  std::vector<std::vector<std::vector<std::pair<uint64_t, uint64_t> > > > temp
    ( voxels.size ( 0 ), std::vector<std::vector<std::pair<uint64_t, uint64_t> > > ( voxels.size (
                                                                                       1 ),
                                                                                     std::vector<std::pair<
                                                                                                   uint64_t, uint64_t> > ( voxels.size ( 2 ) ) ) );

  TIME_BEGIN ( "Visibility ray tracing" )
#pragma omp parallel for
  for ( int i = 0; i < ( int )imageList.size (); ++i )
  {
    const std::string & filename = imageList [i];
    std::string colorFilename = options.listFile;
    colorFilename.replace ( list_index + 1, colorFilename.size () - list_index, filename );
    assert ( i < ( int )cameras.size () );
    if ( !cameras[i].IsValid () )
    {
      continue;
    }
    size_t replacement = colorFilename.find ( "color.jpg" );
    if ( replacement == std::string::npos )
    {
      continue;
    }
#pragma omp critical
    std::cout << "Opening " << colorFilename << std::endl;
    std::string depthFilename ( colorFilename );
    assert ( replacement + 9 <= depthFilename.size () );
    depthFilename.replace ( replacement, 9, "depth.raw" );
    std::string keyFilename ( colorFilename );
    assert ( replacement + 9 <= keyFilename.size () );
    keyFilename.replace ( replacement, 9, "color.ks" );
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points = LoadCloud (
      colorFilename,
      depthFilename,
      "",
      cameras[i],
      options.depthTuning,
      options.downscale, basis );
    carveVoxels ( *points, basis, cameras [i], options.depthTuning, temp, options.voxelSize, min, dimensions );
  }
  TIME_END

  TIME_BEGIN ( "Building final voxel grid" )
#pragma omp parallel for
  for ( int i = 0; i < ( int )temp.size (); ++i )
  {
    for ( int j = 0; j < ( int )temp[i].size (); ++j )
    {
      for ( int k = 0; k < ( int )temp[i][j].size (); ++k )
      {
        voxels [i][j][k] = ( float )temp[i][j][k].first / temp[i][j][k].second;
      }
    }
  }
  TIME_END

  TIME_BEGIN ( "Saving voxels" )
  voxels.save ( options.output + ".kvo" );
  TIME_END
}
