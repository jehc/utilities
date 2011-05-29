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
  const std::string &  colorFilename,
  const std::string &  depthFilename,
  const std::string &  keypointFilename,
  const BundleCamera & camera,
  float                scale,
  int                  downscale )
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
      p = R.transpose () * p;

      Eigen::Vector3f p_cam ( 0, 0, 0 );
      p_cam = cameraMatrix.inverse () * p_cam;
      p_cam[2] *= -1;

      p_cam = p_cam - t / scale;
      p_cam = R.transpose () * p_cam;

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
  for ( int i = 0; i < histogram.size (); ++i )
  {
    for ( int j = 0; j < histogram[i].size (); ++j )
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
  for ( int i = 0; i < histogram.size (); ++i )
  {
    for ( int j = 0; j < histogram[i].size (); ++j )
    {
      double theta_0 = M_PI * i / 60;
      double phi_0 = M_PI * j / 60;
      double delta_phi = M_PI / 60;
      double delta_theta = M_PI / 60;
      double normalization = delta_phi * ( cos ( theta_0 ) - cos ( theta_0 + delta_theta ) );
      histogram[i][j] /= normalization;
    }
  }
  return histogram;
}

void
compute_axis ( const std::vector<std::vector<float> > & histogram,
               Eigen::Vector3f &                        v1,
               Eigen::Vector3f &                        v2,
               Eigen::Vector3f &                        v3 )
{
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
  v1[0] = sin ( M_PI * theta_max / 60 ) * cos ( M_PI * phi_max / 60 );
  v1[1] = sin ( M_PI * theta_max / 60 ) * sin ( M_PI * phi_max / 60 );
  v1[2] = cos ( M_PI * theta_max / 60 );
  double norm = sqrt ( v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] );
  for ( int i = 0; i < 3; ++i )
  {
    v1[i] /= norm;
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

      double dot = xtemp * v1[0] + ytemp * v1[1] + ztemp * v1[2];
      if ( dot > -slack && dot < slack && histogram[i][j] >= max )
      {
        max = histogram[i][j];
        v2[0] = xtemp;
        v2[1] = ytemp;
        v2[2] = ztemp;
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

      double dot1 = xtemp * v1[0] + ytemp * v1[1] + ztemp * v1[2];
      double dot2 = xtemp * v2[0] + ytemp * v2[1] + ztemp * v2[2];
      if ( dot1 > -slack && dot1 < slack && dot2 > -slack && dot2 < slack && histogram[i][j] >= max )
      {
        max = histogram[i][j];
        v3[0] = xtemp;
        v3[1] = ytemp;
        v3[2] = ztemp;
      }
    }
  }
}

void
project_onto_basis ( pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud,
                     const Eigen::Vector3f &                   v1,
                     const Eigen::Vector3f &                   v2,
                     const Eigen::Vector3f &                   v3 )
{
  double projections [6] = { v1[1], -v1[1], v2[1], -v2[1], v3[1], -v3[1] };
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
  double y [3];
  switch ( maxIndex )
  {
  case 0:
    y[0] = 1;
    y[1] = 0;
    y[2] = 0;
    break;
  case 1:
    y[0] = -1;
    y[1] = 0;
    y[2] = 0;
    break;
  case 2:
    y[0] = 0;
    y[1] = 1;
    y[2] = 0;
    break;
  case 3:
    y[0] = 0;
    y[1] = -1;
    y[2] = 0;
    break;
  case 4:
    y[0] = 0;
    y[1] = 0;
    y[2] = 1;
    break;
  case 5:
    y[0] = 0;
    y[1] = 0;
    y[2] = -1;
    break;
  }
  double up[3] = { 0, 1, 0 };
  double axis[3] = { y[2] * up[1] - y[1] * up[2], y[0] * up[2] - y[2] * up[0], y[1] * up[0] - y[0] * up[1] };
  double angle = acos ( y[0] * up[0] + y[1] * up[1] + y[2] * up[2] );
  double R [9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

#if 1
  R[0] = cos ( angle );
  R[4] = cos ( angle );
  R[8] = cos ( angle );

  double E[9] = { axis[0] * axis[0], axis[0] * axis[1], axis[0] * axis[2],
                  axis[1] * axis[0], axis[1] * axis[1], axis[1] * axis[2],
                  axis[2] * axis[0], axis[2] * axis[1], axis[2] * axis[2] };
  for ( int i = 0; i < 9; ++i )
  {
    R[i] += ( 1 - cos ( angle ) ) * E[i];
  }
  R[1] -= sin ( angle ) * axis[2];
  R[2] += sin ( angle ) * axis[1];
  R[3] += sin ( angle ) * axis[2];
  R[5] -= sin ( angle ) * axis[0];
  R[6] -= sin ( angle ) * axis[1];
  R[7] += sin ( angle ) * axis[0];
#endif

  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator i = pointCloud.begin ();
        i != pointCloud.end ();
        ++i )
  {
    double x, y, z, tempx, tempy, tempz;
    tempx = v1[0] * i->normal_x + v1[1] * i->normal_y + v1[2] * i->normal_z;
    tempy = v2[0] * i->normal_x + v2[1] * i->normal_y + v2[2] * i->normal_z;
    tempz = v3[0] * i->normal_x + v3[1] * i->normal_y + v3[2] * i->normal_z;
    x = R[0] * tempx + R[3] * tempy + R[6] * tempz;
    y = R[1] * tempx + R[4] * tempy + R[7] * tempz;
    z = R[2] * tempx + R[5] * tempy + R[8] * tempz;
    i->normal_x = x;
    i->normal_y = y;
    i->normal_z = z;
    tempx = v1[0] * i->x + v1[1] * i->y + v1[2] * i->z;
    tempy = v2[0] * i->x + v2[1] * i->y + v2[2] * i->z;
    tempz = v3[0] * i->x + v3[1] * i->y + v3[2] * i->z;
    x = R[0] * tempx + R[3] * tempy + R[6] * tempz;
    y = R[1] * tempx + R[4] * tempy + R[7] * tempz;
    z = R[2] * tempx + R[5] * tempy + R[8] * tempz;
    i->x = x;
    i->y = y;
    i->z = z;
  }
}

void
print_basis ( const Eigen::Vector3f & v1, const Eigen::Vector3f & v2, const Eigen::Vector3f & v3 )
{
  fprintf ( stderr, "%f %f %f\n", v1[0], v1[1], v1[2] );
  fprintf ( stderr, "%f %f %f\n", v2[0], v2[1], v2[2] );
  fprintf ( stderr, "%f %f %f\n", v3[0], v3[1], v3[2] );
  double angle1 = 180 * acos ( v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] ) / M_PI;
  double angle2 = 180 * acos ( v2[0] * v3[0] + v2[1] * v3[1] + v2[2] * v3[2] ) / M_PI;
  double angle3 = 180 * acos ( v3[0] * v1[0] + v3[1] * v1[1] + v3[2] * v1[2] ) / M_PI;
  fprintf ( stderr, "%lf %lf %lf\n", angle1, angle2, angle3 );
}

void
saveHistogram ( const std::string &                      filename,
                const std::vector<std::vector<float> > & histogram,
                const Eigen::Vector3f &                  v1,
                const Eigen::Vector3f &                  v2,
                const Eigen::Vector3f &                  v3 )
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
  project_onto_basis ( cloud, v1, v2, v3 );
  savePlyFile ( filename, cloud );
}

void
reorient ( pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud )
{
  std::vector<std::vector<float> > histogram = generate_histogram ( pointCloud );
  Eigen::Vector3f v1, v2, v3;
  compute_axis ( histogram, v1, v2, v3 );
  project_onto_basis ( pointCloud, v1, v2, v3 );
#if 0
  print_basis ( v1, v2, v3 );
#endif
  saveHistogram ( "histogram.ply", histogram, v1, v2, v3 );
}

void
findAABB ( const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
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
    { { "bundle",        required_argument,          0,                 'b' },
      { "list",          required_argument,          0,                 'l' },
      { "output",        required_argument,          0,                 'o' },
      { "keypoints",     no_argument,                0,                 'k' },
      { "depth_tuning",  required_argument,          0,                 'd' },
      { "input_cloud",   required_argument,          0,                 'c' },
      { "postprocess",   no_argument,                0,                 'p' },
      { "reprocess",     no_argument,                0,                 'r' },
      { "std_thresh",    required_argument,          0,                 's' },
      { "voxel_size",    required_argument,          0,                 'v' },
      { "mls_radius",    required_argument,          0,                 'm' },
      { "downscale",     required_argument,          0,                 'x' },
      { "validate_data", no_argument,                0,                 'z' },
      { "voxelize",      no_argument,                0,                 'e' },
      { 0,               0,                          0,                 0   } };

    bool bundleFileSet = false;
    bool listFileSet = false;
    bool outputSet = false;
    bool inputCloudSet = false;
    bool depthTuningSet = false;
    bool stdThreshSet = false;
    bool voxelSizeSet = false;
    bool mlsRadiusSet = false;
    bool downscaleSet = false;

    reprocess = false;
    postProcess = false;
    drawKeypoints = false;
    validateData = false;
    voxelize = false;

    while ( ( c = getopt_long ( argc, argv, "b:l:o:kd:c:prs:v:m:", longopts, &indexptr ) ) != -1 )
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
      case 'z':
        validateData = true;
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
      case 'p':
        postProcess = true;
        break;
      case 'r':
        reprocess = true;
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
      case 'e':
        voxelize = true;
        break;
      default:
        std::cout << "Unknown option " << c << std::endl;
        help ();
        break;
      }
    }

    if ( !reprocess )
    {
      if ( !bundleFileSet )
      {
        std::cout << "bundle option is required if not reprocessing" << std::endl;
        help ();
      }
      if ( !listFileSet )
      {
        std::cout << "list option is required if not reprocessing" << std::endl;
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
    }
    else
    {
      if ( !inputCloudSet )
      {
        std::cout << "cloud option is required if reprocessing" << std::endl;
        help ();
      }
    }

    if ( !outputSet )
    {
      std::cout << "output option is required" << std::endl;
      help ();
    }

    if ( postProcess )
    {
      if ( !stdThreshSet )
      {
        std::cout << "std thresh option is required if postprocessing" << std::endl;
        help ();
      }
      if ( !voxelSizeSet )
      {
        std::cout << "voxel size option is required if postprocessing" << std::endl;
        help ();
      }
      if ( !mlsRadiusSet )
      {
        std::cout << "MLS radius option is required if postprocessing" << std::endl;
        help ();
      }
    }

    if ( voxelize )
    {
      if ( !voxelSizeSet )
      {
        std::cout << "voxel size option is required if voxelizing" << std::endl;
        help ();
      }
      if ( !bundleFileSet )
      {
        std::cout << "bundle option is required if not reprocessing" << std::endl;
        help ();
      }
      if ( !listFileSet )
      {
        std::cout << "list option is required if not reprocessing" << std::endl;
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
    }
  }
};

Eigen::Vector3i
material2Voxel ( const Eigen::Vector3f & material, float length, const Eigen::Vector3f & translation )
{
  Eigen::Vector3f result = 1.0 / length * ( material - translation );

  return Eigen::Vector3i ( ( int )result [0], ( int )result[1], ( int )result [2] );
}

Eigen::Vector3f
voxel2Material ( const Eigen::Vector3i & voxel, float length, const Eigen::Vector3f & translation )
{
  return length * ( ( voxel.cast<float>() + Eigen::Vector3f ( 0.5, 0.5, 0.5 ) ) ) + translation;
}

void
carveVoxels ( const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
              const BundleCamera &                            camera,
              KVO &                                           voxels,
              float                                           length,
              const Eigen::Vector3f &                         translation,
              const Eigen::Vector3i &                         dimensions )
{
  assert ( voxels.size ( 0 ) == dimensions[0] );
  assert ( voxels.size ( 1 ) == dimensions[1] );
  assert ( voxels.size ( 2 ) == dimensions[2] );
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin (); i != points.end (); ++i )
  {
    Eigen::Vector3f destination ( i->x, i->y, i->z );
    Eigen::Vector3f direction = destination - camera.GetT ();
    Eigen::Vector3f tDelta = direction.normalized ();
    Eigen::Vector3i source = material2Voxel ( camera.GetT (), length, translation );
    Eigen::Vector3i sink = material2Voxel ( destination, length, translation );
    Eigen::Vector3f positionInGrid = 1.0 / length * ( camera.GetT () - translation );
    Eigen::Vector3f destPositionInGrid = 1.0 / length * ( destination - translation );
    Eigen::Vector3f tMax;
    Eigen::Vector3i increment;
    Eigen::Vector3i diff = source - sink;
    int blocks = abs ( diff[0] ) + abs ( diff[1] ) + abs ( diff[2] );
    for ( int i = 0; i < 3; ++i )
    {
      tDelta [i] = fabs ( 1.0 / tDelta[i] );
      increment [i] = ( int )( direction [i] / fabs ( direction[i] ) );
      int nextBarrier = increment [i] > 0 ? ( int )ceil ( positionInGrid [i] ) : ( int )floor (
        positionInGrid [i] );
      tMax [i] = fabs ( nextBarrier - positionInGrid [i] );
    }

    bool invisible = true;
    float closest = std::numeric_limits<float>::infinity ();
    while ( true )
    {
      int minIndex = 0;
      float min = std::numeric_limits<float>::infinity ();
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

      std::pair<uint64_t, uint64_t> & voxel = voxels [source [0]][source [1]][source [2]];
      if ( invisible )
      {
#pragma omp atomic
        ++voxel.first;
        if ( !blocks )
        {
          invisible = false;
        }
        --blocks;
      }
#pragma omp atomic
      ++voxel.second;
    }
    if ( invisible )
    {
      std::cout << "Missed, but the closest was " << closest << std::endl;
    }
  }
}

int
main ( int argc, char * * argv )
{
  struct timeval tic, toc;

  Options options ( argc, argv );

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr combinedCloud ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );

  std::cout << "Loading data" << std::endl;
  gettimeofday ( &tic, 0 );
  if ( !options.reprocess )
  {
    BundleFile file ( options.bundleFile );
    const std::vector<BundleCamera> cameras = file.GetCameras ();
    std::ifstream images ( options.listFile.c_str () );
    if ( !images )
    {
      std::cout << "Failed to open list file" << std::endl;
      options.help ();
    }

    std::vector<std::string> imageList;
    while ( images ) // && count < maxCount)
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
#pragma omp parallel for
    for ( int i = 0; i < imageList.size (); ++i )
    {
      const std::string & filename = imageList [i];
      std::string tempFilename = options.listFile;
      size_t list_index = options.listFile.find_last_of ( "/" );
      if ( list_index == std::string::npos )
      {
        list_index = 0;
      }
      std::string filenameNew = tempFilename.replace ( list_index + 1,
        tempFilename.size () - list_index, filename );
      assert ( i < cameras.size () );
      if ( !cameras[i].IsValid () )
      {
        continue;
      }
      size_t replacement = filenameNew.find ( "color.jpg" );
      if ( replacement == std::string::npos )
      {
        continue;
      }
      std::cout << "Opening " << filenameNew << std::endl;
      std::string filename2 ( filenameNew );
      assert ( replacement + 9 <= filename2.size () );
      filename2.replace ( replacement, 9, "depth.raw" );
      std::string keyFilename ( filenameNew );
      assert ( replacement + 9 <= keyFilename.size () );
      keyFilename.replace ( replacement, 9, "color.ks" );
      if ( !options.drawKeypoints )
      {
        keyFilename = "";
      }
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points = LoadCloud (
        filenameNew,
        filename2,
        keyFilename,
        cameras[i],
        options.depthTuning,
        options.downscale );
#pragma omp critical
      {
        *combinedCloud += *points;
      }
    }
  }
  else
  {
    if ( -1 == pcl::io::loadPCDFile ( options.inputCloud, *combinedCloud ) )
    {
      std::cout << "Failed to load cloud for reprocessing" << std::endl;
      options.help ();
    }
  }
  gettimeofday ( &toc, 0 );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;

  if ( options.validateData )
  {
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
  }

  if ( options.voxelize )
  {
    std::cout << "Beginning voxelization." << std::endl;
    BundleFile file ( options.bundleFile );
    const std::vector<BundleCamera> cameras = file.GetCameras ();
    std::ifstream images ( options.listFile.c_str () );
    if ( !images )
    {
      std::cout << "Failed to open list file" << std::endl;
      options.help ();
    }

    Eigen::Vector3f min, max;
    findAABB ( *combinedCloud, min, max );
    Eigen::Vector3f width = max - min;
    Eigen::Vector3i dimensions = ( 1.0 / options.voxelSize * width + Eigen::Vector3f ( 1, 1, 1 ) ).cast<int>();

    KVO voxels ( dimensions [0], dimensions [1], dimensions [2], options.voxelSize, min );

    std::vector<std::string> imageList;
    while ( images ) // && count < maxCount)
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
#pragma omp parallel for
    for ( int i = 0; i < 5 /*(imageList.size ()*/; ++i )
    {
      const std::string & filename = imageList [i];
      std::string tempFilename = options.listFile;
      size_t list_index = options.listFile.find_last_of ( "/" );
      if ( list_index == std::string::npos )
      {
        list_index = 0;
      }
      std::string filenameNew = tempFilename.replace ( list_index + 1,
        tempFilename.size () - list_index, filename );
      assert ( i < cameras.size () );
      if ( !cameras[i].IsValid () )
      {
        continue;
      }
      size_t replacement = filenameNew.find ( "color.jpg" );
      if ( replacement == std::string::npos )
      {
        continue;
      }
      std::cout << "Opening " << filenameNew << std::endl;
      std::string filename2 ( filenameNew );
      assert ( replacement + 9 <= filename2.size () );
      filename2.replace ( replacement, 9, "depth.raw" );
      std::string keyFilename ( filenameNew );
      assert ( replacement + 9 <= keyFilename.size () );
      keyFilename.replace ( replacement, 9, "color.ks" );
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points = LoadCloud (
        filenameNew,
        filename2,
        "",
        cameras[i],
        options.depthTuning,
        options.downscale );
      carveVoxels ( *points, cameras [i], voxels, options.voxelSize, min, dimensions );
    }
    voxels.save ( options.output + ".kvo" );
  }

  if ( options.postProcess )
  {
#if 0
    pcl::PointCloud<pcl::PointXYZRGBNormal> final;
    const int desired = 10000000;
    const float probability = ( float )desired / ( float )combinedCloud->size ();
    for ( pcl::PointCloud<pcl::PointXYZRGB>::iterator i = combinedCloud->begin ();
          i != combinedCloud->end ();
          ++i )
    {
      if ( ( float )rand () / ( ( float )RAND_MAX + 1 ) < probability )
      {
        pcl::PointXYZRGBNormal newPoint;
        newPoint.x = i->x;
        newPoint.y = i->y;
        newPoint.z = i->z;
        newPoint.rgb = i->rgb;
        final.push_back ( newPoint );
      }
    }
#else
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr reduced ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );

/*    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> outlierRemoval;
 *  outlierRemoval.setInputCloud (combinedCloud);
 *  outlierRemoval.setNegative (false);
 *  outlierRemoval.setStddevMulThresh(options.stdThresh);
 *  std::cout << "Statistical outlier removal on size " << combinedCloud->size() << std::flush;
 *  gettimeofday (&tic, 0);
 *  outlierRemoval.filter (*reduced);
 *  gettimeofday (&toc, 0);
 *  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;
 */
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> downsampler;
    downsampler.setInputCloud ( combinedCloud );
    downsampler.setLeafSize ( options.voxelSize, options.voxelSize, options.voxelSize );
    std::cout << "Voxel grid downsampling on size " <<
    combinedCloud->size () << std::flush;
    gettimeofday ( &tic, 0 );
    downsampler.filter ( *reduced );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;

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
    std::cout << "Moving least squares on size " << reduced->size () << std::flush;
    gettimeofday ( &tic, 0 );
    normalEstimation.reconstruct ( cleaned );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;

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
    std::cout << "Reorienting on size " << final.size () << std::flush;
    gettimeofday ( &tic, 0 );
    reorient ( final );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;
#endif
    std::cout << "Saving ply of size " << final.size () << std::flush;
    gettimeofday ( &tic, 0 );
    savePlyFile ( options.output + ".ply", final );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;

    std::cout << "Saving pcd of size " << final.size () << std::flush;
    gettimeofday ( &tic, 0 );
    pcl::io::savePCDFile ( options.output + ".pcd", final, true );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;
  }
  else
  {
    std::cout << "Saving ply of size " << combinedCloud->size () << std::flush;
    gettimeofday ( &tic, 0 );
    savePlyFile ( options.output + ".ply", *combinedCloud );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;

    std::cout << "Saving pcd of size " << combinedCloud->size () << std::flush;
    gettimeofday ( &tic, 0 );
    pcl::io::savePCDFile ( options.output + ".pcd", *combinedCloud, true );
    gettimeofday ( &toc, 0 );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;
  }
}
