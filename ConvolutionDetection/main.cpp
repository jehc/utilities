#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "ply_io.h"
#include <fstream>
#include <cmath>

#include <sys/time.h>

#include "omp.h"

void
findAABB ( const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
           cv::Vec3f &                                     min,
           cv::Vec3f &                                     max )
{
  for ( int i = 0; i < 3; ++i )
  {
    min[i] = std::numeric_limits<float>::infinity ();
    max[i] = -std::numeric_limits<float>::infinity ();
  }
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i =
          points.begin ();
        i != points.end ();
        ++i )
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

int
binNormals (
  const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
  float                                           lengthOfEdge,
  const cv::Vec3f &                               min,
  const cv::Vec3f &                               max,
  std::vector<std::vector<std::vector<pcl::PointNormal> > > * &
  normalBins,
  std::vector<std::vector<int> > &                normalizationMap )
{
  cv::Vec3f voxels = max - min;

  for ( int i = 0; i < 3; ++i )
  {
    voxels[i] /= lengthOfEdge;
    voxels[i] += 1;
  }
  normalBins = new std::vector<std::vector<std::vector<pcl::PointNormal> > >
                 ( ( int )ceil ( voxels[0] ),
                 std::vector<std::vector<pcl::PointNormal> > ( ( int )ceil (
                                                                 voxels[1] ),
                                                               std
                                                               ::vector<pcl::
                                                                        PointNormal> ( (
                                                                                         int )ceil ( voxels[2] ) ) ) );
  normalizationMap = std::vector<std::vector<int> > (
    normalBins->size (), std::vector<int> ( ( *normalBins )[0][0].size () ) );
  std::vector<int> floorCount ( ( *normalBins )[0].size () );
#pragma omp parallel for
  for ( int i = 0; i < normalBins->size (); ++i )
  {
    for ( int j = 0; j < ( *normalBins )[i].size (); ++j )
    {
      for ( int k = 0; k < ( *normalBins )[i][j].size (); ++k )
      {
        ( *normalBins )[i][j][k].normal_x = 0;
        ( *normalBins )[i][j][k].normal_y = 0;
        ( *normalBins )[i][j][k].normal_z = 0;
      }
    }
  }
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i =
          points.begin ();
        i != points.end ();
        ++i )
  {
    int x = ( int )( ( i->x - min[0] ) / lengthOfEdge );
    int y = ( int )( ( i->y - min[1] ) / lengthOfEdge );
    int z = ( int )( ( i->z - min[2] ) / lengthOfEdge );
    if ( i->normal_y > 0.9 )
    {
      ++floorCount [y];
    }
    if ( i->normal_x * i->normal_x + i->normal_y * i->normal_y + i->normal_z *
         i->normal_z < 1e-9 )
    {
      std::cout << "bad normal at " << i->x << " " << i->y << " " << i->z <<
      std::endl;
      continue;
    }
//    if (count[x][y][z]) continue;
    assert ( i->x >= min[0] );
    assert ( i->y >= min[1] );
    assert ( i->z >= min[2] );
    assert ( x < normalBins->size () && x >= 0 );
    assert ( y < ( *normalBins )[x].size () && y >= 0 );
    assert ( z < ( *normalBins )[x][y].size () && z >= 0 );
    ( *normalBins )[x][y][z].normal_x += i->normal_x;
    ( *normalBins )[x][y][z].normal_y += i->normal_y;
    ( *normalBins )[x][y][z].normal_z += i->normal_z;
    ++normalizationMap [x][z];
  }
  int maxCount = 0;
  int maxIndex = 0;
  for ( int i = 0; i < floorCount.size (); ++i )
  {
    if ( floorCount [i] > maxCount )
    {
      maxIndex = i;
      maxCount = floorCount [i];
    }
  }
  int yOffset = maxIndex;
#pragma omp parallel for
  for ( int i = 0; i < normalBins->size (); ++i )
  {
    for ( size_t j = 0; j < ( *normalBins )[i].size (); ++j )
    {
      for ( size_t k = 0; k < ( *normalBins )[i][j].size (); ++k )
      {
        double norm = sqrt ( pow ( ( *normalBins )[i][j][k].normal_x,
                                   2.0f ) +
                             pow ( ( *normalBins )[i][j][k].normal_y,
                                   2.0f ) +
                             pow ( ( *normalBins )[i][j][k].normal_z, 2.0f ) );
        ( *normalBins )[i][j][k].normal_x /= norm;
        ( *normalBins )[i][j][k].normal_y /= norm;
        ( *normalBins )[i][j][k].normal_z /= norm;
      }
    }
  }
  return yOffset;
}

float
findFloor ( const pcl::PointCloud<pcl::PointXYZRGBNormal> & points,
            float                                           limit )
{
  float floor = 0;
  int n = 0;

  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i =
          points.begin ();
        i != points.end ();
        ++i )
  {
    if ( i->normal_y > 0.99 && i->y < limit )
    {
      floor += i->y;
      ++n;
    }
  }
  std::cout << "Found " << n << " potential floor points." << std::endl;
  return floor / n;
}

int
opposite ( int region )
{
  switch ( region )
  {
  case 0:
    return 1;
  case 1:
    return 0;
  case 2:
    return 3;
  case 3:
    return 2;
  case 4:
    return 5;
  case 5:
    return 4;
  case 6:
    return 13;
  case 7:
    return 12;
  case 8:
    return 11;
  case 9:
    return 10;
  case 10:
    return 9;
  case 11:
    return 8;
  case 12:
    return 7;
  case 13:
    return 6;
  default:
    assert ( 0 );
  }
}

void
getRegions ( const pcl::PointNormal & normal, std::vector<int> & regions )
{
  const double threshold = 0.9;

  assert ( threshold > 0.71 && threshold < 1.0 );
  regions.clear ();

#if 0
  std::cerr << normal.normal_x << " " << normal.normal_y << " " <<
  normal.normal_z << std::endl;
#endif

  if ( normal.normal_x != normal.normal_x || normal.normal_y !=
       normal.normal_y || normal.normal_z != normal.normal_z )
  {
    return;
  }
  if ( normal.normal_x > threshold )
  {
    regions.push_back ( 0 );
  }
  else if ( normal.normal_x < -threshold )
  {
    regions.push_back ( 1 );
  }
  else if ( normal.normal_y > threshold )
  {
    regions.push_back ( 2 );
  }
  else if ( normal.normal_y < -threshold )
  {
    regions.push_back ( 3 );
  }
  else if ( normal.normal_z > threshold )
  {
    regions.push_back ( 4 );
  }
  else if ( normal.normal_z < -threshold )
  {
    regions.push_back ( 5 );
  }
  else
  {
    int dominantRegion =
      ( ( ( ( normal.normal_z >
              0 ) ? 1 : 0 ) <<
          2 ) |
        ( ( ( normal.normal_y >
              0 ) ? 1 : 0 ) << 1 ) | ( ( normal.normal_x > 0 ) ? 1 : 0 ) ) + 6;
#if 0
    assert (
      ( ( ( ( normal.normal_z >
              0 ) ? 1 : 0 ) <<
          2 ) |
        ( ( ( normal.normal_y >
              0 ) ? 1 : 0 ) <<
          1 ) | ( ( normal.normal_x > 0 ) ? 1 : 0 ) ) + 6 == dominantRegion );
    std::cerr <<
    ( ( ( ( normal.normal_z >
            0 ) ? 1 : 0 ) <<
        2 ) |
      ( ( ( normal.normal_y >
            0 ) ? 1 : 0 ) <<
        1 ) |
      ( ( normal.normal_x >
          0 ) ? 1 : 0 ) ) + 6 << " " << dominantRegion << std::endl;
    std::cerr << dominantRegion - 6 << " " <<
    ( ( ( normal.normal_z >
          0 ) ? 1 : 0 ) <<
      2 ) << " " << normal.normal_z << " " <<
    ( ( ( normal.normal_y >
          0 ) ? 1 : 0 ) <<
      1 ) << " " << normal.normal_y << " " <<
    ( ( normal.normal_x > 0 ) ? 1 : 0 ) << " " << normal.normal_x << std::endl;
#endif
    regions.push_back ( dominantRegion );
  }

  switch ( regions[0] )
  {
  case 0:
    regions.push_back ( 7 );
    regions.push_back ( 9 );
    regions.push_back ( 11 );
    regions.push_back ( 13 );
    break;
  case 1:
    regions.push_back ( 6 );
    regions.push_back ( 8 );
    regions.push_back ( 10 );
    regions.push_back ( 12 );
    break;
  case 2:
    regions.push_back ( 8 );
    regions.push_back ( 9 );
    regions.push_back ( 12 );
    regions.push_back ( 13 );
    break;
  case 3:
    regions.push_back ( 6 );
    regions.push_back ( 7 );
    regions.push_back ( 10 );
    regions.push_back ( 11 );
    break;
  case 4:
    regions.push_back ( 10 );
    regions.push_back ( 11 );
    regions.push_back ( 12 );
    regions.push_back ( 13 );
    break;
  case 5:
    regions.push_back ( 6 );
    regions.push_back ( 7 );
    regions.push_back ( 8 );
    regions.push_back ( 9 );
    break;
  case 6:
    regions.push_back ( 1 );
    regions.push_back ( 3 );
    regions.push_back ( 5 );

    regions.push_back ( 7 );
    regions.push_back ( 8 );
    regions.push_back ( 10 );
    break;
  case 7:
    regions.push_back ( 0 );
    regions.push_back ( 3 );
    regions.push_back ( 5 );

    regions.push_back ( 6 );
    regions.push_back ( 9 );
    regions.push_back ( 11 );
    break;
  case 8:
    regions.push_back ( 1 );
    regions.push_back ( 2 );
    regions.push_back ( 5 );

    regions.push_back ( 9 );
    regions.push_back ( 6 );
    regions.push_back ( 12 );
    break;
  case 9:
    regions.push_back ( 0 );
    regions.push_back ( 2 );
    regions.push_back ( 5 );

    regions.push_back ( 8 );
    regions.push_back ( 7 );
    regions.push_back ( 13 );
    break;
  case 10:
    regions.push_back ( 1 );
    regions.push_back ( 3 );
    regions.push_back ( 4 );

    regions.push_back ( 11 );
    regions.push_back ( 12 );
    regions.push_back ( 6 );
    break;
  case 11:
    regions.push_back ( 0 );
    regions.push_back ( 3 );
    regions.push_back ( 4 );

    regions.push_back ( 10 );
    regions.push_back ( 13 );
    regions.push_back ( 7 );
    break;
  case 12:
    regions.push_back ( 1 );
    regions.push_back ( 2 );
    regions.push_back ( 4 );

    regions.push_back ( 13 );
    regions.push_back ( 10 );
    regions.push_back ( 8 );
    break;
  case 13:
    regions.push_back ( 0 );
    regions.push_back ( 2 );
    regions.push_back ( 4 );

    regions.push_back ( 12 );
    regions.push_back ( 11 );
    regions.push_back ( 9 );
    break;
  default:
    assert ( 0 );
    break;
  }
}

pcl::PointNormal
region2Normal ( int region )
{
  pcl::PointNormal normal;

  normal.normal_x = 0;
  normal.normal_y = 0;
  normal.normal_z = 0;
  switch ( region )
  {
  case 0:
    normal.normal_x = 1;
    break;
  case 1:
    normal.normal_x = -1;
    break;
  case 2:
    normal.normal_y = 1;
    break;
  case 3:
    normal.normal_y = -1;
    break;
  case 4:
    normal.normal_z = 1;
    break;
  case 5:
    normal.normal_z = -1;
    break;
  case 6:
    normal.normal_x = -0.577350269;
    normal.normal_y = -0.577350269;
    normal.normal_z = -0.577350269;
    break;
  case 7:
    normal.normal_x = 0.577350269;
    normal.normal_y = -0.577350269;
    normal.normal_z = -0.577350269;
    break;
  case 8:
    normal.normal_x = -0.577350269;
    normal.normal_y = 0.577350269;
    normal.normal_z = -0.577350269;
    break;
  case 9:
    normal.normal_x = 0.577350269;
    normal.normal_y = 0.577350269;
    normal.normal_z = -0.577350269;
    break;
  case 10:
    normal.normal_x = -0.577350269;
    normal.normal_y = -0.577350269;
    normal.normal_z = 0.577350269;
    break;
  case 11:
    normal.normal_x = 0.577350269;
    normal.normal_y = -0.577350269;
    normal.normal_z = 0.577350269;
    break;
  case 12:
    normal.normal_x = -0.577350269;
    normal.normal_y = 0.577350269;
    normal.normal_z = 0.577350269;
    break;
  case 13:
    normal.normal_x = 0.577350269;
    normal.normal_y = 0.577350269;
    normal.normal_z = 0.577350269;
    break;
  default:
    assert ( 0 );
    break;
  }
  return normal;
}

void
generateLayers (
  const std::vector<std::vector<std::vector<pcl::PointNormal> > > & normalBins,
  std::vector<std::vector<cv::Mat> > * &
  positiveLayers,
  std::vector<std::vector<cv::Mat> > * &
  negativeLayers )
{
  positiveLayers =
    new std::vector<std::vector<cv::Mat> > ( 14,
                                             std::vector<cv::Mat> ( normalBins
                                                                    [0].size () ) );
  negativeLayers =
    new std::vector<std::vector<cv::Mat> > ( 14,
                                             std::vector<cv::Mat> ( normalBins
                                                                    [0].size () ) );
#pragma omp parallel for
  for ( int i = 0; i < positiveLayers->size (); ++i )
  {
    for ( size_t j = 0; j < ( *positiveLayers )[i].size (); ++j )
    {
      ( *positiveLayers )[i][j] = 1e20 * cv::Mat::ones (
        normalBins.size (), normalBins[0][0].size (), CV_32FC1 );
      ( *negativeLayers )[i][j] = 1e20 * cv::Mat::ones (
        normalBins.size (), normalBins[0][0].size (), CV_32FC1 );
    }
  }
#pragma omp parallel for
  for ( int i = 0; i < normalBins.size (); ++i )
  {
    std::vector<int> regions;
    for ( int j = 0; j < normalBins[i].size (); ++j )
    {
      for ( size_t k = 0; k < normalBins[i][j].size (); ++k )
      {
        const pcl::PointNormal & normal = normalBins[i][j][k];
        getRegions ( normal, regions );
        for ( size_t q = 0; q < regions.size (); ++q )
        {
          ( *positiveLayers )[regions[q]][j].at<float>( i, k ) = 0; //regions.size();
//          (*positiveLayers)[regions[q]][j_prime].at<float>(i, k) = abs(j - j_prime) * ((*positiveLayers)[regions[q]][j_prime].at<float>(i, k) + 1);
          ( *negativeLayers )[opposite ( regions[q] )][j].at<float>( i,
                                                                     k ) = 0; //regions.size();
//          (*negativeLayers)[opposite(regions[q])][j_prime].at<float>(i, k) = abs(j - j_prime) * ((*negativeLayers)[opposite(regions[q])][j_prime].at<float>(i, k) + 1);
          break;
        }
      }
    }
  }
}

void
depthTransform ( const std::vector<std::vector<cv::Mat> > & inputLayers,
                 std::vector<std::vector<cv::Mat> > * &     outputLayers )
{
  outputLayers = new std::vector<std::vector<cv::Mat> > (
    inputLayers.size (), std::vector<cv::Mat> ( inputLayers[0].size () ) );
#pragma omp parallel for
  for ( int i = 0; i < inputLayers.size (); ++i )
  {
    for ( size_t j = 0; j < inputLayers[i].size (); ++j )
    {
      ( *outputLayers )[i][j] =
        cv::Mat ( inputLayers[i][j].rows, inputLayers[i][j].cols, CV_32FC1 );
      for ( int y = 0; y < inputLayers[i][j].rows; ++y )
      {
        for ( int x = 0; x < inputLayers[i][j].cols; ++x )
        {
          ( *outputLayers )[i][j].at<float>( y,
                                             x ) = inputLayers[i][j].at<float>(
            y,
            x ) < 1e20 ? 1 : 0;
        }
      }
    }
  }
}

std::vector<float> dt ( const std::vector<float> & f, int n )
{
  std::vector<float> d ( n );
  std::vector<int> v ( n );
  std::vector<float> z ( n + 1 );
  int k = 0;
  v[0] = 0;
  z[0] = -1e20;
  z[1] = 1e20;
  for ( int q = 1; q <= n - 1; q++ )
  {
    assert ( k >= 0 );
    assert ( k < n );
    assert ( v[k] >= 0 );
    assert ( v[k] < n );
    float s =
      ( ( f[q] + q * q ) - ( f[v[k]] + v[k] * v[k] ) ) / ( 2 * q - 2 * v[k] );
    while ( s <= z[k] )
    {
      assert ( k >= 0 );
      assert ( k < n );
      assert ( v[k] >= 0 );
      assert ( v[k] < n );
      k--;
      s =
        ( ( f[q] + q * q ) - ( f[v[k]] + v[k] * v[k] ) ) / ( 2 * q - 2 * v[k] );
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e20;
  }

  k = 0;
  for ( int q = 0; q <= n - 1; q++ )
  {
    while ( z[k + 1] < q )
    {
      k++;
    }
    d[q] = ( q - v[k] ) * ( q - v[k] ) + f[v[k]];
  }

  return d;
}

void
distanceTransform ( const std::vector<std::vector<cv::Mat> > & inputLayers,
                    std::vector<std::vector<cv::Mat> > * &     outputLayers )
{
  outputLayers = new std::vector<std::vector<cv::Mat> > (
    inputLayers.size (), std::vector<cv::Mat> ( inputLayers[0].size () ) );
#pragma omp parallel for
  for ( int i = 0; i < inputLayers.size (); ++i )
  {
    for ( size_t j = 0; j < inputLayers[i].size (); ++j )
    {
      ( *outputLayers )[i][j] =
        cv::Mat ( inputLayers[i][j].rows, inputLayers[i][j].cols, CV_32FC1 );
    }

    int width = inputLayers[i][0].cols;
    int height = inputLayers[i][0].rows;
    std::vector<float> f ( std::max ( std::max ( width,
                                                 height ),
                                      ( int )inputLayers[i].size () ) );
    // transform along columns
    for ( int x = 0; x < width; x++ )
    {
      for ( int z = 0; z < inputLayers[i].size (); ++z )
      {
        for ( int y = 0; y < height; y++ )
        {
          f[y] = inputLayers[i][z].at<float> ( y, x );
        }
        std::vector<float> d = dt ( f, height );
        for ( int y = 0; y < height; y++ )
        {
          ( *outputLayers )[i][z].at<float>( y, x ) = d[y];
        }
      }
    }

    // transform along rows
    for ( int y = 0; y < height; y++ )
    {
      for ( int z = 0; z < inputLayers[i].size (); ++z )
      {
        for ( int x = 0; x < width; x++ )
        {
          f[x] = ( *outputLayers )[i][z].at<float>( y, x );
        }
        std::vector<float> d = dt ( f, width );
        for ( int x = 0; x < width; x++ )
        {
          ( *outputLayers )[i][z].at<float> ( y, x ) = d[x];
        }
      }
    }

    for ( int y = 0; y < height; y++ )
    {
      for ( int x = 0; x < width; x++ )
      {
        for ( int z = 0; z < inputLayers[i].size (); ++z )
        {
          f[z] = ( *outputLayers )[i][z].at<float>( y, x );
        }
        std::vector<float> d = dt ( f, width );
        for ( int z = 0; z < inputLayers[i].size (); z++ )
        {
          ( *outputLayers )[i][z].at<float> ( y, x ) = sqrt ( d[z] );
        }
      }
    }
  }
}

void
distanceTransform (
  const std::vector<std::vector<cv::Mat> > & inputPositiveLayers,
  const std::vector<std::vector<cv::Mat> > &
  inputNegativeLayers,
  std::vector<std::vector<cv::Mat> > * &     outputLayers )
{
  std::vector<std::vector<cv::Mat> > * posResult, * negResult;
  distanceTransform ( inputPositiveLayers, posResult );
  distanceTransform ( inputNegativeLayers, negResult );
  outputLayers = new std::vector<std::vector<cv::Mat> > (
    posResult->size (), std::vector<cv::Mat> ( ( *posResult )[0].size () ) );
  for ( size_t i = 0; i < outputLayers->size (); ++i )
  {
    for ( size_t j = 0; j < ( *outputLayers )[i].size (); ++j )
    {
      ( *outputLayers )[i][j] = ( *posResult )[i][j]; // - negResult[i][j];
    }
  }
  delete posResult;
  delete negResult;
}


std::pair<int, int>
findResponse (
  const std::vector<std::vector<cv::Mat> > & targetLayers,
  int                                        yOffset,
  const std::vector<std::vector<cv::Mat> > & sourceLayers,
  const std::vector<std::vector<int> > &
  targetNormalizationMap,
  const std::vector<std::vector<int> > &
  sourceNormalizationMap,
  float &                                    max,
  cv::Mat &                                  sum )
{
  assert ( targetLayers.size () == sourceLayers.size () );
  sum = cv::Mat::zeros ( targetLayers[0][0].rows,
                         targetLayers[0][0].cols,
                         CV_32FC1 );
#pragma omp parallel for
  for ( int i = 0; i < targetLayers.size (); ++i )
  {
    for ( size_t j = 0;
          j < targetLayers[i].size () - yOffset && j < sourceLayers[i].size ();
          ++j )
    {
      cv::Mat convolved ( targetLayers[i][j].rows,
                          targetLayers[i][j].cols,
                          CV_32FC1 );
      cv::Mat filter ( sourceLayers[i][j].rows,
                       sourceLayers[i][j].cols,
                       CV_32FC1 );
      cv::flip ( sourceLayers[i][j], filter, -1 );
      cv::filter2D ( targetLayers[i][j + yOffset],
                     convolved,
                     -1,
                     filter,
                     cv::Point ( 0, 0 ),
                     0,
                     cv::BORDER_CONSTANT );
#pragma omp critical
      {
        sum += convolved;
      }
    }
  }
  int x = 0;
  int y = 0;
  for ( int j = 0; j < sum.rows - sourceLayers[0][0].rows; ++j )
  {
    for ( int i = 0; i < sum.cols - sourceLayers[0][0].cols; ++i )
    {
      float adjusted = sum.at<float> ( j, i ); //(targetNormalizationMap[j][i] == 0 ? 0.0 : sum.at<float>(j, i)/(targetNormalizationMap[j][i]));
      if ( adjusted < max )
      {
        max = adjusted;
        x = i;
        y = j;
      }
    }
  }
  return std::make_pair ( y, x );
}

void
moveSource ( pcl::PointCloud<pcl::PointXYZRGBNormal> & points )
{
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator i = points.begin ();
        i != points.end ();
        ++i )
  {
    i->x += 20 + 2 * ( float )rand () / ( ( float )RAND_MAX + 1 );
    i->y += 40 + 2 * ( float )rand () / ( ( float )RAND_MAX + 1 );
    i->z += -10 + 2 * ( float )rand () / ( ( float )RAND_MAX + 1 );
  }
}

void
saveBinnedNormals (
  const std::string & filename,
  const std::vector<std::vector<std::vector<pcl::PointNormal> > >
  &                   normalBins )
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
#pragma omp parallel for
  for ( int i = 0; i < normalBins.size (); ++i )
  {
    for ( int j = 0; j < normalBins[i].size (); ++j )
    {
      for ( int k = 0; k < normalBins[i][j].size (); ++k )
      {
        pcl::PointXYZRGBNormal point;
        point.x = 2 * i;
        point.y = 2 * j;
        point.z = 2 * k;
        RgbConverter c;
        c.r = ( uchar )( 127 * normalBins[i][j][k].normal_x + 128 );
        c.g = ( uchar )( 127 * normalBins[i][j][k].normal_y + 128 );
        c.b = ( uchar )( 127 * normalBins[i][j][k].normal_z + 128 );
        point.rgb = c.rgb;
        point.normal_x = normalBins[i][j][k].normal_x;
        point.normal_y = normalBins[i][j][k].normal_y;
        point.normal_z = normalBins[i][j][k].normal_z;
        if ( point.normal_x == point.normal_x && point.normal_y ==
             point.normal_y && point.normal_z == point.normal_z )
        {
#pragma omp critical
          {
            cloud.push_back ( point );
          }
        }
      }
    }
  }
  savePlyFile ( filename, cloud );
}

void
saveResponseMap ( const std::string & filename, const cv::Mat & responseMap )
{
  std::ofstream output ( filename.c_str () );

  if ( !output )
  {
    std::cout << "Failed to open " << filename << " for output" << std::endl;
    exit ( 1 );
  }

  for ( int j = 0; j < responseMap.rows; ++j )
  {
    for ( int i = 0; i < responseMap.cols; ++i )
    {
      output << responseMap.at<float> ( j, responseMap.cols - i - 1 ) << " ";
    }
    output << std::endl;
  }
  output.close ();
}

void saveLayers_char ( const std::string &                        filename,
                       const std::vector<std::vector<cv::Mat> > & layers )
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
#pragma omp parallel for
  for ( int i = 0; i < layers.size (); ++i )
  {
    cv::Mat sum = cv::Mat::zeros ( layers[i][0].rows,
                                   layers[i][0].cols,
                                   CV_32FC1 );
    for ( int j = 0; j < layers[i].size (); ++j )
    {
      for ( int k = 0; k < layers[i][j].rows; ++k )
      {
        for ( int l = 0; l < layers[i][j].cols; ++l )
        {
#if 0
          if ( layers[i][j].at<char>( k, l ) )
          {
            pcl::PointXYZRGBNormal point;
            point.x = 2 * k;
            point.y = 2 * j;
            point.z = 2 * l;
            RgbConverter c;
            pcl::PointNormal normal = region2Normal ( i );
            point.normal_x = normal.normal_x *
                             layers[i][j].at<char>( k, l ) / 127;
            point.normal_y = normal.normal_y *
                             layers[i][j].at<char>( k, l ) / 127;
            point.normal_z = normal.normal_z *
                             layers[i][j].at<char>( k, l ) / 127;
            c.r = ( uchar )( 127 * point.normal_x + 128 );
            c.g = ( uchar )( 127 * point.normal_y + 128 );
            c.b = ( uchar )( 127 * point.normal_z + 128 );
            point.rgb = c.rgb;
 #pragma omp critical
            {
              cloud.push_back ( point );
            }
          }
#endif
          sum.at<float> ( k, l ) += layers [i][j].at<char> ( k, l );
        }
      }
    }
    std::stringstream ss;
    ss << "/home/kmatzen/NOBACKUP/" << filename << ".layer" << i << ".txt";
    saveResponseMap ( ss.str (), sum );
  }
  savePlyFile ( filename, cloud );
}

void saveLayers_float ( const std::string &                        filename,
                        const std::vector<std::vector<cv::Mat> > & layers )
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
#pragma omp parallel for
  for ( int i = 0; i < layers.size (); ++i )
  {
    cv::Mat sum = cv::Mat::zeros ( layers[i][0].rows,
                                   layers[i][0].cols,
                                   CV_32FC1 );
    for ( int j = 0; j < layers[i].size (); ++j )
    {
      for ( int k = 0; k < layers[i][j].rows; ++k )
      {
        for ( int l = 0; l < layers[i][j].cols; ++l )
        {
#if 0
          if ( layers[i][j].at<float>( k, l ) )
          {
            pcl::PointXYZRGBNormal point;
            point.x = 2 * k;
            point.y = 2 * j;
            point.z = 2 * l;
            RgbConverter c;
            pcl::PointNormal normal = region2Normal ( i );
            assert ( layers[i][j].at<float>( k, l ) > 0 );
            point.normal_x = normal.normal_x * layers[i][j].at<float>( k, l );
            point.normal_y = normal.normal_y * layers[i][j].at<float>( k, l );
            point.normal_z = normal.normal_z * layers[i][j].at<float>( k, l );
            c.r = ( uchar )( 127 * point.normal_x + 128 );
            c.g = ( uchar )( 127 * point.normal_y + 128 );
            c.b = ( uchar )( 127 * point.normal_z + 128 );
            point.rgb = c.rgb;
 #pragma omp critical
            {
              cloud.push_back ( point );
            }
          }
#endif
          sum.at<float> ( k, l ) += layers [i][j].at<float> ( k, l );
        }
      }
    }
    std::stringstream ss;
    ss << "/home/kmatzen/NOBACKUP/" << filename << ".layer_dist" << i <<
    ".txt";
    saveResponseMap ( ss.str (), sum );
  }
  savePlyFile ( filename, cloud );
}

void
saveNormalizationMap ( const std::string &                    filename,
                       const std::vector<std::vector<int> > & normalMap )
{
  std::ofstream output ( filename.c_str () );

  if ( !output )
  {
    std::cout << "failed to open file " << filename << " for write" <<
    std::endl;
    exit ( 1 );
  }
  for ( int i = 0; i < normalMap.size (); ++i )
  {
    for ( int j = 0; j < normalMap[i].size (); ++j )
    {
      output << normalMap [i][j] << " ";
    }
    output << std::endl;
  }
  output.close ();
}

int
main ( int argc, char * * argv )
{
  struct timeval tic, toc;

  if ( argc < 5 )
  {
    std::cout << "Usage: " << argv[0] <<
    " [target].pcd [source].pcd [voxel_size] [output]" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZRGBNormal> source;
  pcl::PointCloud<pcl::PointXYZRGBNormal> * target =
    new pcl::PointCloud<pcl::PointXYZRGBNormal>();

  std::cout << "Loading target file " << argv[1] << std::flush;
  gettimeofday ( &tic, NULL );
  if ( -1 == pcl::io::loadPCDFile ( argv[1], *target ) )
  {
    std::cout << "Could not load target PCD " << argv[1] << std::endl;
    return -1;
  }
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;

  std::cout << "Loading source file " << argv[2] << std::flush;
  gettimeofday ( &tic, NULL );
  if ( -1 == pcl::io::loadPCDFile ( argv[2], source ) )
  {
    std::cout << "Could not load source PCD " << argv[2] << std::endl;
    return -1;
  }
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;


#if 0
  moveSource ( target );
#endif

  float voxel_size = atof ( argv[3] );
  if ( voxel_size == 0 )
  {
    std::cout << "voxel_size " << argv[3] << " is invalid" << std::endl;
    return -1;
  }
  cv::Vec3f targetMin, targetMax;

  std::cout << "Finding target AABB " << std::flush;
  gettimeofday ( &tic, NULL );
  findAABB ( *target, targetMin, targetMax );
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;

/*(  std::cout << "Finding floor adjustment " << std::flush;
 * gettimeofday (&tic, NULL);
 * float floorAdjust = findFloor (target, (targetMax [1] + targetMin [1])/2.0);
 * gettimeofday (&toc, NULL);
 * std::cout << "complete in " << toc.tv_sec - tic.tv_sec << " seconds" << std::endl;
 *
 * std::cout << "Floor adjustment is " << floorAdjust << std::endl;
 *
 * targetMin [1] = floorAdjust;
 */
  float lengthOfEdge = voxel_size;
  std::vector<std::vector<std::vector<pcl::PointNormal> > > * targetNormalBins
    = 0;

  std::vector<std::vector<cv::Mat> > * targetLayersPositiveBinary = 0;
  std::vector<std::vector<cv::Mat> > * targetLayersNegativeBinary = 0;
  std::vector<std::vector<cv::Mat> > * targetLayersFilter = 0;
  std::vector<std::vector<int> > targetNormalizationMap;

  std::cout << "Binning target normals " << std::flush;
  gettimeofday ( &tic, NULL );
  int yOffset = binNormals ( *target,
                             lengthOfEdge,
                             targetMin,
                             targetMax,
                             targetNormalBins,
                             targetNormalizationMap );
  delete target;
  target = 0;
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;

  std::cout << "y offset is " << yOffset << std::endl;

#if 1
  std::cout << "Saving normalization map" << std::endl;
  saveNormalizationMap ( "/home/kmatzen/NOBACKUP/normalization.txt",
                         targetNormalizationMap );
#endif

#if 0
  saveBinnedNormals ( "target.ply", targetNormalBins );
#endif

  std::cout << "Generating target layers " << std::flush;
  gettimeofday ( &tic, NULL );
  generateLayers ( *targetNormalBins,
                   targetLayersPositiveBinary,
                   targetLayersNegativeBinary );
  delete targetNormalBins;
  targetNormalBins = 0;
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;

#if 1
  saveLayers_float ( "targetLayers.ply", *targetLayersPositiveBinary );
#endif

  std::cout << "Distance transforming target layers " << std::flush;
  gettimeofday ( &tic, NULL );
  distanceTransform ( *targetLayersPositiveBinary,
                      *targetLayersNegativeBinary,
                      targetLayersFilter );
  delete targetLayersPositiveBinary;
  targetLayersPositiveBinary = 0;
  delete targetLayersNegativeBinary;
  targetLayersNegativeBinary = 0;
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;

#if 1
  std::cout << "Saving target distance transform " << std::flush;
  gettimeofday ( &tic, NULL );
  saveLayers_float ( "targetDistanceTransform.ply", *targetLayersFilter );
  gettimeofday ( &toc, NULL );
  std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
  std::endl;
#endif

  float max = 1e20;
  cv::Mat max_trans;
  std::pair<int, int> max_coord;
  cv::Vec3f sourceMin_max;
#pragma omp parallel for
  for ( int m = 0; m < 8; ++m )
  {
    bool a1 = m & 0x1;
    bool a2 = ( m >> 1 ) & 0x1;
    bool a3 = ( m >> 2 ) & 0x1;
    cv::Mat trans = cv::Mat::zeros ( 3, 3, CV_32FC1 );
    trans.at<float> ( 1, 1 ) = 1.0;
    if ( a1 )
    {
      trans.at<float> ( 0, 0 ) = a2 ? 1.0 : -1.0;
      trans.at<float> ( 2, 2 ) = a3 ? 1.0 : -1.0;
    }
    else
    {
      trans.at<float> ( 2, 0 ) = a2 ? 1.0 : -1.0;
      trans.at<float> ( 0, 2 ) = a3 ? 1.0 : -1.0;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> * sourceTrans =
      new pcl::PointCloud<pcl::PointXYZRGBNormal>();

    for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator j =
            source.begin ();
          j != source.end ();
          ++j )
    {
      pcl::PointXYZRGBNormal newPoint = *j;
      float x =
        trans.at<float> ( 0,
                          0 ) * newPoint.x +
        trans.at<float> ( 0, 1 ) * newPoint.y + trans.at<float> (
          0,
          2 ) * newPoint.z;
      float y =
        trans.at<float> ( 1,
                          0 ) * newPoint.x +
        trans.at<float> ( 1, 1 ) * newPoint.y + trans.at<float> (
          1,
          2 ) * newPoint.z;
      float z =
        trans.at<float> ( 2,
                          0 ) * newPoint.x +
        trans.at<float> ( 2, 1 ) * newPoint.y + trans.at<float> (
          2,
          2 ) * newPoint.z;
      float nx = trans.at<float> ( 0, 0 ) * newPoint.normal_x + trans.at<float> (
        0,
        1 ) * newPoint.normal_y + trans.at<float> ( 0, 2 ) * newPoint.normal_z;
      float ny = trans.at<float> ( 1, 0 ) * newPoint.normal_x + trans.at<float> (
        1,
        1 ) * newPoint.normal_y + trans.at<float> ( 1, 2 ) * newPoint.normal_z;
      float nz = trans.at<float> ( 2, 0 ) * newPoint.normal_x + trans.at<float> (
        2,
        1 ) * newPoint.normal_y + trans.at<float> ( 2, 2 ) * newPoint.normal_z;
      newPoint.x = x;
      newPoint.y = y;
      newPoint.z = z;
      newPoint.normal_x = nx;
      newPoint.normal_y = ny;
      newPoint.normal_z = nz;
      sourceTrans->push_back ( newPoint );
    }

    cv::Vec3f sourceMin, sourceMax;

    std::cout << "Finding source AABB " << std::flush;
    gettimeofday ( &tic, NULL );
    findAABB ( *sourceTrans, sourceMin, sourceMax );
    gettimeofday ( &toc, NULL );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
    std::endl;

    std::vector<std::vector<std::vector<pcl::PointNormal> > > *
    sourceNormalBins = 0;
    std::vector<std::vector<cv::Mat> > * sourceLayersPositiveBinary = 0;
    std::vector<std::vector<cv::Mat> > * sourceLayersNegativeBinary = 0;
    std::vector<std::vector<cv::Mat> > * sourceLayersFilter = 0;
    std::vector<std::vector<int> > sourceNormalizationMap;

    std::cout << "Binning source normals " << std::flush;
    gettimeofday ( &tic, NULL );
    binNormals ( *sourceTrans,
                 lengthOfEdge,
                 sourceMin,
                 sourceMax,
                 sourceNormalBins,
                 sourceNormalizationMap );
    delete sourceTrans;
    sourceTrans = 0;
    gettimeofday ( &toc, NULL );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
    std::endl;

#if 0
    saveBinnedNormals ( "source.ply", sourceNormalBins );
#endif

    std::cout << "Generating source layers " << std::flush;
    gettimeofday ( &tic, NULL );
    generateLayers ( *sourceNormalBins,
                     sourceLayersPositiveBinary,
                     sourceLayersNegativeBinary );
    delete sourceNormalBins;
    sourceNormalBins = 0;
    gettimeofday ( &toc, NULL );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
    std::endl;

#if 1
    saveLayers_char ( "sourceLayers.ply", *sourceLayersPositiveBinary );
#endif

    std::cout << "Distance transforming source layers " << std::flush;
    gettimeofday ( &tic, NULL );
//  distanceTransform (sourceLayersPositiveBinary, sourceLayersNegativeBinary, sourceLayersFilter);
    depthTransform ( *sourceLayersPositiveBinary, sourceLayersFilter );
    delete sourceLayersPositiveBinary;
    sourceLayersPositiveBinary = 0;
    delete sourceLayersNegativeBinary;
    sourceLayersNegativeBinary = 0;
    gettimeofday ( &toc, NULL );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
    std::endl;

#if 1
    std::cout << "Saving source distance transform " << std::flush;
    gettimeofday ( &tic, NULL );
    saveLayers_float ( "sourceDistanceTransform.ply", *sourceLayersFilter );
    gettimeofday ( &toc, NULL );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
    std::endl;
#endif

    std::cout << "Finding response " << std::flush;
    gettimeofday ( &tic, NULL );
    float max_temp = 1e20;
    cv::Mat sum;
    std::pair<int, int> response = findResponse ( *targetLayersFilter,
                                                  yOffset,
                                                  *sourceLayersFilter,
                                                  targetNormalizationMap,
                                                  sourceNormalizationMap,
                                                  max_temp,
                                                  sum );
    delete sourceLayersFilter;
    sourceLayersFilter = 0;
#pragma omp critical
    {
      if ( max_temp < max )
      {
        max = max_temp;
        max_trans = trans;
        max_coord = response;
        sourceMin_max = sourceMin;
      }
      for ( int y = 0; y < 3; ++y )
      {
        for ( int x = 0; x < 3; ++x )
        {
          std::cout << trans.at<float> ( y, x ) << " ";
        }
        std::cout << std::endl;
      }
      std::cout << response.first << " " << response.second << " " <<
      max_temp << std::endl;
    }
    gettimeofday ( &toc, NULL );
    std::cout << " complete in " << toc.tv_sec - tic.tv_sec << " seconds" <<
    std::endl;

#if 1
    std::stringstream ss;
    ss << argv[4] << "_response" << m << ".txt";
    saveResponseMap ( ss.str (), sum );
#endif
  }

  std::cout << "Chosen trans: " << std::endl;
  for ( int j = 0; j < 3; ++j )
  {
    for ( int i = 0; i < 3; ++i )
    {
      std::cout << max_trans.at<float> ( j, i ) << " ";
    }
    std::cout << std::endl;
  }

  delete targetLayersFilter;
  targetLayersFilter = 0;

  pcl::PointCloud<pcl::PointXYZRGBNormal> output;
  for ( pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i =
          source.begin ();
        i != source.end ();
        ++i )
  {
    pcl::PointXYZRGBNormal newPoint = *i;
    float x = max_trans.at<float> ( 0, 0 ) * newPoint.x + max_trans.at<float> (
      0,
      1 ) * newPoint.y + max_trans.at<float> ( 0, 2 ) * newPoint.z;
    float y = max_trans.at<float> ( 1, 0 ) * newPoint.x + max_trans.at<float> (
      1,
      1 ) * newPoint.y + max_trans.at<float> ( 1, 2 ) * newPoint.z;
    float z = max_trans.at<float> ( 2, 0 ) * newPoint.x + max_trans.at<float> (
      2,
      1 ) * newPoint.y + max_trans.at<float> ( 2, 2 ) * newPoint.z;
    float nx =
      max_trans.at<float> ( 0, 0 ) * newPoint.normal_x + max_trans.at<float> (
        0,
        1 )
      * newPoint.normal_y + max_trans.at<float> ( 0, 2 ) * newPoint.normal_z;
    float ny =
      max_trans.at<float> ( 1, 0 ) * newPoint.normal_x + max_trans.at<float> (
        1,
        1 )
      * newPoint.normal_y + max_trans.at<float> ( 1, 2 ) * newPoint.normal_z;
    float nz =
      max_trans.at<float> ( 2, 0 ) * newPoint.normal_x + max_trans.at<float> (
        2,
        1 )
      * newPoint.normal_y + max_trans.at<float> ( 2, 2 ) * newPoint.normal_z;
    newPoint.x = x;
    newPoint.y = y;
    newPoint.z = z;
    newPoint.normal_x = nx;
    newPoint.normal_y = ny;
    newPoint.normal_z = nz;
    newPoint.x += lengthOfEdge * max_coord.first - sourceMin_max [0] +
                  targetMin [0];
    newPoint.y += lengthOfEdge * yOffset - sourceMin_max [1] + targetMin [1];
    newPoint.z += lengthOfEdge * max_coord.second - sourceMin_max [2] +
                  targetMin [2];
    output.push_back ( newPoint );
  }
  std::stringstream ss;
  ss << argv[4] << ".ply";
  savePlyFile ( ss.str (), output );
  return 0;
}
