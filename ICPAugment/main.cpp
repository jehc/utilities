#include <iostream>
#include <string>
#include <set>
#include <fstream>
#include <cmath>

#if 0
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#endif

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <sys/time.h>
#include <getopt.h>

#include <opencv2/opencv.hpp>

#include "ply_io.h"
#include "BundleFile.h"
#include "CoordsFile.h"
#include "TracksFile.h"

#define TIMED struct timeval tic, toc;
#define TIME_BEGIN( msg ) std::cout << msg << std::endl; gettimeofday ( &tic, 0 );
#define TIME_END gettimeofday ( &toc, \
  0 ); std::cout << "Complete in " << toc.tv_sec - tic.tv_sec << " seconds." << std::endl;

void
printCommand ( int argc, char * * argv )
{ 
  for ( int i = 0; i < argc; ++i ) 
  {
    std::cout << argv [i] << " ";
  } 
  std::cout << std::endl;
} 
  
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr LoadCloud (
  const std::string &     colorFilename,
  const std::string &     depthFilename,
  const BundleCamera &    camera,
  double                   scale,
  const cv::Mat & a, const cv::Mat & b, const cv::Mat & c,
#if 0
  std::vector<double> & variances,
#endif
  std::vector<std::pair<double,double> > & indices)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points ( new pcl::PointCloud<pcl::PointXYZRGBNormal>() );

  cv::Mat colorImageDist = cv::imread ( colorFilename );
  
  std::ifstream depthInput ( depthFilename.c_str () );
  if ( !depthInput )
  {
    std::cout << "Could not load file " << depthFilename << std::endl;
    return points;
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
  cameraMatrixCV.at<float> ( 0, 2 ) = colorImageDist.cols / 2.0;
  cameraMatrixCV.at<float> ( 1, 0 ) = 0;
  cameraMatrixCV.at<float> ( 1, 1 ) = camera.GetF ();
  cameraMatrixCV.at<float> ( 1, 2 ) = colorImageDist.rows / 2.0;
  cameraMatrixCV.at<float> ( 2, 0 ) = 0;
  cameraMatrixCV.at<float> ( 2, 1 ) = 0;
  cameraMatrixCV.at<float> ( 2, 2 ) = 1;

  Eigen::Matrix3d cameraMatrix;
  cameraMatrix << camera.GetF (), 0, colorImageDist.cols / 2.0,
  0, camera.GetF (), colorImageDist.rows / 2.0,
  0, 0, 1;

  cv::Mat distCoeffs ( 5, 1, CV_32FC1 );
  distCoeffs.at<float> ( 0, 0 ) = camera.GetK1 ();
  distCoeffs.at<float> ( 1, 0 ) = camera.GetK2 ();
  distCoeffs.at<float> ( 2, 0 ) = 0;
  distCoeffs.at<float> ( 3, 0 ) = 0;
  distCoeffs.at<float> ( 4, 0 ) = 0;

  cv::Mat colorImage, depthMap;

  cv::undistort ( colorImageDist, colorImage, cameraMatrixCV, distCoeffs );

  //Linear interpolation; do not want
  //cv::undistort (depthMapDist, depthMap, cameraMatrix, distCoeffs);

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap ( cameraMatrixCV, distCoeffs, cv::Mat::eye ( 3,
      3,
      CV_32FC1 ), cameraMatrixCV, depthMapDist.size (), CV_32FC1, map1, map2 );
  cv::remap ( depthMapDist, depthMap, map1, map2, cv::INTER_NEAREST );

  const Eigen::Matrix3d & R = camera.GetR ();
  const Eigen::Vector3d & t = camera.GetT ();

  for (int j = 0; j < depthMap.rows; ++j)
  {
    for (int i = 0; i < depthMap.cols; ++i)
    {
      double depthRaw = depthMap.at<float>(j, i);
      if (depthRaw != 0)
      {
        depthMap.at<float>(j, i) = a.at<float>(j, i)*pow(depthRaw, 2.0) + b.at<float>(j, i)*depthRaw + c.at<float>(j, i);
      }
    }
  }

  for ( int indexJ = 0; indexJ < depthMap.rows; indexJ += 4)
  {
    for ( int indexI = 0; indexI < depthMap.cols; indexI += 4)
    {
#if 0
      double sum = 0;
      double sum_sq = 0;
      int n = 0;
      for (int i = std::max(indexI - 5, 0); i <= std::min(indexI + 5, depthMap.cols-1); ++i)
      {
        for (int j = std::max(indexJ - 5, 0); j <= std::min(indexJ + 5, depthMap.rows-1); ++j)
        {
          double depth = depthMap.at<float>(j, i);
          if (isnan(depth) || isinf(depth) || depth <= 0)
          {
            continue;
          }
          sum += depth;
          ++n;
        }
      }
      if (n < 2)
      {
        continue;
      }
      double mean = sum/n;
      assert (mean >= 0);
      for (int i = std::max(indexI - 5, 0); i <= std::min(indexI + 5, depthMap.cols-1); ++i)
      {
        for (int j = std::max(indexJ - 5, 0); j <= std::min(indexJ + 5, depthMap.rows-1); ++j)
        {
          double depth = depthMap.at<float>(j, i);
          if (isnan(depth) || isinf(depth) || depth <= 0)
          {
            continue;
          }
          sum_sq += pow(depth - mean, 2.0);
        }
      }

      double var = sum_sq/(n-1);
#else
      double mean = depthMap.at<float>(indexJ, indexI);
      if (isnan(mean) || isinf(mean) || mean <= 0.2)
      {
        continue;
      }
#endif

      Eigen::Vector3d p ( ( double )indexI, (double)depthMap.rows - 1 - ( double )indexJ, 1 );
      p = scale * mean * p;

      p = cameraMatrix.inverse () * p;
      p[2] *= -1;

      Eigen::Vector3d diff = -p;
      diff.normalize ();

      p = R.transpose()*(p - t);
      diff = R.transpose()*diff;

      pcl::PointXYZRGBNormal point;
      point.x = p[0];
      point.y = p[1];
      point.z = p[2];

      point.normal_x = diff [0];
      point.normal_y = diff [1];
      point.normal_z = diff [2];

      if ( isnan ( point.x ) || isnan ( point.y ) || isnan ( point.z ) )
      {
        std::cout << "NaN found in point cloud" << std::endl;
        std::cout << "Point: " << p << std::endl;
        std::cout << "i: " << indexI << std::endl;
        std::cout << "j: " << indexJ << std::endl;
        std::cout << "depth: " << mean << std::endl;
        std::cout << "z-buffer: " << depthMap.at<float> ( indexJ, indexI ) << std::endl;
        std::cout << "R: " << R << std::endl;
        std::cout << "t: " << t << std::endl;
        std::cout << "scale: " << scale << std::endl;
      }

      const cv::Vec3b & color = colorImage.at<cv::Vec3b>( indexJ, indexI );
      RgbConverter c;
      c.r = color[2];
      c.g = color[1];
      c.b = color[0];
      point.rgb = c.rgb;
      points->push_back (point);
#if 0
      variances.push_back (var);
#endif
      indices.push_back (std::make_pair(indexI, indexJ));
    }
  }

  return points;
}

struct Options
{
  std::string bundleFile;
  std::string listFile;
  std::string output;
  std::string scale;
  std::string coordsFile;
  std::string tracksFile;
  bool highres;

  void
  help () const
  {
    std::cout << "  --bundle [string]" << std::endl;
    std::cout << "  --list [string]" << std::endl;
    std::cout << "  --coords [string]" << std::endl;
    std::cout << "  --tracks [string]" << std::endl;
    std::cout << "  --output [string]" << std::endl;
    std::cout << "  --scale [string]" << std::endl;
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
      { "scale", required_argument,         0,                         's'                                         },
      { "coords", required_argument, 0, 'c'},
      { "tracks", required_argument, 0, 't'},
      { 0,              0,                         0,                         0                                           } };

    bool bundleFileSet = false;
    bool listFileSet = false;
    bool outputSet = false;
    bool scaleSet = false;
    bool coordsSet = false;
    bool tracksSet = false;

    while ( ( c = getopt_long ( argc, argv, "b:l:o:s:c:t:", longopts, &indexptr ) ) != -1 )
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
      case 'c':
        coordsSet = true;
        coordsFile = std::string (optarg);
        break;
      case 't':
        tracksSet = true;
        tracksFile = std::string (optarg);
        break;
      case 'o':
        outputSet = true;
        output = std::string ( optarg );
        break;
      case 's':
        scaleSet = true;
        scale = std::string (optarg);
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
    if ( !scaleSet )
    {
      std::cout << "scale option is required" << std::endl;
      help();
    }

    if ( !outputSet )
    {
      std::cout << "output option is required" << std::endl;
      help ();
    }
    if (!coordsSet) 
    {
      std::cout << "coords option is required" << std::endl;
      help();
    }

    if (!tracksSet)
    {
      std::cout << "tracks option is requried" << std::endl;
      help();
    }
  }
};

Eigen::Matrix3d
get_rotation (const std::vector<Eigen::Vector3d> & right, const std::vector<Eigen::Vector3d> & left)
{
  Eigen::Matrix3d A;
  A.setZero();
  
  for (size_t i = 0; i < right.size(); ++i)
  {
    A += left [i] * right [i].transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd (A, Eigen::ComputeFullU|Eigen::ComputeFullV);

  Eigen::Matrix3d R;
#pragma omp critical  
  R = svd.matrixU().transpose() * svd.matrixV();

  if (R.determinant() < 0.0)
  {
    Eigen::Matrix3d reflect = Eigen::Matrix3d::Identity();
    reflect (2,2) = -1;
#pragma omp critical
    R = svd.matrixU().transpose() * reflect * svd.matrixV();
  }
  return R;
}

size_t
icp_align (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1, 
           pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2, 
           pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree1, 
#if 0
           const std::vector<double> & variances1, 
           const std::vector<double> & variances2, 
#endif
           const std::vector<std::pair<double,double> > & index_pairs1, 
           const std::vector<std::pair<double,double> > & index_pairs2, 
#if 0
           std::set<int> & keys1, 
           std::set<int> & keys2, 
           size_t maxKeys1, 
           size_t maxKeys2, 
#endif
           BundleFile & bundleFile,
           CoordsFile & coordsFile, 
           TracksFile & tracksFile, 
           int index1, 
           int index2,
           pcl::PointCloud<pcl::PointXYZRGB> & debugPoints) 
{
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr originalCloud2 = cloud2;
  bool converged = false;
  std::vector<int> pairs (cloud2->size(), -1);
  std::vector<double> errors (cloud2->size());
  double error = std::numeric_limits<double>::infinity();
  double maxError = error;
  int retryCount = 5;
  while (!converged)
  {
    std::vector<int> k_indices (1);
    std::vector<float> k_sqr_distances (1);
    for (size_t index = 0; index < cloud2->size(); ++index)
    {
      tree1->nearestKSearch (*cloud2, index, 1, k_indices, k_sqr_distances);
      pairs [index] = k_indices [0];
      errors [index] = k_sqr_distances [0];
    }
    std::vector<double> errorsSorted (errors);
    std::sort (errorsSorted.begin(), errorsSorted.end());
    maxError = errorsSorted [(int)(errorsSorted.size()*3.0/4.0)];
    if (sqrt(maxError) > 0.5)
    {
#if 0
      pcl::PointCloud<pcl::PointXYZRGBNormal> temp;
      temp += *cloud1;
      temp += *cloud2;
      std::stringstream ss;
      ss << index1 << "-" << index2 << "_" << maxError << ".ply";
      savePlyFile (ss.str(), temp);
#pragma omp critical
      std::cout << "Max error is " << maxError << std::endl;
#endif
      return 0;
    }
    double newError = 0.0;
    for (size_t i = 0; i <= errorsSorted.size()*3.0/4.0; ++i)
    {
      if (errorsSorted [i] < maxError)
      {
        newError += errorsSorted [i];
      }
    }
    if (error < newError)
    {
      --retryCount;
    }
    converged = !retryCount;
    error = newError;
//    std::cout << "Error: " << error << std::endl;

    Eigen::Vector3d centroid1;
    centroid1.setZero();
    Eigen::Vector3d centroid2;
    centroid2.setZero();
    double sum_num = 0.0;
    double sum_den = 0.0;
    double scale;

    for (size_t i = 0; i < pairs.size(); ++i)
    {
      if (errors [i] > maxError)
      {
        continue;
      }
      const pcl::PointXYZRGBNormal & point1 = cloud1->points[pairs[i]];
      centroid1 += Eigen::Vector3d (point1.x, point1.y, point1.z);
      const pcl::PointXYZRGBNormal & point2 = cloud2->points[i];
      centroid2 += Eigen::Vector3d (point2.x, point2.y, point2.z);
    }
    centroid1 /= pairs.size();
    centroid2 /= pairs.size();

    for (size_t i = 0; i < pairs.size(); ++i)
    {
      if (errors [i] > maxError)
      {
        continue;
      }
      const pcl::PointXYZRGBNormal & point1 = cloud1->points[pairs[i]];
      const pcl::PointXYZRGBNormal & point2 = cloud2->points[i];
      Eigen::Vector3d r = Eigen::Vector3d (point1.x, point1.y, point1.z) - centroid1;
      Eigen::Vector3d l = Eigen::Vector3d (point2.x, point2.y, point2.z) - centroid2;

      sum_num += r.dot(r);
      sum_den += l.dot(l);
    }

    scale = sqrt(sum_num / sum_den);

    std::vector<Eigen::Vector3d> right_zm;
    std::vector<Eigen::Vector3d> left_zm;
    for (size_t i = 0; i < pairs.size(); ++i)
    {
      if (errors[i] > maxError)
      {
        continue;
      }
      const pcl::PointXYZRGBNormal & point1 = cloud1->points[pairs[i]];
      const pcl::PointXYZRGBNormal & point2 = cloud2->points[i];
      Eigen::Vector3d r = Eigen::Vector3d (point1.x, point1.y, point1.z) - centroid1;
      Eigen::Vector3d l = Eigen::Vector3d (point2.x, point2.y, point2.z) - centroid2;

      right_zm.push_back (r);
      left_zm.push_back (scale*l);
    }

    Eigen::Matrix3d R = get_rotation (right_zm, left_zm);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr newCloud2 (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = cloud2->begin(); i != cloud2->end(); ++i)
    {
      pcl::PointXYZRGBNormal newPoint = *i;
      Eigen::Vector3d point (i->x, i->y, i->z);
      point = R*(point - centroid2) + centroid1;
      newPoint.x = point[0];
      newPoint.y = point[1];
      newPoint.z = point[2];
      newCloud2->push_back (newPoint);
    }
    cloud2 = newCloud2;
  }

  for (size_t i = 0; i < 25; ++i)
  {
    int index = (int)((double)pairs.size()*rand()/(RAND_MAX+1.0));
    assert (index >= 0);
    assert (index < (int)pairs.size());
    if (errors [index] > maxError)
    {
      continue;
    }
    const std::pair<double,double> & indices1 = index_pairs1 [pairs[index]];
    const std::pair<double,double> & indices2 = index_pairs2 [index];
    const pcl::PointXYZRGBNormal & point1 = cloud1->points[pairs[index]];
    const pcl::PointXYZRGBNormal & point2 = originalCloud2->points[index];
    RgbConverter c1;
    c1.rgb = point1.rgb;
    RgbConverter c2;
    c2.rgb = point2.rgb;
    Eigen::Vector3i color1 (c1.r, c1.g, c1.b);
    Eigen::Vector3i color2 (c2.r, c2.g, c2.b);
    size_t key1, key2;
#pragma omp critical
    {
      key1 = coordsFile.GetNextID (index1);
      key2 = coordsFile.GetNextID (index2);
      CoordEntry e1 (key1, indices1.first, indices1.second, 0, 0, color1);
      CoordEntry e2 (key2, indices2.first, indices2.second, 0, 0, color2);
      coordsFile.AddEntry (e1, index1);
      coordsFile.AddEntry (e2, index2);
      BundleView view1 (index1, key1, indices1.first, indices1.second);
      BundleView view2 (index2, key2, indices2.first, indices2.second);
      std::vector<BundleView> views;
      views.push_back (view1);
      views.push_back (view2);
      Eigen::Vector3d position1 (point1.x, point1.y, point1.z);
      Eigen::Vector3d position2 (point2.x, point2.y, point2.z);
      BundlePoint bundlePoint ((position1+position2)/2, (color1+color2)/2, views);
      bundleFile.AddPoint (bundlePoint);
      Track track;
      track.AddEntry (TrackEntry (index1, key1));
      track.AddEntry (TrackEntry (index2, key2));
      tracksFile.AddTrack (track);

      pcl::PointXYZRGB debugPoint;
      const Eigen::Vector3d & position = bundlePoint.GetPosition();
      const Eigen::Vector3i & color = bundlePoint.GetColor();
      debugPoint.x = position[0];
      debugPoint.y = position[1];
      debugPoint.z = position[2];
      RgbConverter c;
      c.r = color [0];
      c.g = color [1];
      c.b = color [2];
      debugPoint.rgb = c.rgb;
      debugPoints.push_back (debugPoint);
    }
  }
  return 25;
}

int
main ( int argc, char * * argv )
{
  TIMED

  printCommand ( argc, argv );

  TIME_BEGIN ( "Loading options" )
  Options options ( argc, argv );
  TIME_END

  TIME_BEGIN ( "Loading bundle file" )
  BundleFile bundleFile ( options.bundleFile );
  bundleFile.save (options.output + "/bundle_augmented.out");
  TIME_END

  TIME_BEGIN ("Loading coords file" )
  CoordsFile coordsFile (options.coordsFile);
  coordsFile.save (options.output + "/coords_augmented.txt");
  TIME_END

  TIME_BEGIN ("Loading tracks file")
  TracksFile tracksFile (options.tracksFile);
  tracksFile.save (options.output + "/tracks_augmented.txt");
  TIME_END

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

  std::ifstream scaleFile (options.scale.c_str());
  if (!scaleFile)
  {
    std::cout << "Failed to open scale file" << std::endl;
    options.help();
  }

  TIME_BEGIN ("Loading depth tuning")
  std::vector<double> scales;
  while (scaleFile)
  {
    double scale;
    if (!(scaleFile >> scale))
    {
      break;
    }
    scales.push_back (scale);
  }
  scaleFile.close();

  TIME_BEGIN ("Load correction curves")
  std::ifstream ain ("/hydra/S2/kmatzen/09July2011_calib/a.txt");
  std::ifstream bin ("/hydra/S2/kmatzen/09July2011_calib/b.txt");
  std::ifstream cin ("/hydra/S2/kmatzen/09July2011_calib/c.txt");
  assert (ain);
  assert (bin);
  assert (cin);
  cv::Mat a (480, 640, CV_32FC1);
  cv::Mat b (480, 640, CV_32FC1);
  cv::Mat c (480, 640, CV_32FC1);
  for (int i = 0; i < 640; ++i)
  {
    for (int j = 0; j < 480; ++j)
    {
      ain >> a.at<float>(j, i);
      bin >> b.at<float>(j, i);
      cin >> c.at<float>(j, i);
      assert (ain);
      assert (bin);
      assert (cin);
    }
  }
  TIME_END

  const std::vector<BundleCamera> & cameras = bundleFile.GetCameras();

  size_t list_index = options.listFile.find_last_of ( "/" );
  std::vector<std::string> colorFilenames (imageList.size());
  std::vector<std::string> depthFilenames (imageList.size());
#if 0
  std::vector<size_t> maxKeys (imageList.size());
#endif
  TIME_BEGIN ( "Generating point cloud list" )
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
    if (scales[i] < 1e-9)
    {
      continue;
    }
    std::string suffix = "color.jpg";
    size_t replacement = colorFilename.find ( suffix );
    if ( replacement == std::string::npos )
    {
      continue;
    }
    std::string depthFilename ( colorFilename );
    depthFilename.replace ( replacement, suffix.size(), "depth.raw" );
    colorFilenames [i] = colorFilename;
    depthFilenames [i] = depthFilename;

#if 0
    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    std::string keyFilename (colorFilename);
    keyFilename.replace (replacement, suffix.size(), "color.key");
    std::ifstream keyUncompFile (keyFilename.c_str());
    keyFilename.replace (replacement, suffix.size(), "color.key.gz");
    std::ifstream keyCompFile (keyFilename.c_str(), ios_base::in | ios_base::binary);
    if (!keyUncompFile)
    {
      in.push (boost::iostreams::gzip_decompressor());
      in.push (keyCompFile);
    }
    else
    {
      in.push (keyUncompFile);
    }

    std::istream keyFile (&in);

    if (!keyFile)
    {
      continue;
    }

    if (keyUncompFile)
    {
#pragma omp critical
      std::cout << "Reading key file" << std::endl;
    }
    else if (keyCompFile)
    {
#pragma omp critical
      std::cout << "Reading key.gz file" << std::endl;
    }
    else
    {
#pragma omp critical
      std::cout << "Failed to read key file" << std::endl;
    }

    int maxKey;
    if (!(keyFile >> maxKey))
    {
      continue;
    }

#pragma omp critical
    std::cout << "Found key file size of " << maxKey << std::endl;

    maxKeys [i] = maxKey;
    keyCompFile.close();
    keyUncompFile.close();
#endif
  }
  TIME_END

  TIME_BEGIN ("Build graph edges")
  std::vector<std::set<int> > edges (bundleFile.GetCameras().size());
  std::vector<std::multiset<int> > edgesMulti (bundleFile.GetCameras().size());
#if 0
  std::vector<std::set<int> > keys (file.GetCameras().size());
#endif
  const std::vector<BundlePoint> & points = bundleFile.GetPoints();
  pcl::PointCloud<pcl::PointXYZRGB> debugOriginal;
  for (size_t i = 0; i < points.size(); ++i)
  {
    const Eigen::Vector3d & position = points [i].GetPosition();
    const Eigen::Vector3i & color = points [i].GetColor();
    pcl::PointXYZRGB debugPoint;
    debugPoint.x = position [0];
    debugPoint.y = position [1];
    debugPoint.z = position [2];
    RgbConverter c;
    c.r = color [0];
    c.g = color [1];
    c.b = color [2];
    debugPoint.rgb = c.rgb;
    debugOriginal.push_back (debugPoint);
    const std::vector<BundleView> & views = points [i].GetViews();
    for (size_t j = 0; j < views.size(); ++j)
    {
      const BundleView & view1 = views [j];
#if 0
      keys [view1.GetCamera()].insert (view1.GetKey());
      if (maxKeys [view1.GetCamera()] == 0)
      {
        continue;
      }
      assert ((size_t)view1.GetKey() < maxKeys [view1.GetCamera()]);
#endif
      for (size_t k = j + 1; k < views.size(); ++k)
      {
        const BundleView & view2 = views [k];
        int camera1 = view1.GetCamera();
        int camera2 = view2.GetCamera();
#if 0
        if (maxKeys [camera2] == 0)
        {
          continue;
        }
#endif
        if (camera2 < camera1)
        {
          std::swap (camera1, camera2);
        }
        edges [camera1].insert (camera2);
        edgesMulti [camera1].insert (camera2);
      }
    }
  }
  savePlyFile ("debugOriginal.ply", debugOriginal);
  TIME_END

  std::vector<bool> completed (imageList.size());
  int count = 0;

  TIME_BEGIN ("All pairs ICP")
#pragma omp parallel for
  for (int i = 0; i < (int)edges.size(); ++i)
  {
    if (colorFilenames [i] == "" || depthFilenames [i] == "")
    {
      continue;
    }
#pragma omp critical
    std::cout << "Opening " << colorFilenames [i] << std::endl;
#if 0
    std::vector<double> variances1;
#endif
    std::vector<std::pair<double,double> > indices1;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1 = LoadCloud (
      colorFilenames [i],
      depthFilenames [i],
      cameras[i],
      scales[i],
      a, b, c, 
#if 0
      variances1, 
#endif
      indices1);
    if (cloud1->size() == 0)
    {
      continue;
    }

    pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>());
    tree->setInputCloud (cloud1);
    
    pcl::PointCloud<pcl::PointXYZRGB> debugPoints;
    int totalPoints = 0;
    
    for (std::set<int>::const_iterator j_iter = edges [i].begin(); j_iter != edges [i].end(); ++j_iter)
    {
      int j = *j_iter;
      if (edgesMulti [i].count (j) < 20)
      {
        continue;
      }
      bool skip;
#pragma omp critical
      skip = completed [j];
      if (skip)
      {
        continue;
      }
      if (colorFilenames [j] == "" || depthFilenames [j] == "")
      {
        continue;
      }
      std::cout << "Opening " << colorFilenames [j] << std::endl;
#if 0
      std::vector<double> variances2;
#endif
      std::vector<std::pair<double,double> > indices2;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2 = LoadCloud (
        colorFilenames [j],
        depthFilenames [j],
        cameras[j],
        scales[j],
        a, b, c, 
#if 0
        variances2, 
#endif
        indices2);
      if (cloud2->size() == 0)
      {
        continue;
      }
      int numPoints = icp_align (cloud1, 
                      cloud2, 
                      tree, 
#if 0
                      variances1, 
                      variances2, 
#endif
                      indices1, 
                      indices2, 
#if 0  
                      keys [i], 
                      keys [j], 
                      maxKeys [i], 
                      maxKeys [j], 
#endif
                      bundleFile,
                      coordsFile, 
                      tracksFile, 
                      i, j, 
                      debugPoints);
#pragma omp critical
      std::cout << i << " " << j << " extracted " << numPoints << " points" << std::endl;
#pragma omp critical
      if (!completed [j] && numPoints)
      {
        completed [j] = true;
        std::cout << ++count << "/" << completed.size() << " complete" << std::endl;
      }
      totalPoints += numPoints;
    }
#pragma omp critical
    if (totalPoints)
    {
      size_t id = bundleFile.GetPoints().size();
      std::stringstream ss_bundle;
      ss_bundle << options.output << "/bundle_augmented-" << id << ".out";
      bundleFile.save (ss_bundle.str());
      std::stringstream ss_coords;
      ss_coords << options.output << "/coords_augmented-" << id << ".txt";
      coordsFile.save (ss_coords.str());
      std::stringstream ss_tracks;
      ss_tracks << options.output << "/tracks_augmented-" << id << ".txt";
      tracksFile.save (ss_tracks.str());
      std::stringstream ss;
      ss << "debugPoints" << id << ".ply";
      savePlyFile (ss.str(), debugPoints);
      std::cout << "Saved " << id << "\a" << std::endl;
    }
  }
  TIME_END

  bundleFile.save (options.output + "/bundle_augmented.out");
  coordsFile.save (options.output + "/coords_augmented.txt");
  tracksFile.save (options.output + "/tracks_augmented.txt");

  return 0;
}
