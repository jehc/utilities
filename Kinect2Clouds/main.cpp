//#define SMALL_RUN
//#define MANIFEST_ONLY

#include <iostream>
#include <string>
#include <set>
#include <fstream>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <sys/time.h>
#include <getopt.h>

#include <opencv2/opencv.hpp>

#include "ply_io.h"
#include "BundleFile.h"

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
  
std::vector<double> 
dt ( std::vector<double> f, int n) 
{
  std::vector<double> d (n);
  std::vector<int> v (n);
  std::vector<double> z (n+1);
  int k = 0;
  v[0] = 0;
  z[0] = -1e20;
  z[1] = +1e20;
  for (int q = 1; q <= n-1; q++) 
  {
    double s  = ((f[q]+q*q)-(f[v[k]]+v[k]*v[k]))/(2*q-2*v[k]);
    while (s <= z[k]) 
    {
      k--;
      s  = ((f[q]+q*q)-(f[v[k]]+v[k]*v[k]))/(2*q-2*v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k+1] = +1e20;
  }

  k = 0;
  for (int q = 0; q <= n-1; q++) 
  {
    while (z[k+1] < q)
      k++;
    d[q] = (q-v[k])*(q-v[k]) + f[v[k]];
  }
  return d;
}

void
dt (cv::Mat & im) 
{
  int width = im.cols;
  int height = im.rows;
  std::vector<double> f (std::max(width,height));

  // transform along columns
  for (int x = 0; x < width; x++) 
  {
    for (int y = 0; y < height; y++) 
    {
      f[y] = im.at<float> (y, x);
    }
    std::vector<double> d = dt(f, height);
    for (int y = 0; y < height; y++) 
    {
      im.at<float> (y, x) = d[y];
    }
  }

  // transform along rows
  for (int y = 0; y < height; y++) 
  {
    for (int x = 0; x < width; x++) 
    {
      f[x] = im.at<float>(y, x);
    }
    std::vector<double> d = dt(f, width);
    for (int x = 0; x < width; x++) 
    {
      im.at<float>(y, x) = d[x];
    }
  }
}

std::set<std::pair<int, int> >
LoadKeys ( const std::string & keypointFilename )
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
    double x, y;
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

pcl::PointCloud<pcl::PointSurfel>::Ptr LoadCloud (
  const std::string &     colorFilename,
  const std::string &     depthFilename,
  const std::string &     keypointFilename,
  const BundleCamera &    camera,
  double                   scale,
  const std::vector<std::pair<BundleView,BundlePoint> > & bundlerPoints, const cv::Mat & a, const cv::Mat & b, const cv::Mat & c,
  bool highres)
{
#ifndef MANIFEST_ONLY
  pcl::PointCloud<pcl::PointSurfel>::Ptr points ( new pcl::PointCloud<pcl::PointSurfel>() );
  points->is_dense = false;

  cv::Mat colorImageDist = cv::imread ( colorFilename );
  
  std::ifstream depthInput ( depthFilename.c_str () );
  if ( !depthInput )
  {
    std::cout << "Could not load file " << depthFilename << std::endl;
    return points;
  }

  points->width = colorImageDist.cols;
  points->height = colorImageDist.rows;
  points->points.resize (colorImageDist.cols*colorImageDist.rows);

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

  std::set<std::pair<int, int> > keys = LoadKeys ( keypointFilename );

  //Linear interpolation; do not want
  //cv::undistort (depthMapDist, depthMap, cameraMatrix, distCoeffs);

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap ( cameraMatrixCV, distCoeffs, cv::Mat::eye ( 3,
      3,
      CV_32FC1 ), cameraMatrixCV, depthMapDist.size (), CV_32FC1, map1, map2 );
  cv::remap ( depthMapDist, depthMap, map1, map2, cv::INTER_NEAREST );

  const Eigen::Matrix3d & R = camera.GetR ();
  const Eigen::Vector3d & t = camera.GetT ();

  cv::Mat confidenceMapDist = 1e20*cv::Mat::ones(depthMapDist.rows, depthMapDist.cols, CV_32FC1);

  if (!highres)
  {
  for (std::vector<std::pair<BundleView,BundlePoint> >::const_iterator i = bundlerPoints.begin(); i != bundlerPoints.end(); ++i)
  {
    const Eigen::Vector3d p = i->second.GetPosition();
    Eigen::Vector3d center = -R.transpose() * t;
    Eigen::Vector3d b_point = p - center;
    Eigen::Vector3d z (0, 0, -1);
    z = R.transpose() * z;
    double depthA = z.dot(b_point);
    int indexI = (int)(i->first.GetX() + colorImageDist.cols/2);
    int indexJ = (int)(colorImageDist.rows/2 - i->first.GetY());
    double depthB = depthMapDist.at<float> (indexJ, indexI);
    depthB = a.at<float>(indexJ, indexI) * depthB * depthB + b.at<float>(indexJ, indexI) * depthB + c.at<float>(indexJ, indexI);
    confidenceMapDist.at<float>(indexJ, indexI) = std::min((double)confidenceMapDist.at<float> (indexJ, indexI), 100*abs(depthA - depthB));
  }
  dt (confidenceMapDist);
  }
  else
  {
    confidenceMapDist = cv::Mat::zeros(depthMapDist.rows, depthMapDist.cols, CV_32FC1);
  }

  cv::Mat confidenceMap;
  cv::initUndistortRectifyMap (cameraMatrixCV, distCoeffs, cv::Mat::eye(3, 3, CV_32FC1), cameraMatrixCV, confidenceMapDist.size(), CV_32FC1, map1, map2);
  remap (confidenceMapDist, confidenceMap, map1, map2, cv::INTER_NEAREST);

  if (!highres)
  {
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
  }

  cv::Mat blurredDepth (depthMap.rows, depthMap.cols, CV_32FC1);
//  cv::bilateralFilter (depthMap, blurredDepth, 30, 0.15, 15);

  for ( int j = 0; j < depthMap.rows; ++j)
  {
    for ( int i = 0; i < depthMap.cols; ++i )
    {
      double depthRaw = depthMap.at<float> ( j, i );
      double depth = depthRaw;
  //    double depth = a.at<float>(j, i)*pow(depthRaw, 2.0f) + b.at<float>(j, i)*depthRaw + c.at<float>(j, i);
  //    double depth = -0.018 * depthRaw * depthRaw + 1.0038 * depthRaw + 0.005;
      if ( depthRaw <= 0 || isnan(depth) || isinf(depth) || depth <= 0 || depthRaw < 0.2)// || depth > 3)
      {
        pcl::PointSurfel point;
        point.x = std::numeric_limits<double>::quiet_NaN();
        point.y = std::numeric_limits<double>::quiet_NaN();
        point.z = std::numeric_limits<double>::quiet_NaN();
        point.normal_x = std::numeric_limits<double>::quiet_NaN();
        point.normal_y = std::numeric_limits<double>::quiet_NaN();
        point.normal_z = std::numeric_limits<double>::quiet_NaN();
        point.rgba = 0;
        point.curvature = std::numeric_limits<double>::quiet_NaN();
        point.confidence = std::numeric_limits<double>::quiet_NaN();
        point.radius = std::numeric_limits<double>::quiet_NaN();
        (*points) (i, j) = point;
        continue;
      }

      Eigen::Vector3d p ( ( double )i, (double)depthMap.rows - 1 - ( double )j, 1 );
      p = scale * depth * p;

      p = cameraMatrix.inverse () * p;
      p[2] *= -1;

      Eigen::Vector3d diff = -p;
      diff.normalize ();

      pcl::PointSurfel point;
      point.x = p[0];
      point.y = p[1];
      point.z = p[2];

      //Estimate after whole cloud is loaded
      point.normal_x = std::numeric_limits<double>::quiet_NaN();
      point.normal_y = std::numeric_limits<double>::quiet_NaN();
      point.normal_z = std::numeric_limits<double>::quiet_NaN();

      //TODO
      point.curvature = std::numeric_limits<double>::quiet_NaN();
     
      point.radius = scale * depth / camera.GetF();

      double Wp = exp(-confidenceMap.at<float>(j, i)/10000.0);
      int Sb = std::min(i, std::min(j, std::min (depthMap.rows - 1 - j, depthMap.cols - 1 - i)));
      int Smax = 20;
      //double alphab = 1;
      double Wb = (Sb >= Smax) ? 1.0 : (double)Sb/Smax; 
      
      point.confidence = Wb*Wp;

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
        std::cout << "scale: " << scale << std::endl;
      }

      if ( keys.count ( std::make_pair ( j, i ) ) )
      {
        ByteExtractor be;
        be.b[0] = 255;
        be.b[1] = 0;
        be.b[2] = 0;
        be.b[3] = 255;
        point.rgba = be.rgba;
      }
      else
      {
        const cv::Vec3b & color = colorImage.at<cv::Vec3b>( j, i );
        ByteExtractor be;
        be.b[0] = color[2];
        be.b[1] = color[1];
        be.b[2] = color[0];
        be.b[3] = 255;
        point.rgba = be.rgba;
      }
      (*points) (i, j) = point;
    }
  }

  pcl::PointCloud<pcl::PointSurfel>::iterator i;
/*  pcl::PointCloud<pcl::PointXYZ>::Ptr tempPoints (new pcl::PointCloud<pcl::PointXYZ>());
  for (i = points->begin(); i != points->end(); ++i)
  {
    pcl::PointXYZ point;
    point.x = i->x;
    point.y = i->y;
    point.z = i->z;
    tempPoints->push_back (point);
  }
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normalEstimation;
  normalEstimation.setInputCloud (tempPoints);
  pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>());
  tree->setInputCloud (tempPoints);
  normalEstimation.setSearchMethod (tree);
  normalEstimation.setRadiusSearch (0.03);
  normalEstimation.setViewPoint (0, 0, 0);
  pcl::PointCloud<pcl::PointNormal> normals;
  normalEstimation.compute (normals);
  pcl::PointCloud<pcl::PointNormal>::const_iterator j;
  for (i = points->begin(), j = normals.begin(); i != points->end(); ++i, ++j)*/
  for (i = points->begin(); i != points->end(); ++i)
  {
//    Eigen::Vector3d n (j->normal_x, j->normal_y, j->normal_z);
    Eigen::Vector3d v (-i->x, -i->y, -i->z);
    v.normalize();
/*    if (v.dot(n) < 0)
    {
      n *= -1;
    }*/
    //double alphav = 2;
//    double Wv = pow(v.dot (n), alphav);
//    i->confidence *= Wv;
    i->normal_x = v[0];
    i->normal_y = v[1];
    i->normal_z = v[2];
  }
#endif
  Eigen::Vector3d position = -R.transpose() * t;
  points->sensor_origin_ = Eigen::Vector4f (position[0], position [1], position[2], 1.0);
  points->sensor_orientation_ = Eigen::Quaterniond (R.transpose()).cast<float>();

  return points;
}

struct Options
{
  std::string bundleFile;
  std::string listFile;
  std::string output;
  bool        drawKeypoints;
  std::string depthTuning;
  bool highres;

  void
              help () const
  {
    std::cout << "  --bundle [string]" << std::endl;
    std::cout << "  --list [string]" << std::endl;
    std::cout << "  --highres" << std::endl;
    std::cout << "  --output [string]" << std::endl;
    std::cout << "  --keypoints" << std::endl;
    std::cout << "  --depth_tuning [string]" << std::endl;
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
      { "highres", no_argument, 0, 'h'},
      { 0,              0,                         0,                         0                                           } };

    bool bundleFileSet = false;
    bool listFileSet = false;
    bool outputSet = false;
    bool depthTuningSet = false;

    drawKeypoints = false;
    highres = false;

    while ( ( c = getopt_long ( argc, argv, "b:l:o:kd:", longopts, &indexptr ) ) != -1 )
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
        depthTuning = std::string (optarg);
        break;
      case 'h':
        highres = true;
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
      std::cout << "depth tuning option is required" << std::endl;
      help();
    }

    if ( !outputSet )
    {
      std::cout << "output option is required" << std::endl;
      help ();
    }
  }
};

int
main ( int argc, char * * argv )
{
  TIMED

  printCommand ( argc, argv );

  TIME_BEGIN ( "Loading options" )
  Options options ( argc, argv );
  TIME_END

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud ( new pcl::PointCloud<pcl::PointXYZRGB>() );

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

  std::ifstream depthTuningFile (options.depthTuning.c_str());
  if (!depthTuningFile)
  {
    std::cout << "Failed to open depth tuning file" << std::endl;
    options.help();
  }

  TIME_BEGIN ("Loading depth tuning")
  std::vector<double> depthTuning;
  while (depthTuningFile)
  {
    double depth;
    if (!(depthTuningFile >> depth))
    {
      break;
    }
    depthTuning.push_back (depth);
  }
  depthTuningFile.close();

  TIME_BEGIN ("Load bundler points")
  std::vector<std::vector<std::pair<BundleView,BundlePoint> > > bundlerPoints (cameras.size());
  const std::vector<BundlePoint> & points = file.GetPoints();
  for (std::vector<BundlePoint>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    const std::vector<BundleView> views = i->GetViews();
    for (std::vector<BundleView>::const_iterator j = views.begin(); j != views.end(); ++j)
    {
      bundlerPoints [j->GetCamera()].push_back (std::make_pair(*j, *i));
    }
  }
  TIME_END

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

  int numFrames = 0;

  size_t list_index = options.listFile.find_last_of ( "/" );
  TIME_BEGIN ( "Loading all point clouds" )
#pragma omp parallel for schedule(dynamic)
  for ( int i = 0; i < ( int )imageList.size (); ++i )
  {
    const std::string & filename = imageList [i];
#ifdef SMALL_RUN
    if (filename.find ("view0078") == std::string::npos &&
        filename.find ("view0106") == std::string::npos)
    {
      continue;
    }
#endif
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
    if (depthTuning[i] < 1e-9 && !options.highres)
    {
      continue;
    }
    std::string suffix = options.highres ? ".jpg" : "color.jpg";
    size_t replacement = colorFilename.find ( suffix );
    if (options.highres && colorFilename.find("IMG_") == std::string::npos)
    {
      continue;
    }
    if ( replacement == std::string::npos )
    {
      continue;
    }
#pragma omp atomic
    ++numFrames;
#pragma omp critical
    std::cout << "Opening " << colorFilename << std::endl;
    std::string depthFilename ( colorFilename );
    depthFilename.replace ( replacement, suffix.size(), options.highres ? ".raw" : "depth.raw" );
    std::string keyFilename ( colorFilename );
    assert ( replacement + suffix.size() <= keyFilename.size () );
    keyFilename.replace ( replacement, suffix.size(), "color.ks" );
    if ( !options.drawKeypoints )
    {
      keyFilename = "";
    }
    pcl::PointCloud<pcl::PointSurfel>::Ptr points = LoadCloud (
      colorFilename,
      depthFilename,
      keyFilename,
      cameras[i],
      options.highres ? 1.0 : depthTuning[i],
      bundlerPoints[i], a, b, c, options.highres);
    if (points->size() == 0)
    {
      continue;
    }
    std::string cloudFilename = colorFilename;
    size_t dot = cloudFilename.find_last_of (".");
    cloudFilename.erase (dot);
    size_t slash = cloudFilename.find_last_of ("/");
    cloudFilename.erase (0, slash);
    std::stringstream ss_ply;
    ss_ply << options.output << cloudFilename << ".ply";
#pragma omp critical
    std::cout << "Saving " << ss_ply.str() << std::endl;
    savePlyFile (ss_ply.str(), *points);
    std::stringstream ss_pcd;
    ss_pcd << options.output << cloudFilename << ".pcd";
#pragma omp critical
    std::cout << "Saving " << ss_pcd.str() << std::endl;
    pcl::io::savePCDFile (ss_pcd.str(), *points, true);
  }
  TIME_END

  TIME_BEGIN ("Writing MANIFEST and aln project")
  std::string manifestFilename = options.output + "/MANIFEST";
  std::string alnFilename = options.output + "/manifest.aln";
  std::string pcdManifestFilename = options.output + "/pcd.manifest";
  std::ofstream manifest (manifestFilename.c_str());
  std::ofstream aln (alnFilename.c_str());
  std::ofstream pcdManifest (pcdManifestFilename.c_str());

  aln << numFrames << std::endl;

//#pragma omp parallel for
  for ( int i = 0; i < ( int )imageList.size (); ++i )
  {
    const std::string & filename = imageList [i];
    std::string colorFilename = options.listFile;
    if ( list_index == std::string::npos )
    {
      list_index = 0;
    }
    colorFilename.replace ( list_index + 1, colorFilename.size () - list_index, filename );
    std::string cloudFilename = colorFilename;
    size_t dot = cloudFilename.find_last_of (".");
    cloudFilename.erase (dot);
    size_t slash = cloudFilename.find_last_of ("/");
    cloudFilename.erase (0, slash + 1);

    std::stringstream ss_xf;
    ss_xf << options.output << '/' << cloudFilename << ".xf";
    std::ifstream xf (ss_xf.str().c_str());
    if (!xf)
    {
      continue;
    }

    Eigen::Matrix4f transformation;
    for (int j = 0; j < 4; ++j)
    {
      for (int i = 0; i < 4; ++i)
      {
        xf >> transformation (j, i);
      }
    }
    xf.close();
//#pragma omp critical
    {
      aln << cloudFilename << ".ply" << std::endl;
      aln << "#" << std::endl;
      aln << transformation << std::endl;
      manifest << cloudFilename << ".ply" << std::endl;
      pcdManifest << cloudFilename << ".pcd" << std::endl;
    }
  }
  aln << "0" << std::endl;
  aln.close();
  manifest.close();
  pcdManifest.close();
  TIME_END

  return 0;
}
