#include "BundleFile.h"
#include "ply_io.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <opencv2/opencv.hpp>

struct Datapoint
{
  int i;
  int j;
  float measured;
  float correct;
};

void
collectData (const BundleCamera & camera, const std::string depthFile, const Eigen::Vector4f & plane, std::vector<Datapoint> & datapoints, pcl::PointCloud<pcl::PointXYZRGB> & points)
{
  if (!camera.IsValid())
  {
    return;
  }
  std::ifstream input (depthFile.c_str());
  if (!input)
  {
    std::cout << "No depth file found for " << depthFile << std::endl;
    return;
  }
  uint32_t rows, cols;
  if (!input.read ((char*)&rows, sizeof(uint32_t)))
  {
    std::cout << "Failed to read rows" << std::endl;
    exit (1);
  }
  if (!input.read ((char*)&cols, sizeof(uint32_t)))
  {
    std::cout << "Failed to read cols" << std::endl;
    exit (1);
  }
  cv::Mat1f depthMapDist (rows, cols);
  if (!input.read ((char*)depthMapDist.data, depthMapDist.rows*depthMapDist.cols*sizeof(float)))
  {
    std::cout << "Failed to read data" << std::endl;
    exit (1);
  }
  input.close();

  std::string maskFile = depthFile;
  size_t lastslash = maskFile.find_last_of ("/");
  maskFile.replace (lastslash, 1, "/masks/");
  size_t depthraw = maskFile.find ("depth.raw");
  maskFile.replace (depthraw, 9, "color.jpg");

  cv::Mat maskDist = cv::imread (maskFile, 0);
  if (!maskDist.data)
  {
    std::cout << "Failed to open mask " << maskFile << std::endl;
    exit (1);
  }

  assert (!camera.GetK1() && !camera.GetK2());

  cv::Mat cameraMatrixCV ( 3, 3, CV_32FC1 );
  cameraMatrixCV.at<float> ( 0, 0 ) = camera.GetF ();
  cameraMatrixCV.at<float> ( 0, 1 ) = 0;
  cameraMatrixCV.at<float> ( 0, 2 ) = ( float )cols / 2;
  cameraMatrixCV.at<float> ( 1, 0 ) = 0;
  cameraMatrixCV.at<float> ( 1, 1 ) = camera.GetF ();
  cameraMatrixCV.at<float> ( 1, 2 ) = ( float )rows / 2;
  cameraMatrixCV.at<float> ( 2, 0 ) = 0;
  cameraMatrixCV.at<float> ( 2, 1 ) = 0;
  cameraMatrixCV.at<float> ( 2, 2 ) = 1;

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << camera.GetF (), 0, ( float )cols / 2,
  0, camera.GetF (), ( float )rows / 2,
  0, 0, 1;

  cv::Mat distCoeffs ( 5, 1, CV_32FC1 );
  distCoeffs.at<float> ( 0, 0 ) = camera.GetK1 ();
  distCoeffs.at<float> ( 1, 0 ) = camera.GetK2 ();
  distCoeffs.at<float> ( 2, 0 ) = 0;
  distCoeffs.at<float> ( 3, 0 ) = 0;
  distCoeffs.at<float> ( 4, 0 ) = 0;

  cv::Mat depthMap;

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap ( cameraMatrixCV, distCoeffs, cv::Mat::eye ( 3,
      3,
      CV_32FC1 ), cameraMatrixCV, depthMapDist.size (), CV_32FC1, map1, map2 );
  cv::remap ( depthMapDist, depthMap, map1, map2, cv::INTER_NEAREST );

  cv::Mat mask;

  cv::initUndistortRectifyMap ( cameraMatrixCV, distCoeffs, cv::Mat::eye ( 3,
      3,
      CV_32FC1 ), cameraMatrixCV, maskDist.size (), CV_32FC1, map1, map2 );
  cv::remap ( maskDist, mask, map1, map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, 255 );

  Eigen::Vector3f p0 (0, 0, -plane[3]/plane[2]);
  Eigen::Vector3f p1 (0, 1, -(plane[1] + plane[3])/plane[2]);
  Eigen::Vector3f p2 (1, 0, -(plane[0] + plane[3])/plane[2]);
  Eigen::Vector3f Ia = -camera.GetR().transpose() * camera.GetT();

  for (int i = 0; i < depthMap.rows; ++i)
  {
    for (int j = 0; j < depthMap.cols; ++j)
    {
      if ((unsigned int)mask.at<uchar>(i, j) < 127 || depthMap.at<float>(i, j) < 1e-9)
      {
        continue;
      }

      Datapoint datapoint;
      datapoint.i = i;
      datapoint.j = j;
      datapoint.measured = depthMap.at<float>(i, j);

      Eigen::Vector3f Ib ((float)j, (float)depthMap.rows - 1 - (float)i, 1);
      Ib = cameraMatrix.inverse() * Ib;
      Ib[2] *= -1;
      Ib -= camera.GetT();
      Ib = camera.GetR().transpose() * Ib;

      Eigen::Matrix3f A;
      A << Ia[0] - Ib[0], p1[0] - p0[0], p2[0] - p0[0],
           Ia[1] - Ib[1], p1[1] - p0[1], p2[1] - p0[1],
           Ia[2] - Ib[2], p1[2] - p0[2], p2[2] - p0[2];

      Eigen::Vector3f b;
      b[0] = Ia[0] - p0[0];
      b[1] = Ia[1] - p0[1];
      b[2] = Ia[2] - p0[2];

      Eigen::Vector3f x = A.inverse() * b;

      Eigen::Vector3f z (0, 0, -1);
      z = camera.GetR().transpose() * z;

      datapoint.correct = z.dot(x[0]*(Ib - Ia));
      if (isnan(datapoint.correct))
      {
        std::cout << camera.GetT().transpose() << std::endl;
        std::cout << camera.GetR() << std::endl;
        std::cout << i << " " << j << std::endl;
        std::cout << camera.GetF() << std::endl;
        std::cout << camera.GetK1() << std::endl;
        exit (1);
      }
      Eigen::Vector3f intersection = Ia + x[0]*(Ib - Ia);
      pcl::PointXYZRGB point;
      point.x = intersection[0];
      point.y = intersection[1];
      point.z = intersection[2];
      point.rgb = 0;
      points.push_back (point);
      datapoints.push_back (datapoint);
    }
  }
}

int
main (int argc, char ** argv)
{
  if (argc != 5)
  {
    std::cerr << "Usage: " << argv[0] << " [bundle.out] [list.txt] [plane] [output]" << std::endl;
  }

  BundleFile bundle (argv[1]);

  std::vector<std::pair<float, float> > dataPoints;
  std::vector<std::vector<float> > depthCache;
  std::vector<bool> cached;

  std::string junk;
  std::vector<std::string> depthFiles;
  std::ifstream list (argv[2]);
  std::string filesBase (argv[2]);
  size_t slash = filesBase.find_last_of ("/");
  filesBase.resize (slash);
  while (list)
  {
    std::string filename;
    if (!(list >> filename))
    {
      break;
    }
    getline (list, junk);
    size_t replacement = filename.find ("color.jpg");
    if (replacement == std::string::npos)
    {
      depthFiles.push_back ("");
      continue;
    }
    filename.replace (replacement, 9, "depth.raw");
    depthFiles.push_back (filesBase + "/" + filename);
  }
  list.close();

  std::ifstream planeFile (argv[3]);
  Eigen::Vector4f plane;
  if (!(planeFile >> plane[0] >> plane[1] >> plane[2] >> plane[3]))
  {
    std::cout << "Could not open plane file" << std::endl;
    exit (1);
  }
  planeFile.close();

  const std::vector<BundleCamera> & cameras = bundle.GetCameras();

  std::vector<Datapoint> datapoints;

  pcl::PointCloud<pcl::PointXYZRGB> points;
  for (size_t i = 0; i < cameras.size(); ++i)
  {
    std::cout << "Now on camera " << i << std::endl;
    collectData (cameras[i], depthFiles[i], plane, datapoints, points);
  }

  std::ofstream output (argv[4]);
  for (int i = 0; i < datapoints.size(); ++i)
  {
    output << datapoints[i].i << " " << datapoints[i].j << " " << datapoints[i].measured << " " << datapoints[i].correct << std::endl;
  }
  output.close();

  savePlyFile ("debug.ply", points);
}
