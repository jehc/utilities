#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "ply_io.h"

void
findAABB (const pcl::PointCloud<pcl::PointXYZRGBNormal> & points, cv::Vec3f & min, cv::Vec3f & max) 
{
  for (int i = 0; i < 3; ++i)
  {
    min[i] = std::numeric_limits<float>::infinity();
    max[i] = -std::numeric_limits<float>::infinity();
  }
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    if (i->x < min[0]) min[0] = i->x;
    if (i->y < min[1]) min[1] = i->y;
    if (i->z < min[2]) min[2] = i->z;
    if (i->x > max[0]) max[0] = i->x;
    if (i->y > max[1]) max[1] = i->y;
    if (i->z > max[2]) max[2] = i->z;
  }
}

void
binNormals (const pcl::PointCloud<pcl::PointXYZRGBNormal> & points, float lengthOfEdge, 
           const cv::Vec3f & min, const cv::Vec3f & max,
           std::vector<std::vector<std::vector<pcl::PointNormal> > > & normalBins)
{
  cv::Vec3f voxels = max - min;
  for (int i = 0; i < 3; ++i)
  {
    voxels[i] /= lengthOfEdge;
  }
  normalBins = std::vector<std::vector<std::vector<pcl::PointNormal> > > 
    ((int)ceil(voxels[0]), std::vector<std::vector<pcl::PointNormal> > ((int)ceil(voxels[1]), 
     std::vector<pcl::PointNormal> ((int)ceil(voxels[2]))));
  std::vector<std::vector<std::vector<int> > > count
    (normalBins.size(), std::vector<std::vector<int> > (normalBins[0].size(), std::vector<int> (normalBins[0][0].size())));
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    int x = (int)((i->x - min[0])/lengthOfEdge);
    int y = (int)((i->y - min[1])/lengthOfEdge);
    int z = (int)((i->z - min[2])/lengthOfEdge);
    normalBins[x][y][z].normal_x += i->normal_x;
    normalBins[x][y][z].normal_y += i->normal_y;
    normalBins[x][y][z].normal_z += i->normal_z;
    ++count[x][y][z];
  }
  for (size_t i = 0; i < normalBins.size(); ++i)
  {
    for (size_t j = 0; j < normalBins[i].size(); ++j)
    {
      for (size_t k = 0; k < normalBins[i][j].size(); ++k)
      {
        normalBins[i][j][k].normal_x /= count[i][j][k];
        normalBins[i][j][k].normal_y /= count[i][j][k];
        normalBins[i][j][k].normal_z /= count[i][j][k];
      }
    }
  }
}

void
getQuadrants (const pcl::PointNormal & normal, std::vector<int> & quadrants)
{
  if (abs(normal.normal_x) < 1e-10 && abs(normal.normal_y) < 1e-10 && abs(normal.normal_z) < 1e-10)
  {
    quadrants.clear();
    return;
  }
  int dominantQuadrant = ((normal.normal_x > 0) << 2)|((normal.normal_y > 0) << 1)|(normal.normal_z > 0);
  int neighbor1 = dominantQuadrant ^ 0x1;
  int neighbor2 = dominantQuadrant ^ 0x2;
  int neighbor3 = dominantQuadrant ^ 0x4;
  quadrants.clear();
  quadrants.push_back (dominantQuadrant);
  quadrants.push_back (neighbor1);
  quadrants.push_back (neighbor2);
  quadrants.push_back (neighbor3); 
}

void
generateLayers (const std::vector<std::vector<std::vector<pcl::PointNormal> > > & normalBins, 
               std::vector<std::vector<cv::Mat> > & layers)
{
  layers = std::vector<std::vector<cv::Mat> > (8, std::vector<cv::Mat> (normalBins[0].size()));
  for (size_t i = 0; i < 8; ++i)
  {
    for (size_t j = 0; j < normalBins[0].size(); ++j)
    {
      layers[i][j] = cv::Mat::ones (normalBins[0][0].size(), normalBins.size(), CV_8UC1);
    }
  }
  std::vector<int> quadrants;
  for (size_t i = 0; i < normalBins.size(); ++i)
  {
    for (size_t j = 0; j < normalBins[i].size(); ++j)
    {
      for (size_t k = 0; k < normalBins[i][j].size(); ++k)
      {
        const pcl::PointNormal & normal = normalBins[i][j][k];
        getQuadrants (normal, quadrants);
        for (size_t q = 0; q < quadrants.size(); ++q)
        {
          layers[quadrants[q]][j].at<uchar>(k, i) = 0;
        }
      }
    }
  }
}

void
depthTransform (const std::vector<std::vector<cv::Mat> > & inputLayers,
                   std::vector<std::vector<cv::Mat> > & outputLayers)
{
  for (size_t i = 0; i < inputLayers.size(); ++i)
  {
    for (size_t j = 0; j < inputLayers[i].size(); ++j)
    {
      inputLayers[i][j].convertTo(outputLayers[i][j], CV_32FC1);
    }
  }
}

void
distanceTransform (const std::vector<std::vector<cv::Mat> > & inputLayers, 
                   std::vector<std::vector<cv::Mat> > & outputLayers)
{
  for (size_t i = 0; i < inputLayers.size(); ++i)
  {
    for (size_t j = 0; j < inputLayers[i].size(); ++j)
    {
      distanceTransform (inputLayers[i][j], outputLayers[i][j], CV_DIST_L2, CV_DIST_MASK_PRECISE);
      for (int y = 0; y < outputLayers[i][j].rows; ++y)
      {
        for (int x = 0; x < outputLayers[i][j].cols; ++x)
        {
          outputLayers[i][j].at<float>(y, x) = exp(-outputLayers[i][j].at<float>(y, x));
        }
      }
    }
  }
}

std::pair<int, int>
findResponse (const std::vector<std::vector<cv::Mat> > & targetLayers, 
              const std::vector<std::vector<cv::Mat> > & sourceLayers)
{
  assert (targetLayers.size() == sourceLayers.size());
  assert (targetLayers[0].size() == sourceLayers[0].size());
  cv::Mat sum (targetLayers[0][0].rows, targetLayers[0][0].cols, CV_32FC1);
  for (size_t i = 0; i < targetLayers.size(); ++i)
  {
    for (size_t j = 0; j < targetLayers[i].size(); ++j)
    {
        cv::Mat convolved (targetLayers[i][j].rows, targetLayers[i][j].cols, CV_32FC1);
    	cv::filter2D (targetLayers[i][j], convolved, -1, sourceLayers[i][j]);
        sum += convolved;
    }
  }
  int x = 0;
  int y = 0;
  float max = 0;
  for (int j = 0; j < sum.rows; ++j)
  {
    for (int i = 0; i < sum.cols; ++i)
    {
      if (sum.at<float>(j, i) > max)
      {
        max = sum.at<float>(j, i);
        x = i;
        y = j;
      }
    }
  }
  return std::make_pair(y, x);
}

int
main (int argc, char ** argv)
{
  if (argc < 5)
  {
    std::cout << "Usage: " << argv[0] << " [target].pcd [source].pcd [num_voxels] [output]" << std::endl;
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZRGBNormal> source, target;
  if (-1 == pcl::io::loadPCDFile (argv[1], target))
  {
    std::cout << "Could not load target PCD " << argv[1] << std::endl;
    return -1;
  }
  if (-1 == pcl::io::loadPCDFile (argv[2], source))
  {
    std::cout << "Could not load source PCD " << argv[2] << std::endl;
    return -1;
  }
  int nVoxels = atoi(argv[3]);
  if(nVoxels == 0)
  {
    std::cout << "num_voxels " << argv[3] << " is invalid" << std::endl;
    return -1;
  }
  cv::Vec3f targetMin, targetMax;
  cv::Vec3f sourceMin, sourceMax;
  findAABB (target, targetMin, targetMax);
  findAABB (source, sourceMin, sourceMax);
  float volume = (targetMax[0] - targetMin[0])*(targetMax[1] - targetMin[1])*(targetMax[2] - targetMin[2]);
  float volumePerVoxel = volume/nVoxels;
  float lengthOfEdge = pow(volumePerVoxel, 0.333f);
  std::vector<std::vector<std::vector<pcl::PointNormal> > > sourceNormalBins, targetNormalBins; 
  binNormals (source, lengthOfEdge, sourceMin, sourceMax, sourceNormalBins);
  binNormals (target, lengthOfEdge, targetMin, targetMax, targetNormalBins);
  std::vector<std::vector<cv::Mat> > sourceLayersBinary, targetLayersBinary;
  std::vector<std::vector<cv::Mat> > sourceLayersFilter, targetLayersFilter;
  generateLayers (sourceNormalBins, sourceLayersBinary);
  generateLayers (targetNormalBins, targetLayersBinary);
  distanceTransform (targetLayersBinary, targetLayersFilter);
  depthTransform (sourceLayersBinary, sourceLayersFilter);
  std::pair<int, int> response = findResponse (targetLayersFilter, sourceLayersFilter);
  cv::Vec3f position = targetMin;
  position[0] += lengthOfEdge * response.first;
  position[2] += lengthOfEdge * response.second;
  std::cout << position[0] << " " << position[1] << " " << position[2] << std::endl;
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = source.begin(); i != source.end(); ++i)
  {
    pcl::PointXYZRGBNormal newPoint = *i;
    newPoint.x += position[0];
    newPoint.y += position[1];
    newPoint.z += position[2];
    target.push_back (newPoint);
  }
  std::stringstream ss;
  ss << argv[3] << ".ply";
  savePlyFile (ss.str(), target);
  return 0;
}
