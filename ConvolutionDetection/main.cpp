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
    voxels[i] += 1;
  }
  normalBins = std::vector<std::vector<std::vector<pcl::PointNormal> > > 
    ((int)ceil(voxels[0]), std::vector<std::vector<pcl::PointNormal> > ((int)ceil(voxels[1]), 
     std::vector<pcl::PointNormal> ((int)ceil(voxels[2]))));
  for (int i = 0; i < normalBins.size(); ++i)
  {
    for (int j = 0; j < normalBins[i].size(); ++j)
    {
      for (int k = 0; k < normalBins[i][j].size(); ++k)
      {
        normalBins[i][j][k].normal_x = 0;
        normalBins[i][j][k].normal_y = 0;
        normalBins[i][j][k].normal_z = 0;
      }
    }
  }
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    int x = (int)((i->x - min[0])/lengthOfEdge);
    int y = (int)((i->y - min[1])/lengthOfEdge);
    int z = (int)((i->z - min[2])/lengthOfEdge);
//    if (count[x][y][z]) continue;
    assert (i->x >= min[0]);
    assert (i->y >= min[1]);
    assert (i->z >= min[2]);
    assert (x < normalBins.size() && x >= 0);
    assert (y < normalBins[x].size() && y >= 0);
    assert (z < normalBins[x][y].size() && z >= 0);
    normalBins[x][y][z].normal_x += i->normal_x;
    normalBins[x][y][z].normal_y += i->normal_y;
    normalBins[x][y][z].normal_z += i->normal_z;
  } 
  for (size_t i = 0; i < normalBins.size(); ++i)
  {
    for (size_t j = 0; j < normalBins[i].size(); ++j)
    {
      for (size_t k = 0; k < normalBins[i][j].size(); ++k)
      {
        double norm = sqrt(pow(normalBins[i][j][k].normal_x, 2.0f) + pow(normalBins[i][j][k].normal_y, 2.0f) + pow(normalBins[i][j][k].normal_z, 2.0f));
        normalBins[i][j][k].normal_x /= norm;
        normalBins[i][j][k].normal_y /= norm;
        normalBins[i][j][k].normal_z /= norm;
      }
    }
  }
}

void
getRegions (const pcl::PointNormal & normal, std::vector<int> & regions)
{
  const double threshold = 0.9;
  assert (threshold > 0.71 && threshold < 1.0); 
  regions.clear();

#if 0
  std::cerr << normal.normal_x << " " << normal.normal_y << " " << normal.normal_z << std::endl;
#endif

  if (normal.normal_x != normal.normal_x || normal.normal_y != normal.normal_y || normal.normal_z != normal.normal_z)
  {
    return;
  }
  if (normal.normal_x > threshold)
  {
    regions.push_back (0);
  }
  else if (normal.normal_x < -threshold)
  {
    regions.push_back (1);
  }
  else if (normal.normal_y > threshold)
  {
    regions.push_back (2);
  }
  else if (normal.normal_y < -threshold)
  {
    regions.push_back (3);
  }
  else if (normal.normal_z > threshold)
  {
    regions.push_back (4);
  }
  else if (normal.normal_z < -threshold)
  {
    regions.push_back (5);
  }
  else
  {
    int dominantRegion = ((((normal.normal_z > 0) ? 1 : 0) << 2)|(((normal.normal_y > 0) ? 1 : 0) << 1)|((normal.normal_x > 0) ? 1 : 0)) + 6;
#if 0
    assert (((((normal.normal_z > 0) ? 1 : 0) << 2)|(((normal.normal_y > 0) ? 1 : 0) << 1)|((normal.normal_x > 0) ? 1 : 0)) + 6 == dominantRegion);
    std::cerr << ((((normal.normal_z > 0) ? 1 : 0) << 2)|(((normal.normal_y > 0) ? 1 : 0) << 1)|((normal.normal_x > 0) ? 1 : 0)) + 6 << " " << dominantRegion << std::endl;
    std::cerr << dominantRegion - 6 << " " << (((normal.normal_z > 0) ? 1 : 0) << 2) << " " << normal.normal_z << " " << (((normal.normal_y > 0) ? 1 : 0) << 1) << " " << normal.normal_y << " " << ((normal.normal_x > 0) ? 1 : 0) << " " << normal.normal_x << std::endl;
#endif
    regions.push_back (dominantRegion);
  }
}

pcl::PointNormal
region2Normal (int region)
{
  pcl::PointNormal normal;
  normal.normal_x = 0;
  normal.normal_y = 0; 
  normal.normal_z = 0;
  switch (region)
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
      assert (0);
      break;
  }
  return normal;
}

void
generateLayers (const std::vector<std::vector<std::vector<pcl::PointNormal> > > & normalBins, 
               std::vector<std::vector<cv::Mat> > & layers)
{
  layers = std::vector<std::vector<cv::Mat> > (14, std::vector<cv::Mat> (normalBins[0].size()));
  for (size_t i = 0; i < layers.size(); ++i)
  {
    for (size_t j = 0; j < layers[i].size(); ++j)
    {
      layers[i][j] = cv::Mat::zeros (normalBins.size(), normalBins[0][0].size(), CV_8UC1);
    }
  }
  std::vector<int> regions;
  for (size_t i = 0; i < normalBins.size(); ++i)
  {
    for (size_t j = 0; j < normalBins[i].size(); ++j)
    {
      for (size_t k = 0; k < normalBins[i][j].size(); ++k)
      {
        const pcl::PointNormal & normal = normalBins[i][j][k];
        getRegions (normal, regions);
        for (size_t q = 0; q < regions.size(); ++q)
        {
          layers[regions[q]][j].at<uchar>(i, k) = 1;
        }
      }
    }
  }
}

void
depthTransform (const std::vector<std::vector<cv::Mat> > & inputLayers,
                   std::vector<std::vector<cv::Mat> > & outputLayers)
{
  outputLayers = std::vector<std::vector<cv::Mat> > (inputLayers.size(), std::vector<cv::Mat> (inputLayers[0].size()));
  for (size_t i = 0; i < inputLayers.size(); ++i)
  {
    for (size_t j = 0; j < inputLayers[i].size(); ++j)
    {
      outputLayers[i][j] = cv::Mat (inputLayers[i][j].rows, inputLayers[i][j].cols, CV_32FC1);
      for (int y = 0; y < inputLayers[i][j].rows; ++y)
      {
        for (int x = 0; x < inputLayers[i][j].cols; ++x)
        {
          outputLayers[i][j].at<float>(y, x) = 1 - inputLayers[i][j].at<uchar>(y, x);
        }
      }
    }
  }
}

void
distanceTransform (const std::vector<std::vector<cv::Mat> > & inputLayers, 
                   std::vector<std::vector<cv::Mat> > & outputLayers)
{
  outputLayers = std::vector<std::vector<cv::Mat> > (inputLayers.size(), std::vector<cv::Mat> (inputLayers[0].size()));
  for (size_t i = 0; i < inputLayers.size(); ++i)
  {
    for (size_t j = 0; j < inputLayers[i].size(); ++j)
    {
      outputLayers[i][j] = cv::Mat (inputLayers[i][j].rows, inputLayers[i][j].cols, CV_32FC1);
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
  cv::Mat sum = cv::Mat::zeros(targetLayers[0][0].rows, targetLayers[0][0].cols, CV_32FC1);
  for (size_t i = 0; i < targetLayers.size(); ++i)
  {
    for (size_t j = 0; j < targetLayers[i].size() && j < sourceLayers[i].size(); ++j)
    {
        cv::Mat convolved (targetLayers[i][j].rows, targetLayers[i][j].cols, CV_32FC1);
    	cv::filter2D (targetLayers[i][j], convolved, -1, sourceLayers[i][j], cv::Point(0, 0));
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

void
moveSource (pcl::PointCloud<pcl::PointXYZRGBNormal> & points)
{
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator i = points.begin(); i != points.end(); ++i)
  {
    i->x += 20 + 2*(float)rand()/((float)RAND_MAX + 1);
    i->y += 40 + 2*(float)rand()/((float)RAND_MAX + 1);
    i->z += -10 + 2*(float)rand()/((float)RAND_MAX + 1);
  }
}

void
saveBinnedNormals (const std::string & filename, const std::vector<std::vector<std::vector<pcl::PointNormal> > > & normalBins)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  for (int i = 0; i < normalBins.size(); ++i)
  {
    for (int j = 0; j < normalBins[i].size(); ++j)
    {
      for (int k = 0; k < normalBins[i][j].size(); ++k)
      {
        pcl::PointXYZRGBNormal point;
        point.x = i;
        point.y = j;
        point.z = k;
        RgbConverter c;
        c.r = (uchar)(127*normalBins[i][j][k].normal_x + 128);
        c.g = (uchar)(127*normalBins[i][j][k].normal_y + 128);
        c.b = (uchar)(127*normalBins[i][j][k].normal_z + 128);
        point.rgb = c.rgb;
        point.normal_x = normalBins[i][j][k].normal_x;
        point.normal_y = normalBins[i][j][k].normal_y;
        point.normal_z = normalBins[i][j][k].normal_z;
        if (point.normal_x == point.normal_x && point.normal_y == point.normal_y && point.normal_z == point.normal_z)
        {
          cloud.push_back (point);
        }
      }
    }
  }
  savePlyFile (filename, cloud);
}

void saveLayers (const std::string & filename, const std::vector<std::vector<cv::Mat> > & layers)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  for (int i = 0; i < layers.size(); ++i)
  {
    for (int j = 0; j < layers[i].size(); ++j)
    {
      for (int k = 0; k < layers[i][j].rows; ++k)
      {
        for (int l = 0; l < layers[i][j].cols; ++l)
        {
          if (layers[i][j].at<uchar>(k, l))
          {
            pcl::PointXYZRGBNormal point;
            point.x = k;
            point.y = j;
            point.z = l;
            RgbConverter c;
            pcl::PointNormal normal = region2Normal (i);
            point.normal_x = normal.normal_x;
            point.normal_y = normal.normal_y;
            point.normal_z = normal.normal_z;
            c.r = (uchar)(127*point.normal_x + 128);
            c.g = (uchar)(127*point.normal_y + 128);
            c.b = (uchar)(127*point.normal_z + 128);
            point.rgb = c.rgb;
            cloud.push_back (point);
          }
        }
      }
    }
  }
  savePlyFile (filename, cloud);
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

#if 0
  moveSource (target);
#endif

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

#if 1
  saveBinnedNormals ("source.ply", sourceNormalBins);
  saveBinnedNormals ("target.ply", targetNormalBins);
#endif

  std::vector<std::vector<cv::Mat> > sourceLayersBinary, targetLayersBinary;
  std::vector<std::vector<cv::Mat> > sourceLayersFilter, targetLayersFilter;
  generateLayers (sourceNormalBins, sourceLayersBinary);
  generateLayers (targetNormalBins, targetLayersBinary);

#if 1
  saveLayers ("sourceLayers.ply", sourceLayersBinary);
  saveLayers ("targetLayers.ply", targetLayersBinary);
#endif

  distanceTransform (targetLayersBinary, targetLayersFilter);
  depthTransform (sourceLayersBinary, sourceLayersFilter);
  std::pair<int, int> response = findResponse (targetLayersFilter, sourceLayersFilter);
  cv::Vec3f position = targetMin;
  position[0] += lengthOfEdge * response.second;
  position[2] += lengthOfEdge * response.first;
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
  ss << argv[4] << ".ply";
  savePlyFile (ss.str(), target);
  return 0;
}
