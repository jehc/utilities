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

#include <fstream>
#include <iostream>
#include <cassert>
#include <string>
#include <exception>
#include <set>

#include "ply.h"

union RgbConverter { float rgb; struct { unsigned char r; unsigned char g; unsigned char b; }; };

std::set<std::pair<int, int> > LoadKeys (const std::string & keypointFilename)
{
  std::ifstream input (keypointFilename.c_str());
  std::set<std::pair<int, int> > keys;
  if (!input)
  {
    std::cerr << "Keypoint file " << keypointFilename << " not found" << std::endl;
    throw std::exception();
  }
  std::string junk;
  getline (input, junk);
  while (input)
  {
    float x, y;
    if (!(input >> x))
    {
      break;
    }
    if (!(input >> y))
    {
      break;
    }
    getline(input, junk);
    keys.insert(std::make_pair((int)y, (int)x));
  }
  input.close();
  return keys;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoadCloud (const std::string & colorFilename, const std::string & depthFilename, const std::string & keypointFilename, bool colorKeys, const BundleCamera & camera, float scale)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>());

  cv::Mat colorImageDist = cv::imread (colorFilename);

  std::ifstream depthInput (depthFilename.c_str());
  if (!depthInput)
  {
    std::cout << "Could not load file " << depthFilename << std::endl;
    throw std::exception();
  }
  uint32_t rows, cols;
  if(!depthInput.read((char*)&rows, sizeof(uint32_t)))
  {
    std::cout << "Could not read rows in file " << depthFilename << std::endl;
    throw std::exception();
  }
  if (!depthInput.read((char*)&cols, sizeof(uint32_t)))
  {
    std::cout << "Could not read cols in file " << depthFilename << std::endl;
    throw std::exception();
  }
  cv::Mat1f depthMapDist (rows, cols);
  if (!depthInput.read((char*)depthMapDist.data, depthMapDist.rows*depthMapDist.cols*sizeof(float)))
  {
    std::cout << "Could not read data in file " << depthFilename << std::endl;
    throw std::exception();
  }
  depthInput.close();
  cv::Mat cameraMatrix (3, 3, CV_32FC1);
  cameraMatrix.at<float> (0, 0) = camera.GetF();
  cameraMatrix.at<float> (0, 1) = 0;
  cameraMatrix.at<float> (0, 2) = (float)colorImageDist.cols/2;
  cameraMatrix.at<float> (1, 0) = 0;
  cameraMatrix.at<float> (1, 1) = camera.GetF();
  cameraMatrix.at<float> (1, 2) = (float)colorImageDist.rows/2;
  cameraMatrix.at<float> (2, 0) = 0;
  cameraMatrix.at<float> (2, 1) = 0;
  cameraMatrix.at<float> (2, 2) = 1;

  cv::Mat distCoeffs (5, 1, CV_32FC1);
  distCoeffs.at<float> (0, 0) = camera.GetK1();
  distCoeffs.at<float> (1, 0) = camera.GetK2();
  std::cerr << camera.GetK1() << " " << camera.GetK2() << std::endl;
  distCoeffs.at<float> (2, 0) = 0;
  distCoeffs.at<float> (3, 0) = 0;
  distCoeffs.at<float> (4, 0) = 0;

  cv::Mat colorImage, depthMap;

  cv::undistort (colorImageDist, colorImage, cameraMatrix, distCoeffs);

  std::set<std::pair<int, int> > keys;
  if (colorKeys)
  {
    keys = LoadKeys (keypointFilename);
  }

  //Linear interpolation; do not want
  //cv::undistort (depthMapDist, depthMap, cameraMatrix, distCoeffs);

  cv::Mat map1, map2;
  cv::initUndistortRectifyMap (cameraMatrix, distCoeffs, cv::Mat::eye (3, 3, CV_32FC1), cameraMatrix, depthMapDist.size(), CV_32FC1, map1, map2);
  cv::remap (depthMapDist, depthMap, map1, map2, cv::INTER_NEAREST);

  for (int j = 0; j < depthMap.rows; ++j)
  {
    for (int i = 0; i < depthMap.cols; ++i)
    {
      float depth = depthMap.at<float> (j, i);
      if (depth == 0)
      {
        continue;
      }

      depth = -0.018*depth*depth + 1.0038*depth + 0.005;
      cv::Mat p (cv::Vec3f((float)i, (float)depthMap.rows - 1.0 - (float)j, 1));
      p = depth*p;

      p = cameraMatrix.inv() * p;
      p.at<float>(2, 0) *= -1;

      const cv::Mat & R = camera.GetR();
      const cv::Mat & t = camera.GetT();
      p = p - t/scale;
      p = R.t() * p;
      pcl::PointXYZRGB point;
      point.x = p.at<float>(0,0);
      point.y = p.at<float>(1,0);
      point.z = p.at<float>(2,0);
      if (colorKeys && keys.count (std::make_pair(j, i)))
      {
        RgbConverter c;
        c.r = 255;
        c.g = 0;
        c.b = 0;
        point.rgb = c.rgb;
      }
      else
      {
        const cv::Vec3b & color = colorImage.at<cv::Vec3b>(j, i);
        RgbConverter c;
        c.r = color[2];
        c.g = color[1];
        c.b = color[0];
        point.rgb = c.rgb;
      }
      points->push_back(point);
    }
  }
  return points;
}

void
savePlyFile (const std::string & filename, const pcl::PointCloud<pcl::PointXYZRGB> & pointCloud)
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

std::vector<std::vector<float> > 
generate_histogram(const pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud)
{
  std::vector<std::vector<float> > histogram (30, std::vector<float> (120));
  for (int i = 0; i < histogram.size(); ++i)
  {
    for (int j = 0; j < histogram[i].size(); ++j)
    {
      histogram[i][j] = 0;
    }
  }
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::const_iterator i = pointCloud.begin(); i != pointCloud.end(); ++i)
  {
    float x = i->normal_x;
    float y = i->normal_y;
    float z = i->normal_z;
    float r = sqrt(x*x + y*y + z*z);
    if (z < 0)
    {
      r *= -1;
    }
    if (r == 0)
    {
      continue;
    }
    x /= r;
    y /= r;
    z /= r;
    float phi_temp = atan2(y, x);
    if (phi_temp < 0) phi_temp += 2*M_PI;
    int phi = (int)(60*phi_temp/M_PI);
    int theta = (int)(60*acos(z)/M_PI);
    if (theta == 30) theta = 29;
    if (phi == 120) phi = 119;
    assert (phi < 120 && theta < 30 && theta >= 0 && phi >= 0);
    histogram[theta][phi]++;
  }
  for (int i = 0; i < histogram.size(); ++i)
  {
    for (int j = 0; j < histogram[i].size(); ++j)
    {
      double theta_0 = M_PI*i/60;
      double phi_0 = M_PI*j/60;
      double delta_phi = M_PI/60;
      double delta_theta = M_PI/60;
      double normalization = delta_phi*(cos(theta_0) - cos(theta_0 + delta_theta));
      histogram[i][j] /= normalization;
    }
  }
  return histogram;
}

void
compute_axis(const std::vector<std::vector<float> > & histogram, cv::Vec3f & v1, cv::Vec3f &v2, cv::Vec3f & v3)
{ 
  float max = 0;
  size_t theta_max = 0, phi_max = 0;
  for (size_t i = 0; i < histogram.size(); ++i)
  {
    for (size_t j = 0; j < histogram[i].size(); ++j)
    {
      if (histogram[i][j] >= max)
      {
        max = histogram[i][j];
        theta_max = i;
        phi_max = j;
      }
    }
  }
  v1[0] = sin(M_PI*theta_max/60)*cos(M_PI*phi_max/60);
  v1[1] = sin(M_PI*theta_max/60)*sin(M_PI*phi_max/60);
  v1[2] = cos(M_PI*theta_max/60);
  max = 0;
  const float slack = 0.06;
  for (size_t i = 0; i < histogram.size(); ++i)
  {
    for (size_t j = 0; j < histogram[i].size(); ++j)
    {
      double xtemp = sin(M_PI*i/60)*cos(M_PI*j/60);
      double ytemp = sin(M_PI*i/60)*sin(M_PI*j/60);
      double ztemp = cos(M_PI*i/60);
      double dot = xtemp*v1[0] + ytemp*v1[1] + ztemp*v1[2];
      if (dot > -slack && dot < slack && histogram[i][j] >= max)
      {
        max = histogram[i][j];
        v2[0] = xtemp;
        v2[1] = ytemp;
        v2[2] = ztemp;
      }
    }
  }
  max = 0;
  for (size_t i = 0; i < histogram.size(); ++i)
  {
    for (size_t j = 0; j < histogram[i].size(); ++j)
    {
      double xtemp = sin(M_PI*i/60)*cos(M_PI*j/60);
      double ytemp = sin(M_PI*i/60)*sin(M_PI*j/60);
      double ztemp = cos(M_PI*i/60);
      double dot1 = xtemp*v1[0] + ytemp*v1[1] + ztemp*v1[2];
      double dot2 = xtemp*v2[0] + ytemp*v2[1] + ztemp*v2[2];
      if (dot1 > -slack && dot1 < slack && dot2 > -slack && dot2 < slack && histogram[i][j] >= max)
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
project_onto_basis(pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud, 
                   const cv::Vec3f & v1, const cv::Vec3f & v2, const cv::Vec3f & v3)
{
  double projections [6] = {v1[1], -v1[1], v2[1], -v2[1], v3[1], -v3[1]};
  double maxValue = 0;
  int maxIndex = 0;
  for (int i = 0; i < 6; ++i)
  {
    if (maxValue <= projections[i])
    {
      maxValue = projections[i];
      maxIndex = i;
    }
  }
  double y [3];
  switch (maxIndex)
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
  double up[3] = {0, 1, 0};
  double axis[3] = {y[2]*up[1] - y[1]*up[2], y[0]*up[2] - y[2]*up[0], y[1]*up[0] - y[0]*up[1]};
  double angle = acos(y[0]*up[0] + y[1]*up[1] + y[2]*up[2]);
  double R [9] = {1,0,0,0,1,0,0,0,1};

  R[0] = cos(angle);
  R[4] = cos(angle);
  R[8] = cos(angle);
  
  double E[9] = {axis[0]*axis[0], axis[0]*axis[1], axis[0]*axis[2],
                 axis[1]*axis[0], axis[1]*axis[1], axis[1]*axis[2],
                 axis[2]*axis[0], axis[2]*axis[1], axis[2]*axis[2]};
  for (int i = 0; i < 9; ++i)
  { 
    R[i] += (1 - cos(angle))*E[i];
  }
  R[1] -= sin(angle)*axis[2];
  R[2] += sin(angle)*axis[1];
  R[3] += sin(angle)*axis[2];
  R[5] -= sin(angle)*axis[0];
  R[6] -= sin(angle)*axis[1];
  R[7] += sin(angle)*axis[0];

  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator i = pointCloud.begin(); i != pointCloud.end(); ++i)
  { 
    double x, y, z, tempx, tempy, tempz;
    tempx = v1[0]*i->normal_x + v1[1]*i->normal_y + v1[2]*i->normal_z;
    tempy = v2[0]*i->normal_x + v2[1]*i->normal_y + v2[2]*i->normal_z;
    tempz = v3[0]*i->normal_x + v3[1]*i->normal_y + v3[2]*i->normal_z;
    x = R[0]*tempx + R[3]*tempy + R[6]*tempz;
    y = R[1]*tempx + R[4]*tempy + R[7]*tempz;
    z = R[2]*tempx + R[5]*tempy + R[8]*tempz;
    i->normal_x = x;
    i->normal_y = y;
    i->normal_z = z;
    tempx = v1[0]*i->x + v1[1]*i->y + v1[2]*i->z;
    tempy = v2[0]*i->x + v2[1]*i->y + v2[2]*i->z;
    tempz = v3[0]*i->x + v3[1]*i->y + v3[2]*i->z;
    x = R[0]*tempx + R[3]*tempy + R[6]*tempz;
    y = R[1]*tempx + R[4]*tempy + R[7]*tempz;
    z = R[2]*tempx + R[5]*tempy + R[8]*tempz;
    i->x = x;
    i->y = y;
    i->z = z;

  }
}

void
print_basis(const cv::Vec3f & v1, const cv::Vec3f & v2, const cv::Vec3f & v3)
{
  fprintf (stderr, "%f %f %f\n", v1[0], v1[1], v1[2]);
  fprintf (stderr, "%f %f %f\n", v2[0], v2[1], v2[2]);
  fprintf (stderr, "%f %f %f\n", v3[0], v3[1], v3[2]);
  double angle1 = 180*acos(v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2])/M_PI;
  double angle2 = 180*acos(v2[0]*v3[0] + v2[1]*v3[1] + v2[2]*v3[2])/M_PI;
  double angle3 = 180*acos(v3[0]*v1[0] + v3[1]*v1[1] + v3[2]*v1[2])/M_PI;
  fprintf (stderr, "%lf %lf %lf\n", angle1, angle2, angle3);
}

void
reorient (pcl::PointCloud<pcl::PointXYZRGBNormal> & pointCloud)
{
  std::vector<std::vector<float> > histogram = generate_histogram (pointCloud);
  cv::Vec3f v1, v2, v3;
  compute_axis (histogram, v1, v2, v3);
  project_onto_basis (pointCloud, v1, v2, v3);
  print_basis (v1, v2, v3);
}

int
main (int argc, char ** argv)
{
  if (argc < 4)
  {
    std::cerr << "Usage: " << argv[0] << " [bundle.out] [list.txt] [output] {draw_keypoints} {depth_tuning}" << std::endl;
    return -1;
  }

  bool draw_keypoints = false;
  if (argc == 5)
  {
    draw_keypoints = atoi (argv[4]);
  }

  float scale = 1;
  if (argc == 6)
  {
    scale = atof (argv[5]);
  }

  BundleFile file (argv[1]);
  const std::vector<BundleCamera> cameras = file.GetCameras();
  std::ifstream images (argv[2]);

//  pcl::PointCloud<pcl::PointXYZRGBNormal> combinedCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combinedCloud (new pcl::PointCloud<pcl::PointXYZRGB>());

  int index = 0;
  int count = 0;
  const int maxCount = 5;
  while (images)// && count < maxCount)
  {
    std::string filename, line;
    if (!(images >> filename))
    {
      break;
    }
    getline (images, line);
    if (!cameras[index].IsValid())
    {
      std::cerr << "Camera " << index << " is invalid." << std::endl;
      ++index;
      continue;
    }
    std::cerr << filename << std::endl;
    size_t replacement = filename.find ("color.jpg");
    if (replacement == std::string::npos)
    {
      ++index;
      continue;
    }
    std::string filename2 (filename);
    filename2.replace (replacement, 15, "depth.raw");
    std::string keyFilename (filename);
    keyFilename.replace (replacement, 15, "color.ks");
    std::cerr << filename2 << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points = LoadCloud (filename, filename2, keyFilename, draw_keypoints, cameras[index], scale);
    *combinedCloud += *points;
    ++count;
    ++index;
  }
#if 1
  pcl::PointCloud<pcl::PointXYZRGBNormal> final;
  const int desired = 10000000;
  const float probability = (float)desired/(float)combinedCloud->size();
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator i = combinedCloud->begin(); i != combinedCloud->end(); ++i)
  {
    if ((float)rand()/((float)RAND_MAX + 1) < probability)
    {
      pcl::PointXYZRGBNormal newPoint;
      newPoint.x = i->x;
      newPoint.y = i->y;
      newPoint.z = i->z;
      newPoint.rgb = i->rgb;
      final.push_back (newPoint);
    }
  }
#else
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr reduced (new pcl::PointCloud<pcl::PointXYZRGB>());
  std::cerr << combinedCloud->size() << std::endl;
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlierRemoval;
  outlierRemoval.setInputCloud (combinedCloud);
  outlierRemoval.setNegative (false);
  outlierRemoval.setStddevMulThresh(2);
  outlierRemoval.filter (*reduced);
  std::cerr << reduced->size() << std::endl;
  pcl::VoxelGrid<pcl::PointXYZRGB> downsampler;
  downsampler.setInputCloud (reduced);
  downsampler.setLeafSize (0.01, 0.01, 0.01);
  downsampler.filter (*reduced);
  std::cerr << reduced->size() << std::endl;
  pcl::KdTree<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  tree->setInputCloud (reduced);
  pcl::PointCloud <pcl::PointNormal>::Ptr normals (new pcl::PointCloud<pcl::PointNormal>());
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> normalEstimation;
  normalEstimation.setInputCloud (reduced);
  normalEstimation.setOutputNormals (normals);
  normalEstimation.setSearchRadius (0.04);
  normalEstimation.setSearchMethod (tree);
  pcl::PointCloud<pcl::PointXYZRGB> cleaned;
  normalEstimation.reconstruct (cleaned);
  pcl::PointCloud<pcl::PointXYZRGB>::const_iterator i;
  pcl::PointCloud<pcl::PointXYZRGB>::const_iterator k;
  pcl::PointCloud<pcl::PointNormal>::const_iterator j;
  pcl::PointCloud<pcl::PointXYZRGBNormal> final;
  for (i = cleaned.begin(), j = normals->begin(), k = reduced->begin(); 
       i != cleaned.end(); ++i, ++j, ++k)
  {
    pcl::PointXYZRGBNormal newPoint;
    newPoint.normal_x = j->normal_x;
    newPoint.normal_y = j->normal_y;
    newPoint.normal_z = j->normal_z;
    newPoint.rgb = k->rgb;
    newPoint.x = i->x;
    newPoint.y = i->y;
    newPoint.z = i->z;
    final.push_back (newPoint);
  }
  reorient (final);
#endif
  std::stringstream ss_ply, ss_pcd;
  ss_ply << argv[3] << ".ply";
  ss_pcd << argv[3] << ".pcd";
  savePlyFile (ss_ply.str(), final);
  pcl::io::savePCDFile (ss_pcd.str(), final);
  images.close();
 
  return 0;
}
