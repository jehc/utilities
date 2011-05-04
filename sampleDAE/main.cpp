#include <iostream>
#include <assimp/assimp.h>
#include <assimp/aiPostProcess.h>
#include <assimp/aiScene.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "ply_io.h"
#include <limits>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

struct Triangle
{
  aiVector3D v1, v2, v3, normal;
  Triangle(const aiVector3D & v1, const aiVector3D & v2, 
           const aiVector3D & v3, const aiVector3D & normal)
  : v1(v1), v2(v2), v3(v3), normal(normal)
  {
  }
};

float
areaOfTriangle (const Triangle & t)
{
  aiVector3D p1 = t.v2 - t.v1;
  aiVector3D p2 = t.v3 - t.v1;
  float p1DOTp1 = p1[0]*p1[0] + p1[1]*p1[1] + p1[2]*p1[2];
  float p1DOTp2 = p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2];
  float p2DOTp2 = p2[0]*p2[0] + p2[1]*p2[1] + p2[2]*p2[2];
  return 1.0/2.0*sqrt(p1DOTp1*p2DOTp2 - p1DOTp2*p1DOTp2);
}

bool
intersectRayTriangle (const aiVector3D & source, const aiVector3D & direction, 
                      const Triangle & tri, double & t)
{
  double u, v;

  aiVector3D edge1 = tri.v2 - tri.v1;
  aiVector3D edge2 = tri.v3 - tri.v1;

  aiVector3D tvec;
  
  double det, inv_det;

  aiVector3D pvec (direction[1]*edge2[2] - direction[2]*edge2[1],
                   direction[2]*edge2[0] - direction[0]*edge2[2],
                   direction[0]*edge2[1] - direction[1]*edge2[0]);

  det = edge1[0]*pvec[0] + edge1[1]*pvec[1] + edge1[2]*pvec[2];
  if(det<=1.0e-20 && det >=-1.0e-20) {
      return false;
  }

  inv_det = 1.0 / det;

  tvec = source - tri.v1;

  u = (tvec[0]*pvec[0] + tvec[1]*pvec[1] + tvec[2]*pvec[2]) * inv_det;
  if (u < -0.0000001 || u > 1.0000001)
  {
    return false;
  }

  aiVector3D qvec (tvec[1]*edge1[2] - tvec[2]*edge1[1], 
                   tvec[2]*edge1[0] - tvec[0]*edge1[2], 
                   tvec[0]*edge1[1] - tvec[1]*edge1[0]);


  v = (direction[0]*qvec[0] + direction[1]*qvec[1] + direction[2]*qvec[2]) * inv_det;
  if (v < -0.0000001 || (u + v) > 1.0000001)
  {
    return false;
  }

  t = (edge2[0]*qvec[0] + edge2[1]*qvec[1] + edge2[2]*qvec[2]) * inv_det;

  if (t < 0.0000001 || t!=t || v!=v || u!=u)
  {
    return false;
  }

  return true;

}

int
getIntersectionCount (const aiVector3D & source, const aiVector3D & normal,
                      const std::vector<Triangle> & triangles)
{
  std::vector<Triangle> intersected;
  std::vector<double> distances;
  double distance;
  for (size_t i = 0; i < triangles.size(); ++i)
  {
    if (intersectRayTriangle (source, normal, triangles[i], distance))
    {
      intersected.push_back (triangles[i]);
      distances.push_back (distance);
    }
  }

  int intersectionCount = 0;
  for (size_t i = 0; i < distances.size(); ++i)
  {
    size_t j;
    for (j = 0; j < i; ++j)
    {
      if (fabs(distances[i] - distances[j]) < 1e-3)
      {
        break;
      }
    }
    if (j == i)
    {
      ++intersectionCount;
    }
  }
  
  return intersectionCount;
}

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " [input].dae" << std::endl;
  }
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr samples (new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  struct aiLogStream stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
  aiAttachLogStream(&stream);
  const struct aiScene* scene = 0;
  if (!(scene = aiImportFile(argv[1], 0)))
  {
    std::cout << "Could not load file " << argv[1] << std::endl;
    return -1;
  }
  std::vector<Triangle> triangles;
  aiMesh ** meshes = scene->mMeshes;
  for (int i = 0; i < scene->mNumMeshes; ++i)
  {
    aiMesh * mesh = meshes[i];
    aiVector3D * vertices = mesh->mVertices;
    aiColor4D ** colors = mesh->mColors;
    aiFace * faces = mesh->mFaces;
    aiVector3D * normals = mesh->mNormals;
    for (int j = 0; j < mesh->mNumFaces; ++j)
    {
      const aiFace & face = faces[j];
      const unsigned int * indices = face.mIndices;
      if (face.mNumIndices != 3)
      {
        continue;
      }

      const aiVector3D & v1 = vertices[indices[0]];
      const aiVector3D & v2 = vertices[indices[1]];
      const aiVector3D & v3 = vertices[indices[2]];
      aiVector3D p1 = v2 - v1;
      aiVector3D p2 = v3 - v1;
      aiVector3D normal (p1[1]*p2[2] - p1[2]*p2[1], 
                         p1[2]*p2[0] - p1[0]*p2[2], 
                         p1[0]*p2[1] - p1[1]*p2[0]);
      normal.Normalize();
      //normals[3*j];
      triangles.push_back (Triangle(v1, v2, v3, normal));
    }
  }

  for (size_t i = 0; i < triangles.size(); ++i)
  {
    Triangle & t = triangles[i];
    aiVector3D & normal = t.normal;
    aiVector3D p1 = t.v2 - t.v1;
    aiVector3D p2 = t.v3 - t.v1;

    float area = areaOfTriangle (t);
    int numSamples = (int)(1 + area*1);
    for (int k = 0; k < numSamples; ++k)
    {
      float u = (float)rand()/((float)RAND_MAX + 1);
      float v = (float)rand()/((float)RAND_MAX + 1);
      if (u + v > 1)
      {
        --k;
        continue;
      }
      aiVector3D sample = u*p1 + v*p2 + t.v1;

      double offsetTheta = 0;
      for (int l = 0; l < 40; ++l)
      {
        offsetTheta += 0.5*M_PI*((float)rand()/((float)RAND_MAX + 1));
      }
      offsetTheta /= 40;

#if 0
      std::cerr << "normal " << normal.SquareLength() << std::endl;
#endif

      aiVector3D randVec ((float)(rand())/((float)RAND_MAX + 1), (float)(rand())/((float)RAND_MAX + 1), (float)(rand())/((float)RAND_MAX + 1));
      randVec.Normalize();

#if 0
      std::cerr << "randVec " << randVec.SquareLength() << std::endl;
#endif

      aiVector3D tangent (normal[1]*randVec[2] - normal[2]*randVec[1], normal[2]*randVec[0] - normal[0]*randVec[2], normal[0]*randVec[1] - normal[1]*randVec[0]);
      tangent.Normalize();

#if 0
      std::cerr << "Tangent " << tangent.SquareLength() << std::endl;
#endif

      cv::Matx31f e;
      e(0, 0) = tangent[0];
      e(1, 0) = tangent[1];
      e(2, 0) = tangent[2];
      cv::Matx33f cross = cv::Matx33f::zeros();
      cross(0, 1) = -e(2, 0);
      cross(1, 0) = e(2, 0);
      cross(0, 2) = e(1, 0);
      cross(2, 0) = -e(1, 0);
      cross(1, 2) = -e(0, 0);
      cross(2, 1) = e(0, 0);
      cv::Matx33f A = cos(offsetTheta)*cv::Matx33f::eye() + (1 - cos(offsetTheta))*e*e.t() + sin(offsetTheta)*cross;
      cv::Matx31f x;
      x(0, 0) = normal[0];
      x(1, 0) = normal[1];
      x(2, 0) = normal[2];

      cv::Matx31f newOffset = A*x;

      aiVector3D offset (newOffset(0, 0), newOffset(1, 0), newOffset(2, 0));
offset = normal;
#if 0
      std::cerr << "Offset " << offset.SquareLength() << std::endl;
#endif

      aiVector3D source1 = sample - 1e-4*offset;
      aiVector3D source2 = sample + 1e-4*offset;

      int intersectionCount1 = getIntersectionCount (source1, offset, triangles);
      int intersectionCount2 = getIntersectionCount (source2, -offset, triangles);

      if (intersectionCount1 % 2 == 1 && intersectionCount2 % 2 == 0)
      {
      //NOOP
      }
      else if (intersectionCount2 % 2 == 1 && intersectionCount1 % 1 == 0) 
      {
        normal = -normal;
      }
      else
      {
      //  continue;
      }

      pcl::PointXYZRGBNormal point;
      point.x = sample[0];
      point.y = sample[1];
      point.z = sample[2];
      RgbConverter c;
      c.r = (uint8_t)(127*normal[0] + 128);
      c.g = (uint8_t)(127*normal[1] + 128);
      c.b = (uint8_t)(127*normal[2] + 128);
      point.rgb = c.rgb;
      point.normal_x = normal[0];
      point.normal_y = normal[1];
      point.normal_z = normal[2];
      samples->push_back (point);
    }

    std::cout << (100*(i+1))/triangles.size() << "% complete\r" << std::flush;

  }

  std::cout << std::endl;

  pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> tree;
  tree.setInputCloud (samples);
  std::vector<int> indices (20);
  std::vector<float> dists (20);
  int flipCount = -1;
  while (flipCount)
  {
    flipCount = 0;
  for (pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator i = samples->begin(); i != samples->end(); ++i)
  {
     tree.nearestKSearch (*i, 20, indices, dists);
     pcl::PointNormal normal;
     int oppositeCount = 0;
     for (size_t j = 0; j < indices.size(); ++j)
     {
       const pcl::PointXYZRGBNormal & point = *(samples->begin() + indices[j]);
       if (i->normal_x*point.normal_x + i->normal_y*point.normal_y + i->normal_z*point.normal_z < 0)
       {
         ++oppositeCount;
       }
     }
     if (oppositeCount > indices.size()*0.75)
     {
       ++flipCount;
       std::cerr << "Flipping " << flipCount << "\r" << std::flush;
       i->normal_x *= -1;
       i->normal_y *= -1;
       i->normal_z *= -1;
       RgbConverter c;
       c.r = (uint8_t)(127*i->normal_x + 128);
       c.g = (uint8_t)(127*i->normal_y + 128);
       c.b = (uint8_t)(127*i->normal_z + 128);
       i->rgb = c.rgb;

     }
  }
  std::cerr << std::endl;
  }
  aiReleaseImport(scene);
  aiDetachAllLogStreams();
  std::string filename (argv[1]);
  filename.erase (filename.size() - 3, 3);
  std::stringstream ss_ply, ss_pcd;
  ss_ply << filename << "ply";
  ss_pcd << filename << "pcd";
  std::cout << "Saving to " << ss_ply.str() << " & " << ss_pcd.str() << std::endl;
  savePlyFile (ss_ply.str(), *samples);
  pcl::io::savePCDFile(ss_pcd.str(), *samples);
  return 0;
}
