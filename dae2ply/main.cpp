#include <iostream>
#include <assimp/assimp.h>
#include <assimp/aiPostProcess.h>
#include <assimp/aiScene.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include "ply_io.h"
#include <limits>

float
areaOfTriangle (const aiVector3D & v1, const aiVector3D & v2, const aiVector3D & v3)
{
  aiVector3D p1 = v2 - v1;
  aiVector3D p2 = v3 - v1;
  float p1DOTp1 = p1[0]*p1[0] + p1[1]*p1[1] + p1[2]*p1[2];
  float p1DOTp2 = p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2];
  float p2DOTp2 = p2[0]*p2[0] + p2[1]*p2[1] + p2[2]*p2[2];
  return 1.0/2.0*sqrt(p1DOTp1*p2DOTp2 - p1DOTp2*p1DOTp2);
}

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " [input].dae" << std::endl;
  }
  pcl::PointCloud<pcl::PointXYZRGBNormal> samples;
  struct aiLogStream stream = aiGetPredefinedLogStream(aiDefaultLogStream_STDOUT,NULL);
  aiAttachLogStream(&stream);
  const struct aiScene* scene = 0;
  if (!(scene = aiImportFile(argv[1], aiProcessPreset_TargetRealtime_Quality)))
  {
    std::cout << "Could not load file " << argv[1] << std::endl;
    return -1;
  }
  aiMesh ** meshes = scene->mMeshes;
  for (int i = 0; i < scene->mNumMeshes; ++i)
  {
    aiMesh * mesh = meshes[i];
    aiVector3D * vertices = mesh->mVertices;
    aiColor4D ** colors = mesh->mColors;
    aiFace * faces = mesh->mFaces;
    for (int j = 0; j < mesh->mNumFaces; ++j)
    {
      const aiFace & face = faces[j];
      const unsigned int * indices = face.mIndices;
      for (int offset = 0; offset < face.mNumIndices - 2; ++offset)
      {
        const aiVector3D & v1 = vertices[indices[offset + 0]];
        const aiVector3D & v2 = vertices[indices[offset + 1]];
        const aiVector3D & v3 = vertices[indices[offset + 2]];
        aiColor4D c1, c2, c3;
        if (colors[0])
        {
          c1 = colors[0][indices[offset + 0]];
          c2 = colors[0][indices[offset + 1]];
          c3 = colors[0][indices[offset + 2]];
        }
        float area = areaOfTriangle (v1, v2, v3);
        int numSamples = (int)(1 + 1000*area);
        std::cout << numSamples << std::endl;
        for (int k = 0; k < numSamples; ++k)
        {
          float u = (float)rand()/((float)RAND_MAX + 1);
          float v = (float)rand()/((float)RAND_MAX + 1);
          if (u + v > 1)
          {
            u = 1 - u;
            v = 1 - v;
          }
          aiVector3D p1 = v2 - v1;
          aiVector3D p2 = v3 - v1;
          aiVector3D sample = u*p1 + v*p2 + v1;
          aiVector3D normal(p1[1]*p2[2] - p1[2]*p2[1], p1[2]*p2[0] - p1[0]*p2[2], p1[0]*p2[1] - p1[1]*p2[0]);
          normal /= sqrt(normal.SquareLength());
          aiColor4D c12 = c2 - c1;
          aiColor4D c13 = c3 - c1;
          aiColor4D color = u*c12 + v*c13 + c1;
          pcl::PointXYZRGBNormal point;
          point.x = sample[0];
          point.y = sample[1];
          point.z = sample[2];
          RgbConverter c;
          c.r = (uint8_t)(255*color[0]);
          c.g = (uint8_t)(255*color[1]);
          c.b = (uint8_t)(255*color[2]);
          point.rgb = c.rgb;
          point.normal_x = normal[0];
          point.normal_y = normal[1];
          point.normal_z = normal[2];
          samples.push_back (point);
        }
      }
    } 
  }
  
  aiReleaseImport(scene);
  aiDetachAllLogStreams();
  savePlyFile ("samples.ply", samples);
  return 0;
}
