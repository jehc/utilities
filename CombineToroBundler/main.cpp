#include "Toro3DFile.h"
#include "BundleFile.h"

#include <iostream>

Eigen::Matrix3d
euler2matrix (const Eigen::Vector3d & euler)
{
  double phi = euler [0];
  double theta = euler [1];
  double psi = euler [2];

  Eigen::Matrix3d R_phi = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_theta = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_psi = Eigen::Matrix3d::Identity();

  R_phi (1,1) = cos(phi);  R_phi(1,2) = -sin(phi);
  R_phi (2,1) = sin(phi);  R_phi(2,2) = cos(phi);

  R_theta(0,0) = cos(theta); R_theta(0,2) = sin(theta);
  R_theta(2,0) = -sin(theta); R_theta(2,2) = cos(theta);

  R_psi(0,0) = cos(psi); R_psi(0,1) = -sin(psi);
  R_psi(1,0) = sin(psi); R_psi(1,1) = cos(psi);

  return R_psi*R_theta*R_phi;
}

int
main (int argc, char ** argv)
{
  if (argc != 4)
  {
    std::cout << "Usage: " << argv [0] << " [toro.graph] [bundler_in.out] [bundler_out.out]" << std::endl;
    exit (1);
  }

  Toro3DFile toroFile (argv [1]);
  BundleFile bundleFile (argv [2]);

  const std::vector<Toro3DVertex> & vertices = toroFile.GetVertices();
  for (size_t i = 0; i < vertices.size(); ++i)
  {
    const Toro3DVertex & vertex = vertices [i];
    if (vertex.id < 0)
    {
      bundleFile.InvalidateCamera (i);
      continue;
    }
    Eigen::Vector3d translation (vertex.x, vertex.y, vertex.z);
    Eigen::Vector3d eulerAngles (vertex.phi, vertex.theta, vertex.psi);
    Eigen::Matrix3d rotation = euler2matrix (eulerAngles);
    Eigen::Matrix3d bundlerRotation = rotation.transpose();
    Eigen::Vector3d bundlerTranslation = -rotation.transpose()*translation;
    bundleFile.SetCamera (bundlerRotation, bundlerTranslation, vertex.id);
  }

  bundleFile.save (argv [3]); 

  return 0;
}
