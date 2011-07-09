#include "ply_io.h"
#include <Eigen/Dense>

int
main (int argc, char ** argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: " << argv[0] << " [in.ply]" << std::endl;
    return 1;
  }
  pcl::PointCloud<pcl::PointXYZ> points;
  loadPlyFile (argv[1], points);
  Eigen::Vector3f mean;
  mean.setZero();
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    Eigen::Vector3f point (i->x, i->y, i->z);
    mean += point;
  }
  mean /= points.size();
  Eigen::Matrix3f covariance;
  covariance.setZero();
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    Eigen::Vector3f point (i->x, i->y, i->z);
    point -= mean;
    covariance += point * point.transpose();
  }
  covariance /= points.size() -1;

  Eigen::JacobiSVD<Eigen::Matrix3f> svd;
  svd.compute(covariance, Eigen::ComputeFullU|Eigen::ComputeFullV);
  std::cerr << svd.matrixU() << std::endl << svd.matrixV() << std::endl << svd.singularValues() << std::endl;

  Eigen::Matrix3f U = svd.matrixU();

  Eigen::Vector3f n (U(0, 2), U(1, 2), U(2, 2));

  float a = n[0];
  float b = n[1];
  float c = n[2];
  float d = -a*mean[0] - b*mean[1] - c*mean[2];

  std::cout << a << " " << b << " " << c << " " << d << std::endl;
/*
  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator i = points.begin(); i != points.end(); ++i)
  {
    Eigen::Vector3f point (i->x, i->y, i->z);
    float D = (a*point[0] + b*point[1] + c*point[2] + d)/sqrt(a*a + b*b + c*c);
    std::cout << D << std::endl;
  }
*/
  return 0;
}
