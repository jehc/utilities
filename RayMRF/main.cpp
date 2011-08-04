#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>

#include <Eigen/Dense>

struct RayClique
{
  int position;
  Eigen::Vector3f color;
  std::vector<int> voxels;
};

int
main (int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cout << "Usage: " << argv[0] << " [input] [output]" << std::cout;
    exit (1);
  }

  std::ifstream input (argv[1]);
  if (!input)
  {
    std::cout << "Failed to open " << argv[1] << " for read." << std::endl;
    exit (1);
  }
  
  Eigen::Vector3i dimensions;
  if (!(input >> dimensions [0] >> dimensions[1] >> dimensions [2]))
  {
    std::cout << "Failed to read dimensions" << std::endl;
    exit (1);
  }

  int numRays;
  if (!(input >> numRays))
  {
    std::cout << "Failed to read numRays" << std::endl;
    exit (1);
  }

  std::vector<RayClique> rayCliques (numRays);

  for (size_t i = 0; i < rayCliques.size(); ++i)
  {
    RayClique & ray = rayCliques [i];
    unsigned int r, g, b;
    if (!(input >> r >> g >> b))
    {
      std::cout << "Failed to read rgb for ray " << i << std::endl;
      exit (1);
    }
    ray.color = Eigen::Vector3f(r, g, b);

    if (!(input >> ray.position))
    {
      std::cout << "Failed to read positon for ray " << i << std::endl;
      exit (1);
    }
  
    int numVoxels;
    if (!(input >> numVoxels))
    {
      std::cout << "Failed to read numVoxels for ray " << i << std::endl;
      exit (1);
    }

    ray.voxels.resize (numVoxels);
    for (int j = 0; j < numVoxels; ++j)
    {
      if (!(input >> ray.voxels[j]))
      {
        std::cout << "Failed to read ray " << i << " voxel " << j << std::endl;
        exit (1);
      }
    }
  }

  std::vector<float> x_o (dimensions [0]*dimensions[1]*dimensions[2]);
  std::vector<std::pair<float, Eigen::Vector3f> > x_c_temp (dimensions[0]*dimensions[1]*dimensions[2]);
  std::vector<Eigen::Vector3f> x_c (dimensions[0]*dimensions[1]*dimensions[2]);

  std::vector<std::vector<float> > M_o_i_to_r (rayCliques.size());
  std::vector<std::vector<float> > M_o_r_to_i (rayCliques.size());
  std::vector<std::vector<std::pair<float,Eigen::Vector3f> > > M_c_r_to_i (rayCliques.size());
  std::vector<float> M_uv (x_o.size());
  std::vector<std::vector<int> > neighbors (x_o.size());
  std::vector<std::vector<float> > M_o_p (x_o.size());
  std::vector<std::vector<std::pair<float,Eigen::Vector3f> > > M_c_p (x_o.size());

  for (int i = 0; i < neighbors.size(); ++i)
  {
    int x = i / (dimensions[1]*dimensions[2]);
    int rem = i % (dimensions[1]*dimensions[2]);
    int y = rem / dimensions[2];
    int z = rem % dimensions[2];
    std::vector<int> & candidates = neighbors [i];
    if (x > 0) candidates.push_back (i - dimensions[1]*dimensions[2]);
    if (x < dimensions[0] - 1) candidates.push_back (i + dimensions[1]*dimensions[2]);
    if (y > 0) candidates.push_back (i - dimensions[2]);
    if (y < dimensions[1] - 1) candidates.push_back (i + dimensions[2]);
    if (z > 0) candidates.push_back (i - 1);
    if (z < dimensions[2] - 1) candidates.push_back (i + 1);
    M_o_p[i].resize (candidates.size());
    M_c_p[i].resize (candidates.size());
  }

  for (int r = 0; r < rayCliques.size(); ++r)
  {
    M_o_i_to_r[r].resize (rayCliques[r].voxels.size());
    M_o_r_to_i[r].resize (rayCliques [r].voxels.size());
    M_c_r_to_i[r].resize (rayCliques[r].voxels.size());
  }

  // Ray messages
  for (size_t r = 0; r < rayCliques.size(); ++r)
  {
    const RayClique & ray = rayCliques [r];
    const Eigen::Vector3f & I_r = ray.color;
    size_t n = ray.voxels.size();
    std::vector<float> E_o (n);
    std::vector<float> E_c (n);
    std::vector<float> E_o_up (n);
    std::vector<float> E_star (n);
    std::vector<float> E_star_up (n);
    std::vector<float> E_star_down (n);
    std::vector<float> M_o (n);
    std::vector<float> vis_star (n);
    float E_star_0 = 0;

    // Algorithm 1
    for (size_t i = 0; i < n; ++i)
    {
      M_o [i] = x_o[ray.voxels[i]];//M_o_i_to_r[r][i];
      E_o [i] = std::min (0.0f, M_o [i]);
      E_c [i] = (I_r - x_c [ray.voxels[i]]).squaredNorm();
    }
    E_o_up [n - 1] = 0;
    E_star [n - 1] = E_c [n - 1] + M_o [n - 1];
    for (int i = n - 2; i >= 0; --i)
    {
      E_o_up [i] = E_o_up [i + 1] + E_o[i+1];
      E_star [i] = E_c [i] + M_o [i] + E_o_up [i];
    }
    E_star_up [n - 1] = std::numeric_limits<float>::infinity();
    for (int i = n - 2; i >= 0; --i)
    {
      E_star_up [i] = std::min (E_star_up [i + 1], E_star [i + 1]);
    }
    E_star_down [0] = std::numeric_limits<float>::infinity();
    for (int i = 1; i < n; ++i)
    {
      E_star_down[i] = std::min(E_star_down[i - 1], E_star[i - 1]);
    }
    for (int i = 0; i < n; ++i)
    {
      float M_o_r_to_i_0 = std::min(E_star_down[i] - E_o[i], E_star_up[i]);
      float M_o_r_to_i_1 = std::min(E_star_down[i] - E_o[i], E_star[i] - M_o [i]);
      M_o_r_to_i [r][i] = M_o_r_to_i_1 - M_o_r_to_i_0;
    }

    // Algorithm 2
    E_star_0 = std::numeric_limits<float>::infinity();
    for (int i = 0; i < n; ++i)
    {
      E_star_0 = std::min (E_star_0, E_star [i]);
    }
    for (int i = 0; i < n; ++i)
    {
      vis_star[i] = exp(-(E_star[i] - E_star_0));
      M_c_r_to_i [r][i] = std::make_pair (vis_star[i], vis_star[i]*I_r);
    }
  }

  // Unary occupancy messages
  for (int i = 0; i < x_o.size(); ++i)
  {
    M_uv [i] = pow(x_o[i] - 1.0, 2.0);
  }

  // Pairwise occupancy and color messages
  for (int i = 0; i < x_o.size(); ++i)
  {
    for (int j = 0; j < neighbors [i].size(); ++j)
    {
      float on = (x_o [i] < 0 ? 1.0 : 0.0) + x_o[neighbors[i][j]];
      float off = (x_o [i] >= 0 ? 1.0 : 0.0);
      M_o_p [i][j] = on - off;
      M_c_p [i][j] = std::make_pair((x_c [i] - x_c [neighbors[i][j]]).squaredNorm(), (x_c [i] - x_c [neighbors[i][j]]).squaredNorm()*x_c [i]);
    }
  }

  // Clear variable arrays
  for (int i = 0; i < x_o.size(); ++i)
  {
    x_o[i] = 0;
    x_c_temp[i].first = 0;
    x_c_temp[i].second.setZero();
  }

  // Sum ray messages
  for (int r = 0; r < rayCliques.size(); ++r)
  {
    const RayClique & ray = rayCliques [r];
    const std::vector<int> & voxels = ray.voxels;
    int n = voxels.size();
    for (int i = 0; i < n; ++i)
    {
      int x_i = voxels[i];
      x_o [x_i] += M_o_r_to_i [r][i];
      x_c_temp[x_i].first += M_c_r_to_i[r][i].first;
      x_c_temp[x_i].second += M_c_r_to_i[r][i].second;
    }
  }

  // Sum unary occupancy messages
  for (int i = 0; i < x_o.size(); ++i)
  {
    x_o[i] += M_uv [i];
  }

  // Sum pairwise occupancy and color messages
  for (int i = 0; i < x_o.size(); ++i)
  {
    for (int j = 0; j < neighbors [i].size(); ++j)
    {
      x_o [i] += M_o_p [i][j];
      x_c_temp[i].first += M_c_p [i][j].first;
      x_c_temp[i].second += M_c_p [i][j].second;
    }
  }

  // Normalize colors
  for (int i = 0; i < x_c.size(); ++i)
  {
    x_c [i] = x_c_temp[i].second/x_c_temp[i].first;
  }

  return 0;
}
