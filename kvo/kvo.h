#include <vector>

#include <stdint.h>

#include <Eigen/Dense>

class KVO
{
std::vector<std::vector<std::vector<float> > > data;
float voxel_size;
Eigen::Vector3f translation;
public:
KVO ( int a, int b, int c, float voxel_size, const Eigen::Vector3f & translation )
  : data ( std::vector<std::vector<std::vector<float> > > ( a, std::vector<std::vector<float> > ( b,std::vector<float > ( c ) ) ) )
  , voxel_size ( voxel_size )
  , translation ( translation )
{
}
void save ( const std::string & );
static KVO load ( const std::string & );
std::vector<std::vector<float> > & operator [] ( int i )
{
  return data [i];
}
float length () const
{
  return voxel_size;
}
size_t size ( int ) const;
};
