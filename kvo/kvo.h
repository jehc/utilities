#include <vector>

#include <stdint.h>

#include <Eigen/Dense>

class KVO
{
public:
struct Color
{
  float r, g, b, a;
  Color () : r(0), g(0), b(0), a(0) { }
};
private:
std::vector<std::vector<std::vector<Color> > > data;
float voxel_size;
Eigen::Vector3f translation;
public:
KVO ( int a, int b, int c, float voxel_size, const Eigen::Vector3f & translation )
  : data ( std::vector<std::vector<std::vector<Color> > > ( a, std::vector<std::vector<Color> > ( b,std::vector<Color> ( c ) ) ) )
  , voxel_size ( voxel_size )
  , translation ( translation )
{
}
void save ( const std::string & );
static KVO load ( const std::string & );
std::vector<std::vector<Color> > & operator [] ( int i )
{
  return data [i];
}
float length () const
{
  return voxel_size;
}
size_t size ( int ) const;
};
