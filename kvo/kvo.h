#include <vector>

#include <Eigen/Dense>

class KVO
{
std::vector<std::vector<std::vector<std::pair<uint64_t, uint64_t> > > > data;
float voxel_size;
Eigen::Vector3f translation;
public:
KVO ( int a, int b, int c, float voxel_size, const Eigen::Vector3f & translation )
  : data ( std::vector<std::vector<std::vector<std::pair<uint64_t,
                                                         uint64_t> > > > ( a,
                                                                           std::vector<std::vector<std::pair<
                                                                                                     uint64_t,
                                                                                                     uint64_t> > > (
                                                                             b,
                                                                             std::vector<std::pair<uint64_t, uint64_t> > ( c ) ) ) )
  , voxel_size ( voxel_size )
  , translation ( translation )
{
}
void save ( const std::string & );
static KVO load ( const std::string & );
std::vector<std::vector<std::pair<uint64_t, uint64_t> > > & operator [] ( int i )
{
  return data [i];
}
size_t size ( int );
};
