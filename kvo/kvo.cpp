#include <kvo.h>
#include <cassert>
#include <fstream>
#include <iostream>

void
KVO::save ( const std::string & filename )
{
  std::ofstream output ( filename.c_str () );

  if ( !output )
  {
    std::cout << "Failed to open " << filename << " for write" << std::endl;
    exit ( 1 );
  }

  output << "version 1" << std::endl;

  output << "control ";
  float controlf = 12345.67890;
  uint64_t controli = 0x0123456789ABCDEF;
  output.write ( ( char * )&controlf, sizeof ( float ) );
  output.write ( ( char * )&controli, sizeof ( uint64_t ) );
  output << std::endl;

  output << "voxel_size " << voxel_size << std::endl;

  output << "dims " << size ( 0 ) << " " << size ( 1 ) << " " << size ( 2 ) << std::endl;

  output << "translation " << translation [0] << " " << translation [1] << " " << translation [2] <<
  std::endl;

  output << "data " << sizeof ( uint64_t ) * 2 * size ( 0 ) * size ( 1 ) * size ( 2 ) << " ";

  for ( size_t i = 0; i < data.size (); ++i )
  {
    for ( size_t j = 0; j < data[i].size (); ++j )
    {
      for ( size_t k = 0; k < data[i][j].size (); ++k )
      {
        output.write ( ( char * )&data[i][j][k].first, sizeof ( uint64_t ) );
        output.write ( ( char * )&data[i][j][k].second, sizeof ( uint64_t ) );
      }
    }
  }
  output.close ();
}

KVO
KVO::load ( const std::string & filename )
{
  union flipperf {float f; uint8_t b[4];} fromf, tof;
  union flipperi {uint64_t i; uint8_t b[8];} fromi, toi;
  std::ifstream input ( filename.c_str () );

  if ( !input )
  {
    std::cout << "Failed to open " << filename << " for read" << std::endl;
    exit ( 1 );
  }

  std::string field;

  if ( !( input >> field ) )
  {
    std::cout << "Failed to read version field" << std::endl;
    exit ( 1 );
  }
  if ( field != "version" )
  {
    std::cout << "Expected version field but found " << field << std::endl;
    exit ( 1 );
  }
  int version;
  if ( !( input >> version ) )
  {
    std::cout << "Failed to read version number" << std::endl;
    exit ( 1 );
  }
  if ( version != 1 )
  {
    std::cout << "Only kvo version 1 is supported" << std::endl;
  }

  if ( !( input >> field ) )
  {
    std::cout << "Failed to read control field" << std::endl;
    exit ( 1 );
  }
  if ( field != "control" )
  {
    std::cout << "Expected control field but found " << field << std::endl;
    exit ( 1 );
  }
  int space = input.get ();
  if ( space != ' ' )
  {
    std::cout << "Expected space delimiter" << std::endl;
    exit ( 1 );
  }
  float controlf;
  uint64_t controli;
  input.read ( ( char * )&controlf, sizeof ( float ) );
  input.read ( ( char * )&controli, sizeof ( uint64_t ) );
  flipperf expected;
  expected.f = 12345.67890;
  if ( controlf != expected.f )
  {
    std::cout << "Floating point control value expected 12345.67890 but found " << controlf << std::endl;
    fromf.f = controlf;
    for (int i = 0; i < 4; ++i)
    {
      tof.b[i] = fromf.b[3 - i];
      std::cout << "Expected byte: " << (unsigned int)expected.b[i] << " Actual byte: " << (unsigned int)fromf.b[i] << std::endl;
    }
    std::cout << "Flipping gets " << tof.f << std::endl;
    exit ( 1 );
  }
  if ( controli != 0x0123456789ABCDEF )
  {
    std::cout << "Integer control value expected 0x0123456789ABCDEF but found " << controli << std::endl;
    exit ( 1 );
  }

  if ( !( input >> field ) )
  {
    std::cout << "Failed to read voxel_size field" << std::endl;
    exit ( 1 );
  }
  if ( field != "voxel_size" )
  {
    std::cout << "Expected voxel_size field but found " << field << std::endl;
    exit ( 1 );
  }
  float voxel_size;
  if ( !( input >> voxel_size ) )
  {
    std::cout << "Failed to read voxel_size" << std::endl;
    exit ( 1 );
  }
  if ( voxel_size <= 0 || isnan ( voxel_size ) || isinf ( voxel_size ) )
  {
    std::cout << "voxel_size must be positive and finite but is " << voxel_size << std::endl;
  }

  if ( !( input >> field ) )
  {
    std::cout << "Failed to read dims field" << std::endl;
    exit ( 1 );
  }
  if ( field != "dims" )
  {
    std::cout << "Expected dims field but found " << field << std::endl;
    exit ( 1 );
  }
  int a, b, c;
  if ( !( input >> a >> b >> c ) )
  {
    std::cout << "Failed to read dims" << std::endl;
    exit ( 1 );
  }
  if ( a <= 0 || b <= 0 || c <= 0 )
  {
    std::cout << "dims must be positive but are " << a << " " << b << " " << c << std::endl;
    exit ( 1 );
  }

  if ( !( input >> field ) )
  {
    std::cout << "Failed to read translation field" << std::endl;
    exit ( 1 );
  }
  if ( field != "translation" )
  {
    std::cout << "Expected translation field but found " << field << std::endl;
    exit ( 1 );
  }
  Eigen::Vector3f translation;
  if ( !( input >> translation[0] >> translation[1] >> translation[2] ) )
  {
    std::cout << "Failed to read translation" << std::endl;
    exit ( 1 );
  }

  KVO voxels ( a, b, c, voxel_size, translation );

  if ( !( input >> field ) )
  {
    std::cout << "Failed to read data field" << std::endl;
    exit ( 1 );
  }
  if ( field != "data" )
  {
    std::cout << "Expected data but found " << field << std::endl;
    exit ( 1 );
  }
  size_t bytes;
  if ( !( input >> bytes ) )
  {
    std::cout << "Failed to read bytes" << std::endl;
    exit ( 1 );
  }
  if ( bytes != sizeof ( uint64_t ) * 2 * a * b * c )
  {
    std::cout << "Expected data of size " << sizeof ( uint64_t ) * 2 * a * b * c << " but got " << bytes <<
    std::endl;
    exit ( 1 );
  }
  space = input.get ();
  if ( space != ' ' )
  {
    std::cout << "Expected space delimiter" << std::endl;
    exit ( 1 );
  }

  for ( size_t i = 0; i < a; ++i )
  {
    for ( size_t j = 0; j < b; ++j )
    {
      for ( size_t k = 0; k < c; ++k )
      {
        std::pair<uint64_t, uint64_t> & voxel = voxels.data[i][j][k];
        input.read ( ( char * )&voxel.first, sizeof ( uint64_t ) );
        input.read ( ( char * )&voxel.second, sizeof ( uint64_t ) );
      }
    }
  }
  input.close ();

  return voxels;
}

size_t
KVO::size ( int d )
{
  switch ( d )
  {
  case 0:
    return data.size ();
  case 1:
    return data[0].size ();
  case 2:
    return data[0][0].size ();
  default:
    assert ( 0 );
  }
}
