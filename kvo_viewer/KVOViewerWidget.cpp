#include "KVOViewerWidget.h"

#include <cassert>
#include <iostream>
#include <fstream>

#include "omp.h"

KVOViewerWidget::KVOViewerWidget ( const QGLFormat & format, QWidget * parent, const std::string & filename )
  : QGLView ( format, parent )
  , filename ( filename )
  , numSlices (1000)
  , slice ( 1.0 )
  , scale ( 1.0 )
  , realColors (0)
  , lighting (0)
{
}

void
KVOViewerWidget::initializeGL ( QGLPainter * painter )
{
  // load the voxel opacity data
  KVO voxels = KVO::load ( filename );

  double scale = std::max ( std::max ( voxels.size ( 0 ), voxels.size ( 1 ) ), voxels.size ( 2 ) );

  for ( int i = 0; i < 3; ++i )
  {
    aspect [i] = voxels.size ( i )/scale;
    fudge [i] = 1.0/voxels.size(i);
  }
  voxelSize = 2.0 / scale;

  std::cout << "Loaded kvo of size " << voxels.size ( 0 ) << " " << voxels.size ( 1 ) << " " << voxels.size (
    2 ) << std::endl;

  initializeShaders ( painter );

  initializeTexture ( voxels );

  // Set the background to white
  glClearColor ( 1.0, 1.0, 1.0, 1.0 );

  // Enable [1, 1-source alpha] blending program
  glBlendFunc ( GL_ONE, GL_ONE_MINUS_SRC_ALPHA );

  resizeGL ( width (), height () );
}

void
KVOViewerWidget::initializeTexture ( KVO & voxels )
{
  transferFunction.resize ( 1024 * 4 );

  // Bind program
  shaderProgram->bind();

  // create the texture
  glGenTextures ( 1, &transferFunctionTexture );

  // bind the texture
  glBindTexture ( GL_TEXTURE_1D, transferFunctionTexture );

  // interpolation stuff
  glTexParameteri ( GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );

  // create texture
  std::vector<GLfloat> tf ( 1024 );

  // copy buffer
  glTexImage1D ( GL_TEXTURE_1D, 0, GL_RGBA32F, 1024, 0, GL_ALPHA, GL_FLOAT, &tf [0] );

  // create the texture
  glGenTextures ( 1, &voxelTexture );

  // bind the texture
  glBindTexture ( GL_TEXTURE_3D, voxelTexture );

  // The magical parameters make the bad people go away
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE );

  // create buffer
  std::vector<GLfloat> buffer ( 4 * voxels.size ( 0 ) * voxels.size ( 1 ) * voxels.size ( 2 ) );
  double max = -std::numeric_limits<double>::infinity();
  double min = std::numeric_limits<double>::infinity();

#if 0
  std::ofstream outData ("data");
#endif

  // populate buffer
  for ( size_t j = 0; j < voxels.size ( 1 ); ++j )
  {
    for ( size_t k = 0; k < voxels.size ( 2 ); ++k )
    {
      for ( size_t i = 0; i < voxels.size ( 0 ); ++i )
      {
        if (voxels[i][j][k].a > max && !isnan (voxels[i][j][k].a) && !isinf (voxels[i][j][k].a))
        {
          max = voxels[i][j][k].a;
std::cout << max << " " << min << std::endl;
        }
        if (voxels[i][j][k].a < min && !isnan (voxels[i][j][k].a) && !isinf (voxels[i][j][k].a))
        {
          min = voxels[i][j][k].a;
std::cout << max << " " << min << std::endl;
        }
      }
    }
  }
  for ( size_t k = 0; k < voxels.size ( 2 ); ++k ) // depth
  {
    for ( size_t i = 0; i < voxels.size ( 0 ); ++i ) //width
    {
      for ( size_t j = 0; j < voxels.size ( 1 ); ++j ) // height
      {
#if 0
        outData << voxels[i][j][k].a << std::endl;
#endif
//        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3] = (isnan(voxels[i][j][k].a) || isinf(voxels[i][j][k].a)) ? 1023.5/1024.0 : (1023.0*(voxels[i][j][k].a - min) / (max - min) + 0.5)/1024.0;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3] = (isnan(voxels[i][j][k].a) || isinf(voxels[i][j][k].a)) ? 512.0/1024.0 : (voxels[i][j][k].a > 0 ? 1023.5/1024.0 : 0.5/1024.0);

        std::vector<float> neighbors;
        float value = voxels [i][j][k].a;
        for (int i_n = std::max (0, (int)i - 1); i_n <= std::min((int)voxels.size(0) - 1, (int)i + 1); ++i_n)
        {
          for (int j_n = std::max(0, (int)j - 1); j_n <= std::min((int)voxels.size(1) - 1, (int)j + 1); ++j_n)
          {
            for (int k_n = std::max(0, (int)k - 1); k_n <= std::min((int)voxels.size (2) - 1, (int)k + 1); ++k_n)
            {
              neighbors.push_back (voxels [i_n][j_n][k_n].a);
            }
          }
        }
        bool boundary = false;
        for (int n = 0; n < (int)neighbors.size(); ++n)
        {
          if ((value > 0 && neighbors [n] < 0) || (value < 0 && neighbors [n] > 0))
          {
            boundary = true;
            break;
          }
        }
 
 //       buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3] = (isnan(value) || isinf (value) || !boundary) ? 0.5/1024.0 : 1023.5/1024.0;

       //std::cout << buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3] << std::endl;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k)] = isnan(voxels[i][j][k].r) ? 0.0 : voxels[i][j][k].r / 255.0;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 1] = isnan(voxels[i][j][k].g) ? 0.0 : voxels[i][j][k].g / 255.0;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 2] = isnan(voxels[i][j][k].b) ? 0.0 : voxels[i][j][k].b / 255.0;
      }
    }
  }
#if 0
  outData.close();
#endif

  // copy buffer
  glTexImage3D ( GL_TEXTURE_3D, 0, GL_RGBA32F, voxels.size ( 0 ), voxels.size ( 1 ), voxels.size (
      2 ), 0, GL_RGBA, GL_FLOAT, &buffer [0] );

  // unbind texture
  glBindTexture ( GL_TEXTURE_3D, 0 );

  // create the texture
  glGenTextures ( 1, &normalTexture );

  // bind the texture
  glBindTexture ( GL_TEXTURE_3D, normalTexture );

  // The magical parameters make the bad people go away
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE );

#if 0
  std::ofstream outNormals ("normals");
#endif

  // create buffer
  std::vector<GLfloat> normalBuffer ( 3 * voxels.size ( 0 ) * voxels.size ( 1 ) * voxels.size ( 2 ) );
  for ( size_t k = 0; k < voxels.size ( 2 ); ++k ) // depth
  {
    for ( size_t i = 0; i < voxels.size ( 0 ); ++i ) //width
    {
      for ( size_t j = 0; j < voxels.size ( 1 ); ++j ) // height
      {
        GLfloat & x = normalBuffer[3*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k)];
        GLfloat & y = normalBuffer[3*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 1];
        GLfloat & z = normalBuffer[3*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 2];
        if (i == 0 || j == 0 || k == 0 || i == voxels.size (0) - 1 || j == voxels.size (1) - 1 || k == voxels.size (2) - 1)
        {
          x = y = z = 0;
          continue;
        } 
        x = buffer[4*(j*voxels.size (0) + (i+1) + voxels.size (1)*voxels.size (0)*k) + 3] - buffer[4*(j*voxels.size (0) + (i-1) + voxels.size (1)*voxels.size (0)*k) + 3];
        y = buffer[4*((j+1)*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3] - buffer[4*((j-1)*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3];
        z = buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*(k+1)) + 3] - buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*(k-1)) + 3];
        float norm = sqrt(x*x + y*y + z*z);
        if (norm > 0)
        {
          x /= norm;
          y /= norm;
          z /= norm; 
        }
#if 0
        outNormals << norm << std::endl;
#endif
      }
    }
  }

#if 0
  outNormals.close();
#endif

  // copy buffer
  glTexImage3D ( GL_TEXTURE_3D, 0, GL_RGBA32F, voxels.size ( 0 ), voxels.size ( 1 ), voxels.size (
      2 ), 0, GL_RGB, GL_FLOAT, &normalBuffer [0] );

  // unbind texture
  glBindTexture ( GL_TEXTURE_3D, 0 );

  // Unbind program
  shaderProgram->release();
}

void
KVOViewerWidget::initializeShaders ( QGLPainter * painter )
{
  // Create vertex shader source
  const GLchar * vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec4 texCoord;\n"
    "uniform mat4 projectionMatrix;\n"
    "uniform vec3 light;\n"
    "varying vec3 coord;\n"
    "varying vec3 position;\n"
    "void\n"
    "main ()\n"
    "{\n"
    "  coord = texCoord.stp;\n"
    "  gl_Position = projectionMatrix * vertex;\n"
    "  position = normalize(light - gl_Position.xyz);\n"
    "}";

  // Create fragment shader source
  const GLchar * fragmentShaderSource =
    "uniform float samples;\n"
    "uniform sampler3D dataTex;\n"
    "uniform sampler3D normalTex;\n"
    "uniform sampler1D transferFunction;\n"
    "uniform float scale;\n"
    "uniform float slice;\n"
    "uniform int realColors;\n"
    "uniform int lighting;\n"
    "varying vec3 coord;\n"
    "varying vec3 position;\n"
    "void\n"
    "main ()\n"
    "{\n"
    "  float v = texture3D (dataTex, coord).a;\n"
    "  float a0 = texture1D (transferFunction, v).a;\n"
    "  vec3 c = realColors != 0 ? texture3D (dataTex, coord).rgb : texture1D (transferFunction, v).rgb;\n"
    "  if (lighting != 0)\n"
    "  {\n"
    "    vec3 normal = texture3D (normalTex, coord).xyz;\n"
    "    float diffuse = abs (dot (normal, position));\n"
    "    c = diffuse*c;\n"
    "  }\n"
    "  const float s0 = 500;\n"
    "  float a = (1 - pow(1 - a0, s0/samples));\n"
    "  gl_FragColor = (coord.t < slice) ? scale * a * vec4(c, 1.0) : vec4(0.0,0.0,0.0,0.0);\n"
    "}";

  // Create shader program
  shaderProgram = new QGLShaderProgram ( painter->context (), this );

  // Attach vertex shader
  shaderProgram->addShaderFromSourceCode ( QGLShader::Vertex, vertexShaderSource );

  // Attach fragment shader
  shaderProgram->addShaderFromSourceCode ( QGLShader::Fragment, fragmentShaderSource );

  // Link shader program
  shaderProgram->link ();

  // Bind the program
  shaderProgram->bind();

  // Get texture location
  textureLocation = shaderProgram->uniformLocation ( "dataTex" );

  // Get normal texture location
  normalLocation = shaderProgram->uniformLocation ("normalTex");

  // Get real colors location
  realColorsLocation = shaderProgram->uniformLocation ("realColors");

  // Get lighting location
  lightingLocation = shaderProgram->uniformLocation ("lighting");

  // Get light location
  lightLocation = shaderProgram->uniformLocation ("light");

  // Get transfer function location
  transferFunctionLocation = shaderProgram->uniformLocation ( "transferFunction" );

  // Get samples location
  samplesLocation = shaderProgram->uniformLocation ( "samples" );

  // Get projection matrix location
  projectionMatrixLocation = shaderProgram->uniformLocation ( "projectionMatrix" );

  // Get vertex location
  vertexLocation = shaderProgram->attributeLocation ( "vertex" );

  // Get texCoord location
  texCoordLocation = shaderProgram->attributeLocation ( "texCoord" );

  // Get slice location
  sliceLocation = shaderProgram->uniformLocation ( "slice" );

  // Get scale location
  scaleLocation = shaderProgram->uniformLocation ( "scale" );

  // Unbind program
  shaderProgram->release ();
}

void
KVOViewerWidget::resizeGL ( int width, int height )
{
  // update viewport
  glViewport ( 0, 0, width, height );
}

void
KVOViewerWidget::paintGL ( QGLPainter * painter )
{
  if ( painter->isPicking () )
  {
    assert ( 0 );
  }

  // Enable the shader program
  shaderProgram->bind ();

  // Create the FBO
  QGLFramebufferObject fbo (width(), height(), QGLFramebufferObject::NoAttachment, GL_TEXTURE_2D, GL_RGBA32F);

  // Bind the FBO
  fbo.bind();

  // Get modelview matrix
  QMatrix4x4 qModelViewMatrix = camera ()->modelViewMatrix ();
  Eigen::Matrix4d modelViewMatrix;
  for ( int i = 0; i < 4; ++i )
  {
    for ( int j = 0; j < 4; ++j )
    {
      modelViewMatrix ( i, j ) = qModelViewMatrix ( i, j );
    }
  }

  // Clear the viewport
  glClear ( GL_COLOR_BUFFER_BIT );

  // Enable blending
  glEnable ( GL_BLEND );

  // Set texture unit 3 active
  painter->glActiveTexture (GL_TEXTURE3);

  // Bind texture
  glBindTexture (GL_TEXTURE_3D, normalTexture);

  // Set texture uniform to texture unit 3);
  shaderProgram->setUniformValue (normalLocation, 3);

  // Set texture unit 0 active
  painter->glActiveTexture ( GL_TEXTURE0 );

  // Bind texture
  glBindTexture ( GL_TEXTURE_3D, voxelTexture );

  // Set texture uniform to texture unit 0
  shaderProgram->setUniformValue ( textureLocation, 0 );

  // Set texture unit 1 active
  painter->glActiveTexture ( GL_TEXTURE1 );

  // Bind texture
  glBindTexture ( GL_TEXTURE_1D, transferFunctionTexture );

  // Set texture uniform to texture unit 1
  shaderProgram->setUniformValue ( transferFunctionLocation, 1 );

  // Set projection matrix
  shaderProgram->setUniformValue ( projectionMatrixLocation,
    camera ()->projectionMatrix ( ( double )width () / height () ) );

  // Set scale
  shaderProgram->setUniformValue ( scaleLocation, (GLfloat) scale );

  // Set slice
  shaderProgram->setUniformValue ( sliceLocation, (GLfloat) slice );

  // Set real colors
  shaderProgram->setUniformValue (realColorsLocation, (GLint)realColors);

  // Set lighting
  shaderProgram->setUniformValue (lightingLocation, (GLint)lighting);

  // Set light
  const QVector3D & light = camera()->eye();
  shaderProgram->setUniformValue (lightLocation, light);

  // Find corners
  std::vector<Eigen::Vector4d> corners;
  corners.push_back ( Eigen::Vector4d ( -aspect[0], -aspect[1], -aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( -aspect[0], -aspect[1], aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( -aspect[0], aspect[1], -aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( -aspect[0], aspect[1], aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( aspect[0], -aspect[1], -aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( aspect[0], -aspect[1], aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( aspect[0], aspect[1], -aspect[2], 1.0 ) );
  corners.push_back ( Eigen::Vector4d ( aspect[0], aspect[1], aspect[2], 1.0 ) );
  std::vector<std::pair<int, int> > edges;
  edges.push_back ( std::make_pair ( 0, 1 ) );
  edges.push_back ( std::make_pair ( 0, 2 ) );
  edges.push_back ( std::make_pair ( 0, 4 ) );
  edges.push_back ( std::make_pair ( 1, 3 ) );
  edges.push_back ( std::make_pair ( 1, 5 ) );
  edges.push_back ( std::make_pair ( 2, 3 ) );
  edges.push_back ( std::make_pair ( 2, 6 ) );
  edges.push_back ( std::make_pair ( 3, 7 ) );
  edges.push_back ( std::make_pair ( 4, 5 ) );
  edges.push_back ( std::make_pair ( 4, 6 ) );
  edges.push_back ( std::make_pair ( 5, 7 ) );
  edges.push_back ( std::make_pair ( 6, 7 ) );
  for ( size_t i = 0; i < corners.size (); ++i )
  {
    corners [i] = modelViewMatrix * corners [i];
    corners [i] = corners [i] / corners[i][3];
  }
  double min = std::numeric_limits<double>::infinity ();
  double max = -std::numeric_limits<double>::infinity ();
  for ( size_t i = 0; i < corners.size (); ++i )
  {
    if ( corners[i][2] < min )
    {
      min = corners[i][2];
    }
    if ( corners[i][2] > max )
    {
      max = corners[i][2];
    }
  }

  std::vector<Eigen::Vector4d> voxelCorners;
  voxelCorners.push_back ( Eigen::Vector4d ( -voxelSize, -voxelSize, -voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( -voxelSize, -voxelSize, voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( -voxelSize, voxelSize, -voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( -voxelSize, voxelSize, voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( voxelSize, -voxelSize, -voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( voxelSize, -voxelSize, voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( voxelSize, voxelSize, -voxelSize, 1.0 ) );
  voxelCorners.push_back ( Eigen::Vector4d ( voxelSize, voxelSize, voxelSize, 1.0 ) );
  for ( size_t i = 0; i < corners.size (); ++i )
  {
    voxelCorners [i] = modelViewMatrix * voxelCorners [i];
    voxelCorners [i] = voxelCorners [i] / corners[i][3];
  }
  double voxelMin = std::numeric_limits<double>::infinity ();
  double voxelMax = -std::numeric_limits<double>::infinity ();
  for ( size_t i = 0; i < corners.size (); ++i )
  {
    if ( voxelCorners[i][2] < voxelMin )
    {
      voxelMin = voxelCorners[i][2];
    }
    if ( voxelCorners[i][2] > voxelMax )
    {
      voxelMax = voxelCorners[i][2];
    }
  }

  // Generate slices
  double sliceDelta = (max - min)/numSlices;//( voxelMax - voxelMin ) * 0.5;
  std::vector<std::vector<std::pair<double, Eigen::Vector4d> > > slices ( ( max - min ) / sliceDelta );
#pragma omp parallel for
  for ( size_t s = 0; s < slices.size (); ++s )
  {
    std::vector<std::pair<double, Eigen::Vector4d> > & intersections = slices[s];

    for ( size_t i = 0; i < edges.size (); ++i )
    {
      const std::pair<int, int> & edge = edges [i];
      Eigen::Matrix3d A;
      A << corners[edge.first][0] - corners[edge.second][0], 1, 0,
      corners[edge.first][1] - corners[edge.second][1], 0, 1,
      corners[edge.first][2] - corners[edge.second][2], 0, 0;

      Eigen::Vector3d b ( corners[edge.first][0],
                          corners[edge.first][1],
                          corners[edge.first][2] - ( min + s * sliceDelta ) );

      Eigen::Vector3d x = A.inverse () * b;

      if ( x[0] < 0 || x[0] > 1 || x[0] != x[0] )
      {
        continue;
      }

      Eigen::Vector4d intersection =
        ( corners[edge.first] + x[0] * ( corners[edge.second] - corners[edge.first] ) );
      intersections.push_back ( std::make_pair ( 0, intersection ) );
    }

  }

  // Find center of slices and the angles
  int total_vertices = 0;
  std::vector<Eigen::Vector4d> centers ( slices.size () );
  for ( size_t i = 0; i < slices.size (); ++i )
  {
    std::vector<std::pair<double, Eigen::Vector4d> > & intersections = slices [i];
    Eigen::Vector4d & center = centers [i];
    center.setZero ();
    for ( size_t j = 0; j < intersections.size (); ++j )
    {
      center += intersections [j].second;
    }
    center /= intersections.size ();

    for ( size_t j = 0; j < intersections.size (); ++j )
    {
      intersections [j].first = atan2 ( intersections [j].second[1] - center[1],
        intersections [j].second[0] - center[0] );
    }

    std::sort ( intersections.begin (), intersections.end (), comp );
    total_vertices += intersections.size ();
  }

  // Make texcoord array
  std::vector<QVector3D> texcoords ( total_vertices );
  int index = 0;
  for ( size_t i = 0; i < slices.size (); ++i )
  {
    std::vector<std::pair<double, Eigen::Vector4d> > & intersections = slices [i];
    for ( size_t j = 0; j < intersections.size (); ++j )
    {
      Eigen::Vector4d n = modelViewMatrix.inverse () * intersections [j].second;
      n /= n[3];
      texcoords [index].setX((1.0 - fudge[0]) * 0.5 * n[0] / aspect[0] + 0.5);
      texcoords [index].setY((1.0 - fudge[1]) * 0.5 * n[1] / aspect[1] + 0.5);
      texcoords [index].setZ((1.0 - fudge[2]) * 0.5 * n[2] / aspect[2] + 0.5);
      ++index;
    }
  }

  // Copy texcoords
  shaderProgram->setAttributeArray ( texCoordLocation, &texcoords[0]);

  // Enable the tex coords
  shaderProgram->enableAttributeArray ( texCoordLocation );

  // Make vertex array
  std::vector<QVector3D> vertices ( total_vertices );
  index = 0;
  for ( size_t i = 0; i < slices.size (); ++i )
  {
    std::vector<std::pair<double, Eigen::Vector4d> > & intersections = slices [i];
    for ( size_t j = 0; j < intersections.size (); ++j )
    {
      const Eigen::Vector4d & n = intersections [j].second;
      vertices [index].setX(n [0]);
      vertices [index].setY(n [1]);
      vertices [index].setZ(n [2]);
      ++index;
    }
  }

  // Set number of slices
  shaderProgram->setUniformValue ( samplesLocation, ( GLfloat )slices.size () );

  // Copy vertices
  shaderProgram->setAttributeArray ( vertexLocation, &vertices[0] );

  // Enable the vertices
  shaderProgram->enableAttributeArray ( vertexLocation );

  // Render slices back to front
  index = 0;
  for ( size_t i = 0; i < slices.size (); ++i )
  {
    glDrawArrays ( GL_POLYGON, index, slices[i].size () );
    index += slices[i].size ();
  }

  // Disable the vertices
  shaderProgram->disableAttributeArray ( vertexLocation );

  // Disable the tex coords
  shaderProgram->disableAttributeArray ( texCoordLocation );

  // Disable blending
  glDisable ( GL_BLEND );

  // Disable program
  shaderProgram->release ();

  // Unbind FBO
  fbo.release();

  // Blit the FBO
  QGLFramebufferObject::blitFramebuffer (0, QRect(0,0,width(),height()),&fbo,QRect(0,0,width(),height()));
}

void
KVOViewerWidget::transferFunctionRChanged ( const QVector<double> & transferFunctionR )
{
  for ( int i = 0; i < transferFunctionR.size (); ++i )
  {
    transferFunction [4 * i] = transferFunctionR [i];
  }
  transferFunctionChanged ();
}

void
KVOViewerWidget::transferFunctionGChanged ( const QVector<double> & transferFunctionG )
{
  for ( int i = 0; i < transferFunctionG.size (); ++i )
  {
    transferFunction [4 * i + 1] = transferFunctionG [i];
  }
  transferFunctionChanged ();
}

void
KVOViewerWidget::transferFunctionBChanged ( const QVector<double> & transferFunctionB )
{
  for ( int i = 0; i < transferFunctionB.size (); ++i )
  {
    transferFunction [4 * i + 2] = transferFunctionB [i];
  }
  transferFunctionChanged ();
}

void
KVOViewerWidget::transferFunctionAChanged ( const QVector<double> & transferFunctionA )
{
  for ( int i = 0; i < transferFunctionA.size (); ++i )
  {
    transferFunction [4 * i + 3] = transferFunctionA [i];
  }
  transferFunctionChanged ();
}

void
KVOViewerWidget::transferFunctionChanged ()
{
  // Bind the program
  shaderProgram->bind();

  // bind the texture
  glBindTexture ( GL_TEXTURE_1D, transferFunctionTexture );

  // copy buffer
  glTexImage1D ( GL_TEXTURE_1D, 0, GL_RGBA32F, 1024, 0, GL_RGBA, GL_FLOAT, &transferFunction [0] );

  // Unbind the program
  shaderProgram->release();

  repaint ();
}

void
KVOViewerWidget::sliceChanged ( int v )
{
  slice = v / 100.0;
  repaint ();
}

void
KVOViewerWidget::scaleChanged ( int v )
{
  scale = v / 100.0;
  repaint ();
}

void
KVOViewerWidget::numSlicesChanged (int n)
{
  numSlices = n;
  repaint ();
}

void
KVOViewerWidget::realColorsChanged (int state)
{
  realColors = state;
  repaint ();
}

void
KVOViewerWidget::lightingChanged (int state)
{
  lighting = state;
  repaint ();
}
