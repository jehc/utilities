#include "KVOViewerWidget.h"

#include <cassert>
#include <iostream>

#include "omp.h"

KVOViewerWidget::KVOViewerWidget ( const QGLFormat & format, QWidget * parent, const std::string & filename )
  : QGLView ( format, parent )
  , filename ( filename )
  , slice ( 1.0 )
  , scale ( 1.0 )
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

  initializeTexture ( voxels );

  initializeShaders ( painter );

  initializeFBO (painter);

  // Set the background to white
  glClearColor ( 1.0, 1.0, 1.0, 1.0 );

  resizeGL ( width (), height () );
}

void
KVOViewerWidget::initializeFBO (QGLPainter * painter)
{
  // Create the FBO
  painter->glGenFramebuffers (1, &fbo);

  // Create the color texture
  glGenTextures (1, &fullScreenTexture);

  // Unbind the FBO
  painter->glBindFramebuffer (GL_FRAMEBUFFER, 0);
}

void
KVOViewerWidget::initializeTexture ( KVO & voxels )
{
  transferFunction.resize ( 1024 * 4 );

  // enable 1d textures
  glEnable ( GL_TEXTURE_1D );

  // create the texture
  glGenTextures ( 1, &transferFunctionTexture );

  // bind the texture
  glBindTexture ( GL_TEXTURE_1D, transferFunctionTexture );

  // interpolation stuff
  glTexParameteri ( GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP );

  // create texture
  std::vector<GLfloat> tf ( 1024 );

  // copy buffer
  glTexImage1D ( GL_TEXTURE_1D, 0, GL_RGBA32F, 1024, 0, GL_ALPHA, GL_FLOAT, &tf [0] );

  // unbind texture
  glBindTexture ( GL_TEXTURE_1D, 0 );

  // disable 1d textures
  glDisable ( GL_TEXTURE_1D );

  // enable 3d textures
  glEnable ( GL_TEXTURE_3D );

  // create the texture
  glGenTextures ( 1, &voxelTexture );

  // bind the texture
  glBindTexture ( GL_TEXTURE_3D, voxelTexture );

  // The magical parameters make the bad people go away
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP );
  glTexParameteri ( GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP );

  // create buffer
  std::vector<GLfloat> buffer ( 4 * voxels.size ( 0 ) * voxels.size ( 1 ) * voxels.size ( 2 ) );
  double max = 0;

  // populate buffer
  for ( size_t j = 0; j < voxels.size ( 1 ); ++j )
  {
    for ( size_t k = 0; k < voxels.size ( 2 ); ++k )
    {
      for ( size_t i = 0; i < voxels.size ( 0 ); ++i )
      {
        if (voxels[i][j][k].a > max)
        {
          max = voxels[i][j][k].a;
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
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 3] = isnan(voxels[i][j][k].a) ? 0.5/1024.0 : (1023.0*voxels[i][j][k].a / max + 0.5)/1024.0;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k)] = isnan(voxels[i][j][k].r) ? 0.0 : voxels[i][j][k].r / 255.0;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 1] = isnan(voxels[i][j][k].g) ? 0.0 : voxels[i][j][k].g / 255.0;
        buffer[4*(j*voxels.size (0) + i + voxels.size (1)*voxels.size (0)*k) + 2] = isnan(voxels[i][j][k].b) ? 0.0 : voxels[i][j][k].b / 255.0;
      }
    }
  }

  // copy buffer
  glTexImage3D ( GL_TEXTURE_3D, 0, GL_RGBA32F, voxels.size ( 0 ), voxels.size ( 1 ), voxels.size (
      2 ), 0, GL_RGBA, GL_FLOAT, &buffer [0] );

  // unbind texture
  glBindTexture ( GL_TEXTURE_3D, voxelTexture );

  // disable 3d textures
  glDisable ( GL_TEXTURE_3D );
}

void
KVOViewerWidget::initializeShaders ( QGLPainter * painter )
{
  // Create vertex shader source
  const GLchar * vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec4 texCoord;\n"
    "uniform mat4 projectionMatrix;\n"
    "varying vec3 coord;\n"
    "void\n"
    "main ()\n"
    "{\n"
    "  coord = texCoord.stp;\n"
    "  gl_Position = projectionMatrix * vertex;\n"
    "}";

  // Create fragment shader source
  const GLchar * fragmentShaderSource =
    "uniform float samples;\n"
    "uniform sampler3D dataTex;\n"
    "uniform sampler1D transferFunction;\n"
    "uniform float scale;\n"
    "uniform float slice;\n"
    "varying vec3 coord;\n"
    "void\n"
    "main ()\n"
    "{\n"
    "  float v = texture3D (dataTex, coord).a;\n"
    "  float a0 = texture1D (transferFunction, v).a;\n"
    "  vec3 c = texture1D (transferFunction, v).rgb;\n"
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

  // Create vertex shader source
  const GLchar * fullScreenVertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec4 texCoord;\n"
    "varying vec2 coord;\n"
    "void\n"
    "main ()\n"
    "{\n"
    "  coord = texCoord.st;\n"
    "  gl_Position = vertex;\n"
    "}";

  // Create fragment shader source
  const GLchar * fullScreenFragmentShaderSource =
    "uniform sampler2D texture;\n"
    "varying vec2 coord;\n"
    "void\n"
    "main ()\n"
    "{\n"
    "  gl_FragColor = texture2D (texture, coord);\n"
    "}";

  // Create shader program
  fullScreenProgram = new QGLShaderProgram ( painter->context (), this );

  // Attach vertex shader
  fullScreenProgram->addShaderFromSourceCode ( QGLShader::Vertex, fullScreenVertexShaderSource );

  // Attach fragment shader
  fullScreenProgram->addShaderFromSourceCode ( QGLShader::Fragment, fullScreenFragmentShaderSource );

  // Link shader program
  fullScreenProgram->link ();

  // Bind the program
  fullScreenProgram->bind();

  // Get texture location
  fullScreenTextureLocation = fullScreenProgram->uniformLocation ( "texture" );

  // Get vertex location
  fullScreenVertexLocation = fullScreenProgram->attributeLocation ("vertex");
 
  // Get texcoord location
  fullScreenTexcoordLocation = fullScreenProgram->attributeLocation ("texCoord");

  // Unbind the program
  fullScreenProgram->release();
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

  // Enable texture 2D
  glEnable (GL_TEXTURE_2D);

  // Bind the FBO
  painter->glBindFramebuffer (GL_FRAMEBUFFER, fbo);

  // Use texture unit 2
  glActiveTexture (GL_TEXTURE2);

  // Bind the destination texture
  glBindTexture (GL_TEXTURE_2D, fullScreenTexture);

  // Generate storage for FBO
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA32F, width(), height(), 0, GL_RGBA, GL_FLOAT, NULL);

  // Create color attachment for FBO
  painter->glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fullScreenTexture, 0);

  glViewport (0, 0, width(), height());

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

  // Enable [1, 1-source alpha] blending program
  glBlendFunc ( GL_ONE, GL_ONE_MINUS_SRC_ALPHA );

  // Enable texture 3D
  glEnable ( GL_TEXTURE_3D );

  // Set texture unit 0 active
  painter->glActiveTexture ( GL_TEXTURE0 );

  // Bind texture
  glBindTexture ( GL_TEXTURE_3D, voxelTexture );

  // Set texture uniform to texture unit 0
  shaderProgram->setUniformValue ( textureLocation, 0 );

  // Enable texture 1D
  glEnable ( GL_TEXTURE_1D );

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
  double sliceDelta = ( voxelMax - voxelMin ) * 0.5;
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

  // Unbind texture
  glBindTexture ( GL_TEXTURE_1D, 0 );

  // Disable texture 1D
  glDisable ( GL_TEXTURE_1D );

  // Unbind texture
  glBindTexture ( GL_TEXTURE_3D, 0 );

  // Disable texture 3D
  glDisable ( GL_TEXTURE_3D );

  // Disable blending
  glDisable ( GL_BLEND );

  // Disable program
  shaderProgram->release ();

  // Unbind FBO
  painter->glBindFramebuffer (GL_FRAMEBUFFER, 0);

  glViewport (0,0,width(),height());

  // Clear the viewport
  glClear ( GL_COLOR_BUFFER_BIT );

  // Bind the full screen program
  fullScreenProgram->bind();

  // Use texture unit 2
  painter->glActiveTexture (GL_TEXTURE2);

  // Bind texture
  glBindTexture (GL_TEXTURE_2D, fullScreenTexture);

  // Set texture uniform to texture unit 2
  fullScreenProgram->setUniformValue (fullScreenTextureLocation, 2);

  // Generate vertices
  std::vector<QVector3D> fullScreenVertices;
  fullScreenVertices.push_back (QVector3D (-1, -1, 0));
  fullScreenVertices.push_back (QVector3D ( 1, -1, 0));
  fullScreenVertices.push_back (QVector3D ( 1,  1, 0));
  fullScreenVertices.push_back (QVector3D (-1,  1, 0));

  // Copy vertices
  fullScreenProgram->setAttributeArray ( fullScreenVertexLocation, &fullScreenVertices[0] );

  // Enable the vertices
  fullScreenProgram->enableAttributeArray ( fullScreenVertexLocation );

  // Generate texcoords
  std::vector<QVector3D> fullScreenTexcoords;
  fullScreenTexcoords.push_back (QVector2D (0, 0));
  fullScreenTexcoords.push_back (QVector2D (1, 0));
  fullScreenTexcoords.push_back (QVector2D (1, 1));
  fullScreenTexcoords.push_back (QVector2D (0, 1));

  // Copy texcoords
  fullScreenProgram->setAttributeArray ( fullScreenTexcoordLocation, &fullScreenTexcoords[0] );

  // Enable the texcoords
  fullScreenProgram->enableAttributeArray ( fullScreenTexcoordLocation );

  // Draw the rectangle
  glDrawArrays (GL_POLYGON, 0, 4);

  // Disable the vertices
  fullScreenProgram->disableAttributeArray ( fullScreenVertexLocation );

  // Disable the texcoords
  fullScreenProgram->disableAttributeArray ( fullScreenTexcoordLocation );

  // Unbind the full screen program
  fullScreenProgram->release();

  // Unbind texture
  glBindTexture (GL_TEXTURE_2D, 0);

  // Disable texture 2D
  glDisable (GL_TEXTURE_2D);
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
  // enable 1d textures
  glEnable ( GL_TEXTURE_1D );

  // bind the texture
  glBindTexture ( GL_TEXTURE_1D, transferFunctionTexture );

  // copy buffer
  glTexImage1D ( GL_TEXTURE_1D, 0, GL_RGBA32F, 1024, 0, GL_RGBA, GL_FLOAT, &transferFunction [0] );

  // unbind texture
  glBindTexture ( GL_TEXTURE_1D, 0 );

  // disable 1d textures
  glDisable ( GL_TEXTURE_1D );
  repaint ();
}

void
KVOViewerWidget::sliceChanged ( int v )
{
  slice = 1.0 - v / 100.0;
  repaint ();
}

void
KVOViewerWidget::scaleChanged ( int v )
{
  scale = v / 100.0;
  repaint ();
}
