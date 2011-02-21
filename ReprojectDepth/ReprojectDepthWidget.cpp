#include <cassert>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>
#include <QKeyEvent>

#include <GL/glew.h>

#include "ReprojectDepthWidget.h"
#include "ShaderException.h"

ReprojectDepthWidget::ReprojectDepthWidget( const std::string & calibFile,
                                            const std::string & imageManifestFile )
  : init(false), depthPatches ( 0 ), depthColors ( 0 ),
  depthVertices ( 0 ), maxDepth ( 6 ), minDepth (0.4)
{
  makeCurrent();
  GLenum err = glewInit();
  if (GLEW_OK != err)
  {
	  std::cerr << "Error: " << glewGetErrorString(err) << std::endl;
  }

  loadManifest ( imageManifestFile );
  loadCalibParameters ( calibFile );
  setAttribute ( Qt::WA_PaintOnScreen );
  setAttribute ( Qt::WA_NoSystemBackground );
  setFixedSize ( raw_depth_size.at<int>(0,0), raw_depth_size.at<int>(0,1) );
  // TODO maxdepth
}

ReprojectDepthWidget::~ReprojectDepthWidget()
{
  delete[] depthColors;
  delete[] depthVertices;
}

void ReprojectDepthWidget::loadPair ( )
{
  currentIter = captureIter;
  loadColorImage ();
  loadDepthMap ();
  ++captureIter;
}

void ReprojectDepthWidget::loadColorImage()
{
  std::stringstream ss_color;
  ss_color << *currentIter << ".color.png";
 
  cv::Mat temp = cv::imread ( ss_color.str().c_str () );
  cv::Mat colorImage;
  cv::undistort ( temp, colorImage, rgb_intrinsics, rgb_distortion );

  std::stringstream ss_color_calib;
  ss_color_calib << *currentIter << ".color.calib.jpg";
  cv::imwrite (ss_color_calib.str().c_str(), colorImage);
}

float ReprojectDepthWidget::convertToDepth ( float kd )
{
  double b = depth_base_and_offset.at<float>(0,0);
  double doff = depth_base_and_offset.at<float>(0, 1);
  double f = (depth_intrinsics.at<double>(0,0) + depth_intrinsics.at<double>(1,1))/2;
  double z = b*f/(1./8*(doff - kd));
  return z;
}

void ReprojectDepthWidget::loadCalibParameters ( const std::string & filename )
{
  cv::FileStorage fs (filename, cv::FileStorage::READ);

  fs["rgb_intrinsics"] >> rgb_intrinsics;
  fs["rgb_distortion"] >> rgb_distortion;
  fs["depth_intrinsics"] >> depth_intrinsics;
  fs["depth_distortion"] >> depth_distortion;
  fs["R"] >> R;
  fs["T"] >> T;
  fs["rgb_size"] >> rgb_size;
  fs["raw_rgb_size"] >> raw_rgb_size;
  fs["depth_size"] >> depth_size;
  fs["raw_depth_size"] >> raw_depth_size;
  fs["depth_base_and_offset"] >> depth_base_and_offset;

  fs.release();
}

void ReprojectDepthWidget::loadDepthMap ( )
{
  if ( depthColors )
  {
    delete[] depthColors;
    depthColors = 0;
  }

  if ( depthVertices )
  {
    delete[] depthVertices;
    depthVertices = 0;
  }

  const float invalidDepth = 2046;
  const int border = 0;
  std::stringstream ss;
  ss << *currentIter << ".depth.yml";
  IplImage * tmp = (IplImage *)cvLoad (ss.str().c_str());
  cv::Mat buffer (tmp);
  int validCount = 0;
  for ( int j = 0; j < buffer.rows; ++j )
  {
    for ( int i = 0; i < buffer.cols; ++i )
    {
      if ( i <= border || j <= border || i >= buffer.cols - border - 1 || j >= buffer.rows -
           border - 1 )
      {
        buffer.at<float>( j, i ) = 2047;
      }
    }
  }
  cv::Mat buffer2 ( buffer.rows, buffer.cols, CV_32FC1 );
  for ( int i = 0; i < buffer.cols; ++i )
  {
    for ( int j = 0; j < buffer.rows; ++j )
    {
      if ( buffer.at<float>( j, i ) > invalidDepth )
      {
        buffer2.at<float>( j, i ) = 0;
      }
      else
      {
        buffer2.at<float>( j, i ) = buffer.at<float>( j, i );
      }
    }
  }
  cv::Mat image;
  //We need to avoid linear interpolation here.  Breaking down undistort into
  //its two components instead.
  //cv::undistort ( buffer, image, depth_intrinsics, depth_distortion );
  cv::Mat map1, map2;
  cv::initUndistortRectifyMap (depth_intrinsics, depth_distortion, cv::Mat::eye(3, 3, CV_32FC1), depth_intrinsics, buffer.size(), CV_32FC1, map1, map2);
  cv::remap (buffer, image, map1, map2, cv::INTER_NEAREST);
  cvReleaseImage (&tmp);
  for ( int i = 0; i < image.cols; ++i )
  {
    for ( int j = 0; j < image.rows; ++j )
    {
      image.at<float> (j, i) = convertToDepth (image.at<float>(j, i));
    }
  }
  for ( int i = 0; i < image.cols - 1; ++i )
  {
    for ( int j = 0; j < image.rows - 1; ++j )
    {
      if ( image.at<float>( j, i ) > 0 &&
           image.at<float>( j + 1, i ) > 0 &&
           image.at<float>( j + 1, i + 1 ) > 0 &&
           image.at<float>( j, i + 1 ) > 0 )
      {
        ++validCount;
      }
    }
  }
  depthPatches = validCount;
  depthVertices = new GLfloat[validCount * 6 * 3];
  depthColors = new GLfloat[validCount * 6];
  int vertexIndex = 0;
  for ( int i = 0; i < image.cols - 1; ++i )
  {
    for ( int j = 0; j < image.rows - 1; ++j )
    {
      if ( image.at<float>( j, i ) > 0 &&
           image.at<float>( j + 1, i ) > 0 &&
           image.at<float>( j + 1, i + 1 ) > 0 &&
           image.at<float>( j, i + 1 ) > 0 )
      {
        double cx_d = depth_intrinsics.at<double> (0, 2);
        double cy_d = depth_intrinsics.at<double> (1, 2);
        double fx_d = depth_intrinsics.at<double> (0, 0);
        double fy_d = depth_intrinsics.at<double> (1, 1);

        depthVertices[vertexIndex * 6 * 3] = ( i - cx_d ) / fx_d * image.at<float>( j, i );
        depthVertices[vertexIndex * 6 * 3 + 1] = (j - cy_d) / fy_d * image.at<float>( j, i );
        depthVertices[vertexIndex * 6 * 3 + 2] = image.at<float>( j, i );
        depthColors[vertexIndex * 6] = image.at<float>( j, i ) / maxDepth;
        
        depthVertices[vertexIndex * 6 * 3 + 3] = ( i + 1 - cx_d ) / fx_d * image.at<float>( j + 1, i + 1 );
        depthVertices[vertexIndex * 6 * 3 + 4] = (( j + 1 ) - cy_d ) / fy_d * image.at<float>( j + 1, i + 1 );
        depthVertices[vertexIndex * 6 * 3 + 5] = image.at<float>( j + 1, i + 1 );
        depthColors[vertexIndex * 6 + 1] = image.at<float>( j, i ) / maxDepth;

        depthVertices[vertexIndex * 6 * 3 + 6] = ( i + 1 - cx_d ) / fx_d * image.at<float>( j, i + 1 );
        depthVertices[vertexIndex * 6 * 3 + 7] = (j - cy_d ) / fy_d * image.at<float>( j, i + 1 );
        depthVertices[vertexIndex * 6 * 3 + 8] = image.at<float>( j, i + 1 );
        depthColors[vertexIndex * 6 + 2] = image.at<float>( j, i ) / maxDepth;

        depthVertices[vertexIndex * 6 * 3 + 9] = ( i + 1 - cx_d ) / fx_d * image.at<float>( j + 1, i + 1 );
        depthVertices[vertexIndex * 6 * 3 + 10] = (( j + 1 ) - cy_d)  / fy_d * image.at<float>( j + 1, i + 1 );
        depthVertices[vertexIndex * 6 * 3 + 11] = image.at<float>( j + 1, i + 1 );
        depthColors[vertexIndex * 6 + 3] = image.at<float>( j, i ) / maxDepth;

        depthVertices[vertexIndex * 6 * 3 + 12] = ( i - cx_d ) / fx_d * image.at<float>( j, i );
        depthVertices[vertexIndex * 6 * 3 + 13] = (j - cy_d ) / fy_d * image.at<float>( j, i );
        depthVertices[vertexIndex * 6 * 3 + 14] = image.at<float>( j, i );
        depthColors[vertexIndex * 6 + 4] = image.at<float>( j, i ) / maxDepth;

        depthVertices[vertexIndex * 6 * 3 + 15] = ( i - cx_d ) / fx_d * image.at<float>( j + 1, i );
        depthVertices[vertexIndex * 6 * 3 + 16] = (( j + 1 ) - cy_d) / fy_d * image.at<float>( j + 1, i );
        depthVertices[vertexIndex * 6 * 3 + 17] = image.at<float>( j + 1, i );
        depthColors[vertexIndex * 6 + 5] = image.at<float>( j, i ) / maxDepth;
        ++vertexIndex;
      }
    }
  }
}

void ReprojectDepthWidget::nextImage()
{
  if (captureIter == captures.end())
  {
    exit (0);
  }
  init = true;
  loadPair();
  updateGL();
  //savePly();
}

void ReprojectDepthWidget::savePly()
{
  std::stringstream ss_color;
  ss_color << *currentIter << ".color.calib.jpg";
  cv::Mat color = cv::imread (ss_color.str().c_str());

  std::stringstream ss_depth;
  ss_depth << *currentIter << ".depth.calib.yml";
  IplImage * tmp = (IplImage *)cvLoad (ss_depth.str().c_str());
  cv::Mat depth (tmp);

  std::stringstream ss_ply;
  ss_ply << *currentIter << ".ply";
  std::ofstream output (ss_ply.str().c_str());

  output << "ply" << std::endl;
  output << "format ascii 1.0" << std::endl;
  output << "element face 0" << std::endl;
  output << "property list uchar int vertex_indices" << std::endl;
  output << "element vertex " << color.rows*color.cols << std::endl;
  output << "property float x" << std::endl;
  output << "property float y" << std::endl;
  output << "property float z" << std::endl;
  output << "property uchar diffuse_blue" << std::endl;
  output << "property uchar diffuse_green" << std::endl;
  output << "property uchar diffuse_red" << std::endl;
  output << "end_header" << std::endl;

  for (int j = 0; j < color.rows; ++j)
  {
    for (int i = 0; i < color.cols; ++i)
    {
      cv::Mat point (3, 1, CV_64FC1);
      point.at<double>(0, 0) = i;
      point.at<double>(1, 0) = j;
      point.at<double>(2, 0) = 1;
      point = rgb_intrinsics.inv() * point;
      point = depth.at<float>(j, i) * point;
      output << point.at<double>(0, 0) << " " << point.at<double>(1, 0) << " " << point.at<double>(2, 0) << " ";
      cv::Vec3b c = color.at<cv::Vec3b>(j, i);
      output << (unsigned int)c[0] << " " << (unsigned int)c[1] << " " << (unsigned int)c[2] << std::endl;
    }
  }
  cvReleaseImage (&tmp);
  output.close();
}

void ReprojectDepthWidget::keyPressEvent ( QKeyEvent * event )
{
  switch ( event->key () )
  {
  case Qt::Key_Space:
    nextImage ();
    break;
  case Qt::Key_Escape:
    exit ( 0 );
  }
}

void ReprojectDepthWidget::loadManifest ( const std::string & filename )
{
  std::ifstream input ( filename.c_str () );

  while ( input )
  {
    std::string capture;
    if (!(input >> capture))
    {
      break;
    }
    captures.push_back (capture);
  }
  captureIter = captures.begin();
  input.close();
}

void ReprojectDepthWidget::initializeGL ()
{
  // Set the clear color to black
  glClearColor ( 0, 0, 0, 0 );

  GLuint depthVshader = glCreateShader ( GL_VERTEX_SHADER );

  const char * depthVshaderSrc[1] = {
    "attribute vec4 vertex;\n"
    "attribute vec4 color;\n"
    "uniform mat4 ModelViewMatrix;\n"
    "uniform mat4 ProjectionMatrix;\n"
    "varying vec4 newColor;\n"
    "void main(void)\n"
    "{\n"
    "  gl_Position = ProjectionMatrix * ModelViewMatrix * vec4(vertex.xyz, 1);\n"
    "  newColor = vec4(0, color.r, 0, 1);\n"
    "}\n"
  };

  glShaderSource ( depthVshader, 1, depthVshaderSrc, 0 );

  glCompileShader ( depthVshader );

  GLint value = 0;
  glGetShaderiv ( depthVshader, GL_COMPILE_STATUS, &value );
  if ( value == GL_FALSE )
  {
    glGetShaderiv ( depthVshader, GL_INFO_LOG_LENGTH, &value );
    char * log = new char [value];
    GLint len;
    glGetShaderInfoLog ( depthVshader, value, &len, log );
    std::string stringLog ( log, value );
    delete [] log;
    throw ShaderException ( ShaderException::CompileError, stringLog );
  }

  GLuint depthFshader = glCreateShader ( GL_FRAGMENT_SHADER );

  const char * depthFshaderSrc[1] = {
    "varying vec4 newColor;\n"
    "void main (void)\n"
    "{\n"
    "  gl_FragColor = vec4(newColor.rgb, 1);\n"
    "}\n"
  };

  glShaderSource ( depthFshader, 1, depthFshaderSrc, 0 );

  glCompileShader ( depthFshader );

  glGetShaderiv ( depthFshader, GL_COMPILE_STATUS, &value );
  if ( value == GL_FALSE )
  {
    glGetShaderiv ( depthFshader, GL_INFO_LOG_LENGTH, &value );
    char * log = new char [value];
    GLint len;
    glGetShaderInfoLog ( depthFshader, value, &len, log );
    std::string stringLog ( log, value );
    delete [] log;
    throw ShaderException ( ShaderException::CompileError, stringLog );
  }

  depthProgram = glCreateProgram ();

  glAttachShader ( depthProgram, depthVshader );

  glAttachShader ( depthProgram, depthFshader );

  glLinkProgram ( depthProgram );

  glGetProgramiv ( depthProgram, GL_LINK_STATUS, &value );
  if ( value == GL_FALSE )
  {
    glGetProgramiv ( depthProgram, GL_INFO_LOG_LENGTH, &value );
    char * log = new char [value];
    GLint len;
    glGetProgramInfoLog ( depthProgram, value, &len, log );
    std::string stringLog ( log, value );
    delete [] log;
    throw ShaderException ( ShaderException::LinkError, stringLog );
  }

  depthVertexAttribute = glGetAttribLocation ( depthProgram, "vertex" );

  depthColorAttribute = glGetAttribLocation ( depthProgram, "color" );

  projection_uniform = glGetUniformLocation (depthProgram, "ProjectionMatrix");

  modelview_uniform = glGetUniformLocation (
    depthProgram,
    "ModelViewMatrix" );

  glEnable(GL_DEPTH_TEST);
}

void ReprojectDepthWidget::resizeGL ( int w, int h )
{
  // Viewport is the size of the widget
  glViewport ( 0, 0, w, h );
}

void ReprojectDepthWidget::paintGL ()
{
  if (!init)
  {
    return;
  }
  double n = minDepth;
  double f = maxDepth;
  double cx_rgb = rgb_intrinsics.at<double> (0, 2);
  double cy_rgb = rgb_intrinsics.at<double> (1, 2);
  double fx_rgb = rgb_intrinsics.at<double> (0, 0);
  double fy_rgb = rgb_intrinsics.at<double> (1, 1);
  double fw = n * width() / fx_rgb;
  double l = fw * ( -cx_rgb / width() );
  double fh = n * height() / fy_rgb;
  double r = fw * ( 1 - cx_rgb / width() );
  double t = fh * ( -cy_rgb  / height() );
  double b = fh * ( 1 - cy_rgb / height() );

  cv::Mat Rinv = R.inv();

  GLfloat modelview [16];
  for (int i = 0; i < 16; ++i)
  {
    modelview[i] = 0;
  }
  for (int j = 0; j < 3; ++j)
  {
    for (int i = 0; i < 3; ++i)
    {
      modelview [i * 4 + j] = Rinv.at<double> (j, i);
    }
  }
  modelview [12] = -T.at<double>(0, 0);
  modelview [13] = -T.at<double>(1, 0);
  modelview [14] = -T.at<double>(2, 0);
  modelview [15] = 1;

  double A = (r + l)/(r - l);
  double B = (t + b)/(t - b);
  double C = -(f + n)/(f - n);
  double D = -2*f*n/(f - n);

  GLfloat projection [16];
  projection [0] = 2*n/(r - l);
  projection [1] = 0;
  projection [2] = 0;
  projection [3] = 0;
  projection [4] = 0;
  projection [5] = 2*n/(t - b);
  projection [6] = 0;
  projection [7] = 0;
  projection [8] = -A;
  projection [9] = -B;
  projection [10] = -C;
  projection [11] = 1;
  projection [12] = 0;
  projection [13] = 0;
  projection [14] = D;
  projection [15] = 0;

  glUseProgram ( depthProgram );

  glUniformMatrix4fv (modelview_uniform, 1, GL_FALSE, modelview);

  glUniformMatrix4fv (projection_uniform, 1, GL_FALSE, projection);

  // Clear the screen
  glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  glVertexAttribPointer ( depthVertexAttribute,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          0,
                          depthVertices );

  glEnableVertexAttribArray ( depthVertexAttribute );

  glVertexAttribPointer ( depthColorAttribute, 1, GL_FLOAT, GL_FALSE, 0,
                          depthColors );

  glEnableVertexAttribArray ( depthColorAttribute );

  glDrawArrays ( GL_TRIANGLES, 0, 6 * depthPatches );

  glDisableVertexAttribArray ( depthVertexAttribute );

  glDisableVertexAttribArray ( depthColorAttribute );
	
  std::vector<float> final;
  final.resize(width()*height());
  glReadPixels(0, 0, width(), height(), GL_GREEN, GL_FLOAT, &final[0]);
  cv::Mat finalDepth (height(), width(), CV_32FC1);
  for (int j = 0; j < height(); ++j)
  {
    for (int i = 0; i < width(); ++i)
    {
      finalDepth.at<float> (j, i) = final[(finalDepth.rows - 1 - j)*640 + i] * maxDepth;
    }
  }
  std::stringstream ss;
  ss << *currentIter << ".depth.calib.yml";
  IplImage tmp = finalDepth;
  cvSave (ss.str().c_str(), &tmp);
}
