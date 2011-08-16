#include <GL/glew.h>
#include "HighResViewerWidget.h"

#include "ply_io.h"

#include <pcl/point_types.h>

#include <fstream>

HighResViewerWidget::HighResViewerWidget (QWidget * parent, const std::string & bundleFile, const std::string & listFile, const std::string & tuningFile, const std::string & surfaceFile)
: QGLWidget (parent)
, bundleFile (bundleFile)
, listFile (listFile)
, tuningFile (tuningFile)
, surfaceFile (surfaceFile)
, image (0)
, overlay (0)
, displayImage (true)
, saveRangeFlag (false)
, surfaceMode (true)
{
  setFocusPolicy (Qt::ClickFocus);
  setFixedSize (1024, 768);
}

void
HighResViewerWidget::load ()
{
  loadPlyFileGL (surfaceFile, surfaceVertices, surfaceColors, surfaceIndices);
  std::cout << surfaceVertices.size() << " " << surfaceColors.size() << " " << surfaceIndices.size() << std::endl;
  BundleFile bundle (bundleFile);

  cameras = bundle.GetCameras();

  std::ifstream list (listFile.c_str());
  while (list)
  {
    std::string name;
    if (!(list >> name))
    {
      break;
    }
    files.push_back (name);
    getline (list, name);
  }
  list.close();

  std::ifstream tuningIn (tuningFile.c_str());
  if (!tuningIn)
  {
    std::cout << "Failed to open " << tuningFile << std::endl;
    exit (1);
  }
  std::vector<float> tuning;
  while (tuningIn)
  {
    float val;
    if (!(tuningIn >> val))
    {
      break;
    }
    tuning.push_back (val);
  }
  tuningIn.close();

  assert (files.size() == cameras.size());

  for (size_t i = 0; i < files.size(); ++i)
  {
    if (files[i].find ("rawcolor.jpg") == std::string::npos)
    {
      highres.push_back (i);
    }
    else if (cameras[i].IsValid() && tuning[i] > 0)
    {
      lowres.push_back (i);
    }
  }

  imageLoaded.resize (highres.size());
  pointLoaded.resize (lowres.size());

  imageTransformations.resize (highres.size());
  images.resize (highres.size());
  projectionMatrices.resize (highres.size());
  imageVertices.resize (highres.size());

  pointVertices.resize (lowres.size());
  pointColors.resize (lowres.size());
  pointTransformations.resize (lowres.size());
  indices.resize (lowres.size());

  imageTexcoords.resize (8);
  imageTexcoords [0] = 0;
  imageTexcoords [1] = 1;
  imageTexcoords [2] = 0;
  imageTexcoords [3] = 0;
  imageTexcoords [4] = 1;
  imageTexcoords [5] = 0;
  imageTexcoords [6] = 1;
  imageTexcoords [7] = 1;

}

void
HighResViewerWidget::loadImage ()
{
  Eigen::Affine3d xf;
  Eigen::AngleAxisd R (cameras[highres[image]].GetR().cast<double>());
  Eigen::Translation3d t (cameras[highres[image]].GetT().cast<double>());
  imageTransformations [image] = t*R;

  std::string imageFile = listFile;
  size_t slash = listFile.find_last_of ("/");
  imageFile.erase (slash+1);
  imageFile += files[highres[image]];
   
  std::cout << "Opening " << imageFile << std::endl;
  images [image] = cv::imread (imageFile);
  Eigen::Matrix4f projection;
  GLdouble fovy = 2*atan2(images[image].rows, 2*cameras[highres[image]].GetF());
  GLdouble aspect = (GLdouble)images[image].cols/images[image].rows;
  GLdouble zNear = 1.0;
  GLdouble zFar = 12.0;
  GLdouble focal = cameras[highres[image]].GetF();
  GLdouble f = 1.0/tan(fovy/2.0);
  projectionMatrices [image] << f/aspect, 0, 0, 0,
                  0, f, 0, 0,
                  0, 0, (zFar + zNear)/(zNear - zFar), 2*zFar*zNear/(zNear - zFar),
                  0, 0, -1, 0;
  std::cout << "Focal: " << focal << " Res: " << images[image].cols << "x" << images[image].cols << std::endl;
  Eigen::Vector3f a (-images[image].cols/2/focal, -images[image].rows/2/focal, -1);
  Eigen::Vector3f b (-images[image].cols/2/focal, images[image].rows/2/focal, -1);
  Eigen::Vector3f c (images[image].cols/2/focal, images[image].rows/2/focal, -1);
  Eigen::Vector3f d (images[image].cols/2/focal, -images[image].rows/2/focal, -1);
  imageVertices [image].resize (12);
  for (int i = 0; i < 3; ++i)
  {
    imageVertices [image][i] = 12*a[i];
    imageVertices [image][3+i] = 12*b[i];
    imageVertices [image][6+i] = 12*c[i];
    imageVertices [image][9+i] = 12*d[i];
  }
//  setFixedSize (images[image].cols, images[image].rows);
  imageLoaded [image] = true;
}

void
HighResViewerWidget::loadPoints()
{
  if (pointsCached.size() > 100)
  {
    int evictIndex = pointsCached.front();
    pointsCached.pop();
    std::vector<GLfloat> verticesTemp;
    std::vector<GLuint> indicesTemp;
    std::vector<GLubyte> colorsTemp;
    pointVertices[evictIndex].swap (verticesTemp);
    indices[evictIndex].swap (indicesTemp);
    pointColors[evictIndex].swap (colorsTemp);
    pointLoaded [evictIndex] = false;
    std::cout << "Evicted " << evictIndex << std::endl;
  }
  pointsCached.push (overlay);
  std::cout << pointsCached.size() << " clouds cached" << std::endl;
    
  std::string cloudFile = listFile;
  size_t slash = listFile.find_last_of ("/");
  cloudFile.erase (slash+1);
  cloudFile += files[lowres[overlay]];
  size_t dot = cloudFile.find_last_of (".");
  cloudFile.erase (dot);
  cloudFile += ".pcd";

  pcl::PointCloud<pcl::PointSurfel> points;
  std::cout << "Opening " << cloudFile << std::endl;
  pcl::io::loadPCDFile (cloudFile, points);

  pointVertices [overlay].reserve (3*points.size());
  pointColors [overlay].reserve (3*points.size());

  Eigen::AngleAxisd R (points.sensor_orientation_.cast<double>());
  Eigen::Translation3d t (points.sensor_origin_[0], points.sensor_origin_[1], points.sensor_origin_[2]);
  pointTransformations [overlay] = t*R;

  indices [overlay].reserve (points.size());

  std::vector<std::vector<int> > positions (points.height, std::vector<int> (points.width, -1));
  int position = 0;
  for (size_t j = 0; j < points.height; ++j)
  {
    for (size_t i = 0; i < points.width; ++i)
    {
      const pcl::PointSurfel & point = points (i, j);
      if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
      {
        continue;
      }
      positions [j][i] = position;
      ++position;
      pointVertices [overlay].push_back (point.x);
      pointVertices [overlay].push_back (point.y);
      pointVertices [overlay].push_back (point.z);
      ByteExtractor c;
      c.rgba = point.rgba;
      pointColors [overlay].push_back (c.b[0]);
      pointColors [overlay].push_back (c.b[1]);
      pointColors [overlay].push_back (c.b[2]);
    }
  }

  for (size_t j = 0; j < points.height - 1; ++j)
  {
    for (size_t i = 0; i < points.width - 1; ++i)
    {
      int index1 = positions [j][i];
      int index2 = positions [j][i+1];
      int index3 = positions [j+1][i];
      int index4 = positions [j+1][i+1];
      const pcl::PointSurfel & point1 = points (i, j);
      const pcl::PointSurfel & point2 = points (i+1, j);
      const pcl::PointSurfel & point3 = points (i, j+1);
      const pcl::PointSurfel & point4 = points (i+1, j+1);
      Eigen::Vector3f v1 (point1.x, point1.y, point1.z);
      Eigen::Vector3f v2 (point2.x, point2.y, point2.z);
      Eigen::Vector3f v3 (point3.x, point3.y, point3.z);
      Eigen::Vector3f v4 (point4.x, point4.y, point4.z);
      bool length12 = (v1 - v2).norm() < 0.25;
      bool length34 = (v3 - v4).norm() < 0.25;
      bool length13 = (v1 - v3).norm() < 0.25;
      bool length24 = (v2 - v4).norm() < 0.25;
      bool upperLeft = (index1!=-1)&&(index3!=-1)&&(index4!=-1)&&length13&&length34;
      bool upperRight = (index3!=-1)&&(index4!=-1)&&(index2!=-1)&&length34&&length24;
      bool lowerLeft = (index3!=-1)&&(index1!=-1)&&(index2!=-1)&&length12&&length13;
      bool lowerRight = (index1!=-1)&&(index2!=-1)&&(index4!=-1)&&length12&&length24;
      bool both = upperLeft&&lowerRight;
      if (upperLeft&&!both)
      {
        indices [overlay].push_back (index1);
        indices [overlay].push_back (index3);
        indices [overlay].push_back (index4);
      }
      if (upperRight&&!both)
      {
        indices [overlay].push_back (index2);
        indices [overlay].push_back (index3);
        indices [overlay].push_back (index4);
      }
      if (lowerLeft&&!both)
      {
        indices [overlay].push_back (index1);
        indices [overlay].push_back (index2);
        indices [overlay].push_back (index3);
      }
      if (lowerRight&&!both)
      {
        indices [overlay].push_back (index1);
        indices [overlay].push_back (index2);
        indices [overlay].push_back (index4);
      }
      if (both)
      {
        indices [overlay].push_back (index1);
        indices [overlay].push_back (index2);
        indices [overlay].push_back (index4);
        indices [overlay].push_back (index1);
        indices [overlay].push_back (index3);
        indices [overlay].push_back (index4);
      }
      assert (indices[overlay].size() % 3 == 0);
    }
  }
//34
//12
/*
  for (size_t i = 0; i < pointVertices [overlay].size()/3; ++i)
  {
    indices [overlay].push_back (i);
  }
*/
  pointLoaded[overlay] = true;
}

void
HighResViewerWidget::initializeGL()
{
  GLenum err = glewInit();
  if (GLEW_OK != err)
  {
    /* Problem: glewInit failed, something is seriously wrong. */
    fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
  }

  fprintf(stdout, "Status: Using GLEW %s\n", glewGetString(GLEW_VERSION));
  
  if (!imageLoaded.size() && !pointLoaded.size())
  {
    load();
  }

  glClearColor (0, 0, 0, 0);

  glActiveTexture (GL_TEXTURE0);
  glGenTextures (1, &imageTexture);
  glBindTexture (GL_TEXTURE_2D, imageTexture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 

  const char * PointVShader = 
    "uniform mat4 projection;\n"
    "uniform mat4 modelview;\n"
    "uniform mat4 modelviewinv;\n"
    "attribute vec3 vertex;\n"
    "attribute vec3 color;\n"
    "varying vec3 vcolor;\n"
    "void\n"
    "main()\n"
    "{\n"
    "  gl_Position = projection*modelviewinv*modelview*vec4(vertex, 1.0);\n"
    "  float depth = -(modelviewinv*modelview*vec4(vertex, 1.0)).z;\n"
//    "  vcolor = vec3(0.0, (depth-1)/11, 0.0);color;\n"
    "  vcolor = color;\n"
    "}\n";

  const char * PointFShader = 
    "varying vec3 vcolor;\n"
    "void\n"
    "main()\n"
    "{\n"
   "  gl_FragColor = vec4(vcolor, 1);\n"
    "}\n";

  const char * RangeVShader =
    "uniform mat4 projection;\n"
    "uniform mat4 modelview;\n"
    "uniform mat4 modelviewinv;\n"
    "attribute vec3 vertex;\n"
    "varying vec4 color;\n"
    "void\n"
    "main()\n"
    "{\n"
    "  vec4 transformed = modelviewinv*modelview*vec4(vertex, 1.0);\n"
    "  gl_Position = projection*transformed;\n"
    "  float depth = -transformed.z;\n"
    "  color = vec4(depth, 0.0, 0.0, 1.0);\n"
    "}\n";

  const char * RangeFShader =
    "varying vec4 color;\n"
    "void\n"
    "main()\n"
    "{\n"
    "  gl_FragData[0] = color;\n"
    "}\n";

  const char * ImageVShader =
    "uniform mat4 projection;\n"
    "uniform mat4 modelview;\n"
    "attribute vec3 vertex;\n"
    "attribute vec2 texcoord;\n"
    "varying vec2 texcoordvar;\n"
    "void\n"
    "main()\n"
    "{\n"
    "  texcoordvar = texcoord;\n"
    "  gl_Position = projection*vec4(vertex, 1.0);\n"
    "}\n";

  const char * ImageFShader =
    "uniform sampler2D tex;\n"
    "varying vec2 texcoordvar;\n"
    "void\n"
    "main()\n"
    "{\n"
    "  gl_FragColor = texture2D (tex, texcoordvar);\n"
    "}\n";

  pointShader = new QGLShaderProgram (this);
  pointShader->addShaderFromSourceCode (QGLShader::Vertex, PointVShader);
  pointShader->addShaderFromSourceCode (QGLShader::Fragment, PointFShader);
  pointShader->link();
  pointVertexLocation = pointShader->attributeLocation ("vertex");
  assert (pointVertexLocation != -1);
  pointColorLocation = pointShader->attributeLocation ("color");
  assert (pointColorLocation != -1);
  pointProjectionLocation = pointShader->uniformLocation ("projection");
  assert (pointProjectionLocation != -1);
  pointModelviewLocation = pointShader->uniformLocation ("modelview");
  assert (pointModelviewLocation != -1);
  pointModelviewInvLocation = pointShader->uniformLocation ("modelviewinv");
  assert (imageModelviewLocation != -1);
  pointShader->release();

  rangeShader = new QGLShaderProgram (this);
  rangeShader->addShaderFromSourceCode (QGLShader::Vertex, RangeVShader);
  rangeShader->addShaderFromSourceCode (QGLShader::Fragment, RangeFShader);
  rangeShader->link();
  rangeVertexLocation = rangeShader->attributeLocation ("vertex");
  assert (rangeVertexLocation != -1);
  rangeProjectionLocation = rangeShader->uniformLocation ("projection");
  assert (rangeProjectionLocation != -1);
  rangeModelviewLocation = rangeShader->uniformLocation ("modelview");
  assert (rangeModelviewLocation != -1);
  rangeModelviewInvLocation = rangeShader->uniformLocation ("modelviewinv");
  assert (rangeModelviewLocation != -1);
  rangeShader->release();

  imageShader = new QGLShaderProgram (this);
  imageShader->addShaderFromSourceCode (QGLShader::Vertex, ImageVShader);
  imageShader->addShaderFromSourceCode (QGLShader::Fragment, ImageFShader);
  imageShader->link();
  imageShader->bind();
  imageVertexLocation = imageShader->attributeLocation ("vertex");
  assert (imageVertexLocation != -1);
  imageTexcoordLocation = imageShader->attributeLocation ("texcoord");
  assert (imageTexcoordLocation != -1);
  imageTexLocation = imageShader->uniformLocation ("tex");
  assert (imageTexLocation != -1);
  imageProjectionLocation = imageShader->uniformLocation ("projection");
  assert (imageProjectionLocation != -1);
  imageShader->release();

  QGLWidget::initializeGL ();
}

void
HighResViewerWidget::setImage()
{
  if (!imageLoaded[image])
  {
    loadImage();
  }
  imageShader->setAttributeArray (imageTexcoordLocation, &imageTexcoords [0], 2);
  imageShader->setUniformValue (imageProjectionLocation, QMatrix4x4(projectionMatrices[image].data()).transposed());
  imageShader->setAttributeArray (imageVertexLocation, &imageVertices [image][0], 3);
  glBindTexture (GL_TEXTURE_2D, imageTexture);
  imageShader->setUniformValue (imageTexLocation, 0);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA, images [image].cols, images [image].rows, 0, GL_BGR, GL_UNSIGNED_BYTE, images [image].data);
  

  resize (images[image].cols, images[image].rows);
}

void
HighResViewerWidget::resizeGL (int width, int height)
{
  glViewport (0, 0, width, height);
}

void
HighResViewerWidget::paintGL()
{

  if (saveRangeFlag)
  {
    saveRange();
    saveRangeFlag = false;
  }

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
  imageShader->bind();
  setImage();
  imageShader->enableAttributeArray (imageVertexLocation);
  imageShader->enableAttributeArray (imageTexcoordLocation);
  std::cout << (projectionMatrices[image]*Eigen::Vector4d(imageVertices[image][6], imageVertices[image][7], imageVertices[image][8], 1)).transpose() << std::endl;
  if (displayImage)
  {
    glDrawArrays (GL_QUADS, 0, 4);
  }
  imageShader->disableAttributeArray (imageVertexLocation);
  imageShader->disableAttributeArray (imageTexcoordLocation);
  glBindTexture (GL_TEXTURE_2D, 0);
  imageShader->release();

  if (!pointLoaded [overlay])
  {
    loadPoints();
  }
  pointShader->bind();
  pointShader->enableAttributeArray (pointVertexLocation);
  pointShader->enableAttributeArray (pointColorLocation);
  if (surfaceMode)
  {
    pointShader->setUniformValue (pointModelviewLocation, QMatrix4x4());
    pointShader->setAttributeArray (pointVertexLocation, &surfaceVertices [0], 3);
    pointShader->setAttributeArray (pointColorLocation, GL_UNSIGNED_BYTE, (void*)&surfaceColors [0], 3);
  }
  else
  {
    pointShader->setUniformValue (pointModelviewLocation, QMatrix4x4(pointTransformations[overlay].data()).transposed());
    pointShader->setAttributeArray (pointVertexLocation, &pointVertices [overlay][0], 3);
    pointShader->setAttributeArray (pointColorLocation, GL_UNSIGNED_BYTE, (void*)&pointColors [overlay][0], 3);
  }
  pointShader->setUniformValue (pointProjectionLocation, QMatrix4x4(projectionMatrices[image].data()).transposed());
  pointShader->setUniformValue (pointModelviewInvLocation, QMatrix4x4(imageTransformations[image].data()).transposed());
//  glEnable (GL_BLEND);
  glEnable(GL_DEPTH_TEST);
//  glBlendFunc(GL_DST_ALPHA, GL_SRC_ALPHA);
  if (surfaceMode)
  {
    glDrawElements (GL_TRIANGLES, surfaceIndices.size(), GL_UNSIGNED_INT, (GLvoid*)&surfaceIndices[0]);
  }
  else
  {
    glDrawElements (GL_TRIANGLES, indices[overlay].size(), GL_UNSIGNED_INT, (GLvoid*)&indices[overlay][0]);
  }
  glDisable (GL_DEPTH_TEST);
//  glDisable (GL_BLEND);
  pointShader->disableAttributeArray (pointVertexLocation);
  pointShader->disableAttributeArray (pointColorLocation);
  pointShader->release();
}

void
HighResViewerWidget::saveRange ()
{
  if (!pointLoaded[overlay])
  {
    loadPoints();
  }

  GLuint rangeDepthTexture;
  glGenTextures (1, &rangeDepthTexture);
  glBindTexture (GL_TEXTURE_2D, rangeDepthTexture);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, images[image].cols, images[image].rows, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
  glBindTexture (GL_TEXTURE_2D, 0);

  GLuint rangeColorTexture;
  glGenTextures (1, &rangeColorTexture);
  glBindTexture (GL_TEXTURE_2D, rangeColorTexture);
  glTexImage2D (GL_TEXTURE_2D, 0, GL_RGBA32F, images[image].cols, images[image].rows, 0, GL_RGBA, GL_FLOAT, 0);
  glBindTexture (GL_TEXTURE_2D, 0);

  GLuint fbo;
  glGenFramebuffers (1, &fbo);
  glBindFramebuffer (GL_FRAMEBUFFER, fbo);
  glFramebufferTexture2D (GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, rangeDepthTexture, 0);
  glFramebufferTexture2D (GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rangeColorTexture, 0);

  if (GL_FRAMEBUFFER_COMPLETE != glCheckFramebufferStatus (GL_FRAMEBUFFER))
  {
    std::cout << "FBO failure" << std::endl;
  }

  glViewport (0, 0, images[image].cols, images[image].rows);

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  
  rangeShader->bind();
  rangeShader->setUniformValue (rangeProjectionLocation, QMatrix4x4(projectionMatrices[image].data()).transposed());
  rangeShader->setUniformValue (rangeModelviewInvLocation, QMatrix4x4(imageTransformations[image].data()).transposed());
  if (surfaceMode)
  {
    rangeShader->setAttributeArray (rangeVertexLocation, &surfaceVertices[0], 3);
    rangeShader->setUniformValue (rangeModelviewLocation, QMatrix4x4());
  }
  else
  {
    rangeShader->setAttributeArray (rangeVertexLocation, &pointVertices [overlay][0], 3);
    rangeShader->setUniformValue (rangeModelviewLocation, QMatrix4x4(pointTransformations[overlay].data()).transposed());
  }
  rangeShader->enableAttributeArray (rangeVertexLocation);
  glEnable (GL_DEPTH_TEST);
  if (surfaceMode)
  {
    glDrawElements (GL_TRIANGLES, surfaceIndices.size(), GL_UNSIGNED_INT, (GLvoid*)&surfaceIndices[0]);
  }
  else
  {
    glDrawElements (GL_TRIANGLES, indices[overlay].size(), GL_UNSIGNED_INT, (GLvoid*)&indices[overlay][0]);
  }
  glDisable (GL_DEPTH_TEST);
  rangeShader->disableAttributeArray (rangeVertexLocation);
  rangeShader->release();

  glBindFramebuffer (GL_FRAMEBUFFER, 0);

  glBindTexture (GL_TEXTURE_2D, rangeColorTexture);

  cv::Mat output (images[image].rows, images[image].cols, CV_32FC1);
  cv::Mat outputFlipped (output.rows, output.cols, CV_32FC1);
  std::vector<GLfloat> data (images[image].rows*images[image].cols);
  glGetTexImage (GL_TEXTURE_2D, 0, GL_RED, GL_FLOAT, &data[0]);

  glBindTexture (GL_TEXTURE_2D, rangeDepthTexture);
  glGetTexImage (GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, output.data);
#if 0
  for (int j = 0; j < output.rows; ++j)
  {
    for (int i = 0; i < output.cols; ++i)
    {
      if (output.at<float>(j, i) < 1.0)
      {
        std::cout << j << " " << i << " " << output.at<float>(j, i) << std::endl;
        exit (0);
      }
    }
  }
#endif

#if 0
  for (int i = 0; i < data.size(); ++i)
  {
    if (data[i] > 0)
    { 
      std::cout << data[i] << " " << i << std::endl;
      exit(0);
    }
  }
#endif

  cv::flip (output, outputFlipped, 0);
  bool save = false;
  for (int j = 0; j < outputFlipped.rows; ++j)
  {
    for (int i = 0; i < outputFlipped.cols; ++i)
    {
//      int index = outputFlipped.cols*(outputFlipped.rows-1-j) + i;
//      float value = data[index] > 0 ? data[index] : 12.0;
#if 0
      if (data1 || data2 || data3 || data4)
      {
        std::cout << data1 << " " << data2 << " " << data3 << " " << data4 <<  " " << integer << " " << value << std::endl;
      }
#endif
      float value = outputFlipped.at<float>(j, i);
      if (!save && value < 1.0)
      {
        save = true;
      }
      outputFlipped.at<float>(j, i) = 1.0/(1.0 - 11.0/12.0*value);

#if 0
if (value > 0)
{
std::cout << i << " " << j << " " << outputFlipped.at<float>(j, i) << std::endl;
}
#endif
    }
  }
  if (save)
  {
    std::stringstream ss_raw;
    ss_raw << files[highres[image]] << "-" << (surfaceMode ? "surface" : files[lowres[overlay]]);
    std::string newFile = ss_raw.str();
    size_t pos;
    while ((pos = newFile.find_first_of("./")) != std::string::npos)
    {
      newFile.erase (pos, 1);
    }
    newFile += ".raw";
    std::cout << "Saving " << newFile << std::endl;
    std::ofstream outputFile (newFile.c_str());
    outputFile.write ((char*)&output.rows, sizeof(uint32_t));
    outputFile.write ((char*)&output.cols, sizeof(uint32_t));
    outputFile.write ((char*)outputFlipped.data, sizeof(float)*output.rows*output.cols);
    outputFile.close ();
  }

  glBindTexture (GL_TEXTURE_2D, 0);

  glDeleteFramebuffers (1, &fbo);
  glDeleteTextures (1, &rangeDepthTexture);
  glDeleteTextures (1, &rangeColorTexture);

  glViewport (0, 0, width(), height());
}

void
HighResViewerWidget::keyPressEvent (QKeyEvent * event)
{
  switch (event->key())
  {
    case Qt::Key_A:
      surfaceMode = !surfaceMode;
      repaint ();
      return;
    case Qt::Key_Z:
      displayImage = !displayImage;
      repaint();
      return;
    case Qt::Key_Up:
      if (overlay != (int)pointVertices.size() - 1)
      {
        ++overlay;
        repaint();
      }
      return;
    case Qt::Key_Down:
      if (overlay != 0)
      {
        --overlay;
        repaint();
      }
      return;
    case Qt::Key_Left:
      if (image != 0)
      {
        --image;
        repaint();
      }
      return;
    case Qt::Key_Right:
      if (image != (int)imageVertices.size() - 1)
      {
        ++image;
        repaint();
      }
      return;
    case Qt::Key_S:
      if (surfaceMode)
      {
        saveRangeFlag = true;
        repaint ();
      }
      else
      {
        for (overlay = 0; overlay < (int)pointVertices.size() - 1; ++overlay)
        {
          saveRangeFlag = true;
          repaint();
        }
      }
      return;
  }
}
