#include "HighResViewerWidget.h"

#include "ply_io.h"

#include <pcl/point_types.h>

#include <fstream>

HighResViewerWidget::HighResViewerWidget (QWidget * parent, const std::string & bundleFile, const std::string & listFile, const std::string & tuningFile)
: QGLWidget (parent)
, bundleFile (bundleFile)
, listFile (listFile)
, tuningFile (tuningFile)
, image (0)
, overlay (0)
, displayImage (true)
{
  setFocusPolicy (Qt::ClickFocus);
  setFixedSize (1024, 768);
}

void
HighResViewerWidget::load ()
{
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
  GLdouble zNear = 1;
  GLdouble zFar = 100.0;
  GLdouble focal = cameras[highres[image]].GetF();
  GLdouble f = 1.0/tan(fovy/2.0);
  projectionMatrices [image] << f/aspect, 0, 0, 0,
                  0, f, 0, 0,
                  0, 0, (zFar + zNear)/(zNear - zFar), 2*zFar*zNear/(zNear - zFar),
                  0, 0, -1, 0;
  Eigen::Matrix3f K;
  K << cameras[highres[image]].GetF(), 0, 0,
         0, cameras[highres[image]].GetF(), 0,
         0, 0, 1;
  Eigen::Matrix3f Kinv = K.inverse();
  Eigen::Vector3f a (-images[image].cols/2/focal, -images[image].rows/2/focal, -1);
  Eigen::Vector3f b (-images[image].cols/2/focal, images[image].rows/2/focal, -1);
  Eigen::Vector3f c (images[image].cols/2/focal, images[image].rows/2/focal, -1);
  Eigen::Vector3f d (images[image].cols/2/focal, -images[image].rows/2/focal, -1);
  imageVertices [image].resize (12);
  for (int i = 0; i < 3; ++i)
  {
    imageVertices [image][i] = a[i];
    imageVertices [image][3+i] = b[i];
    imageVertices [image][6+i] = c[i];
    imageVertices [image][9+i] = d[i];
  }
//  setFixedSize (images[image].cols, images[image].rows);
  imageLoaded [image] = true;
}

void
HighResViewerWidget::loadPoints()
{
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

  for (size_t i = 0; i < points.points.size(); ++i)
  {
    if (std::isnan(points.points[i].x) || std::isnan(points.points[i].y) || std::isnan(points.points[i].z))
    {
      continue;
    }
    pointVertices [overlay].push_back (points.points [i].x);
    pointVertices [overlay].push_back (points.points [i].y);
    pointVertices [overlay].push_back (points.points [i].z);
    ByteExtractor c;
    c.rgba = points.points [i].rgba;
    pointColors [overlay].push_back (c.b[0]);
    pointColors [overlay].push_back (c.b[1]);
    pointColors [overlay].push_back (c.b[2]);
  }

  pointLoaded[overlay] = true;
}

void
HighResViewerWidget::initializeGL()
{

  if (!imageLoaded.size() && !pointLoaded.size())
  {
    load();
  }

  glClearColor (0, 0, 0, 1);

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
    "void\n"
    "main()\n"
    "{\n"
    "  gl_Position = projection*modelviewinv*modelview*vec4(vertex, 1.0);\n"
    "  gl_FrontColor = vec4(color, 1.0/(pow(gl_Position.z+1.0,2.0)));\n"
    "  gl_Position.y = -gl_Position.y;\n"
    "  float size = 1.0/pow(gl_Position.z + 1.0,2.0);\n"
    "  gl_PointSize = size > 5.0 ? 5.0 : size;\n"
    "}\n";

  const char * PointFShader = 
    "void\n"
    "main()\n"
    "{\n"
    "  gl_FragColor = gl_Color;\n"
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
    "  gl_Position.y = -gl_Position.y;\n"
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

  glEnable (GL_BLEND);
  glEnable (GL_VERTEX_PROGRAM_POINT_SIZE);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  QGLWidget::initializeGL ();
}

void
HighResViewerWidget::setPoints()
{
  if (!pointLoaded [overlay])
  {
    loadPoints();
  }
  pointShader->setUniformValue (pointModelviewLocation, QMatrix4x4(pointTransformations[overlay].data()).transposed());
  pointShader->setUniformValue (pointProjectionLocation, QMatrix4x4(projectionMatrices[image].data()).transposed());
  pointShader->setAttributeArray (pointVertexLocation, &pointVertices [overlay][0], 3);
  pointShader->setAttributeArray (pointColorLocation, GL_UNSIGNED_BYTE, (void*)&pointColors [overlay][0], 3);
  pointShader->setUniformValue (pointModelviewInvLocation, QMatrix4x4(imageTransformations[image].data()).transposed());
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
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
/*
  std::cout << (projectionMatrices[image]*Eigen::Vector4d(imageVertices[image][0],imageVertices[image][1],imageVertices[image][2],1)).transpose() << std::endl;
  std::cout << (projectionMatrices[image]*Eigen::Vector4d(imageVertices[image][3],imageVertices[image][4],imageVertices[image][5],1)).transpose() << std::endl;
  std::cout << (projectionMatrices[image]*Eigen::Vector4d(imageVertices[image][6],imageVertices[image][7],imageVertices[image][8],1)).transpose() << std::endl;
  std::cout << (projectionMatrices[image]*Eigen::Vector4d(imageVertices[image][9],imageVertices[image][10],imageVertices[image][11],1)).transpose() << std::endl;
*/
  imageShader->bind();
  setImage();
  imageShader->enableAttributeArray (imageVertexLocation);
  imageShader->enableAttributeArray (imageTexcoordLocation);
  if (displayImage)
  {
    glDrawArrays (GL_QUADS, 0, 4);
  }
  imageShader->disableAttributeArray (imageVertexLocation);
  imageShader->disableAttributeArray (imageTexcoordLocation);
  imageShader->release();

  pointShader->bind();
  setPoints();
  pointShader->enableAttributeArray (pointVertexLocation);
  pointShader->enableAttributeArray (pointColorLocation);
//  std::cout << (projectionMatrices[image]*pointTransformations[overlay]*imageTransformations[image]*Eigen::Vector4d(pointVertices[overlay][0], pointVertices[overlay][1], pointVertices[overlay][2], 1)).transpose() << std::endl;
  glDrawArrays (GL_POINTS, 0, pointVertices[overlay].size()/3);
  pointShader->disableAttributeArray (pointVertexLocation);
  pointShader->disableAttributeArray (pointColorLocation);
  pointShader->release();
}

void
HighResViewerWidget::keyPressEvent (QKeyEvent * event)
{
  switch (event->key())
  {
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
  }
}
