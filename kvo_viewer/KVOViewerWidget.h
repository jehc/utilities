#ifndef KVO_VIEWER_WIDGET_H
#define KVO_VIEWER_WIDGET_H

#include <QtGui>
#include <QtOpenGL>
#include <qglview.h>
#include <string>

#include "kvo.h"

class KVOViewerWidget : public QGLView
{
  Q_OBJECT

  std::string filename;

  GLuint voxelTexture;
  GLuint transferFunctionTexture;

  QGLShaderProgram * shaderProgram;

  Eigen::Vector3f aspect;

  float voxelSize;
  float slice;
  float scale;

  std::vector<float> transferFunction;

  int vertexLocation;
  int textureLocation;
  int transferFunctionLocation;
  int samplesLocation;
  int projectionMatrixLocation;
  int texCoordLocation; 
  int scaleLocation;
  int sliceLocation; 

  void initializeTexture (KVO &);
  void initializeShaders (QGLPainter *);
  void transferFunctionChanged ();

class IntersectionCompare
{
  public:
  bool operator () (const std::pair<float, Eigen::Vector4f> & a, const std::pair<float, Eigen::Vector4f> & b)
  {
    return a.first < b.first;
  }
} comp;

public:
  KVOViewerWidget (const QGLFormat &, QWidget *, const std::string &);

protected:
  void initializeGL(QGLPainter *);
  void resizeGL (int, int);
  void paintGL (QGLPainter *);
  QSize sizeHint() const { return QSize (1024, 768); }

public slots:
  void transferFunctionRChanged (const QVector<float> &);
  void transferFunctionGChanged (const QVector<float> &);
  void transferFunctionBChanged (const QVector<float> &);
  void transferFunctionAChanged (const QVector<float> &);
  void scaleChanged (int);
  void sliceChanged (int);
};

#endif
