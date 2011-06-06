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

int numSlices;

GLuint voxelTexture;
GLuint transferFunctionTexture;

QGLShaderProgram * shaderProgram;

Eigen::Vector3d aspect;
Eigen::Vector3d fudge;

double voxelSize;
double slice;
double scale;
int realColors;

std::vector<GLfloat> transferFunction;

int vertexLocation;
int textureLocation;
int transferFunctionLocation;
int samplesLocation;
int projectionMatrixLocation;
int texCoordLocation;
int scaleLocation;
int sliceLocation;
int realColorsLocation;

void initializeTexture ( KVO & );
void initializeShaders ( QGLPainter * );
void transferFunctionChanged ();

class IntersectionCompare
{
public:
bool operator () ( const std::pair<double, Eigen::Vector4d> & a, const std::pair<double, Eigen::Vector4d> & b )
{
  return a.first < b.first;
}
} comp;

public:
KVOViewerWidget ( const QGLFormat &, QWidget *, const std::string & );

protected:
void initializeGL ( QGLPainter * );
void resizeGL ( int, int );
void paintGL ( QGLPainter * );
QSize sizeHint () const
{
  return QSize ( 1024, 768 );
}

public slots:
void transferFunctionRChanged ( const QVector<double> & );
void transferFunctionGChanged ( const QVector<double> & );
void transferFunctionBChanged ( const QVector<double> & );
void transferFunctionAChanged ( const QVector<double> & );
void scaleChanged ( int );
void sliceChanged ( int );
void numSlicesChanged (int);
void realColorsChanged (int);
};

#endif
