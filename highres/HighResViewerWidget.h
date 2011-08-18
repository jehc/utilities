#ifndef HIGHRESVIEWER_WIDGET_H
#define HIGHRESVIEWER_WIDGET_H

#include <QtOpenGL>

#include "BundleFile.h"

#include <string>
#include <queue>

#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

#include <omp.h>

class HighResViewerWidget : public QGLWidget
{
omp_lock_t points_lock;
std::string bundleFile;
std::string listFile;
std::string tuningFile;
std::string surfaceFile;
std::vector<BundleCamera> cameras;
std::vector<size_t> highres;
std::vector<size_t> lowres;
std::vector<std::string> files;
std::vector<bool> imageLoaded;
std::vector<bool> pointLoaded;
std::vector<Eigen::Matrix4d> projectionMatrices;
std::vector<Eigen::Affine3d> imageTransformations;
std::vector<Eigen::Affine3d> pointTransformations;
std::vector<cv::Mat> images;
std::vector<float> focalLengths;
std::vector<std::vector<GLfloat> > pointVertices;
std::vector<std::vector<GLubyte> > pointColors;
std::vector<std::vector<GLfloat> > imageVertices;
std::vector<GLfloat> surfaceVertices;
std::vector<GLuint> surfaceIndices;
std::vector<GLubyte> surfaceColors;
std::vector<GLfloat> imageTexcoords;
std::vector<std::vector<GLuint> > indices;
std::queue<int> pointsCached;
int image;
int overlay;
bool displayImage;
bool saveRangeFlag;
bool surfaceMode;

QGLShaderProgram * pointShader;
int pointVertexLocation;
int pointColorLocation;
int pointProjectionLocation;
int pointModelviewLocation;
int pointModelviewInvLocation;

QGLShaderProgram * rangeShader;
int rangeVertexLocation;
int rangeProjectionLocation;
int rangeModelviewLocation;
int rangeModelviewInvLocation;

QGLShaderProgram * imageShader;
GLuint imageTexture;
int imageVertexLocation;
int imageTexcoordLocation;
int imageTexLocation;
int imageProjectionLocation;

void setPoints();
void setImage();
void loadImage();
void loadPoints(int);
void saveRange();

public:
HighResViewerWidget (QWidget *, const std::string &, const std::string &, const std::string &, const std::string &);
void load();
void initializeGL();
void resizeGL (int, int);
void paintGL();
void keyPressEvent (QKeyEvent *);
};

#endif
