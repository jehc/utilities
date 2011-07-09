#ifndef HIGHRESVIEWER_WIDGET_H
#define HIGHRESVIEWER_WIDGET_H

#include <QtOpenGL>

#include "BundleFile.h"

#include <string>

#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>

class HighResViewerWidget : public QGLWidget
{
std::string bundleFile;
std::string listFile;
std::string tuningFile;
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
std::vector<std::vector<GLbyte> > pointColors;
std::vector<std::vector<GLfloat> > imageVertices;
std::vector<GLfloat> imageTexcoords;
int image;
int overlay;
bool displayImage;

QGLShaderProgram * pointShader;
int pointVertexLocation;
int pointColorLocation;
int pointProjectionLocation;
int pointModelviewLocation;
int pointModelviewInvLocation;

QGLShaderProgram * imageShader;
GLuint imageTexture;
int imageVertexLocation;
int imageTexcoordLocation;
int imageTexLocation;
int imageProjectionLocation;

void setPoints();
void setImage();
void loadImage();
void loadPoints();

public:
HighResViewerWidget (QWidget *, const std::string &, const std::string &, const std::string &);
void load();
void initializeGL();
void resizeGL (int, int);
void paintGL();
void keyPressEvent (QKeyEvent *);
};

#endif
