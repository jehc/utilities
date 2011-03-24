#ifndef REPROJECT_DEPTH_WIDGET_H
#define REPROJECT_DEPTH_WIDGET_H

#include <QGLWidget>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

class ReprojectDepthWidget : public QGLWidget
{
Q_OBJECT
bool init;
int depthPatches;

cv::Mat rgb_intrinsics;
cv::Mat rgb_distortion;
cv::Mat rgb_intrinsics_corrected;

cv::Mat depth_intrinsics;
cv::Mat depth_distortion;
cv::Mat depth_intrinsics_shifted;

cv::Mat R;
cv::Mat T;

cv::Mat rgb_size;
cv::Mat depth_size;

cv::Mat raw_rgb_size;
cv::Mat raw_depth_size;

cv::Mat depth_base_and_offset;

GLubyte * depthColors;
GLfloat * depthVertices;
double maxDepth;
double minDepth;

GLuint depthProgram;
GLuint depthVertexAttribute;
GLuint depthColorAttribute;
GLuint modelview_uniform;
GLuint projection_uniform;

std::vector<std::string> captures;
std::vector<std::string>::iterator currentIter, captureIter;

protected:
void initializeGL ();
void savePly();
void resizeGL ( int, int );
void paintGL ();
void loadDepthMap ( );
void loadColorImage ();
void loadManifest ( const std::string & );
void loadCalibParameters ( const std::string & );
float convertToDepth ( float );
void loadPair ();
void keyPressEvent ( QKeyEvent * );

public:
ReprojectDepthWidget ( const std::string &, const std::string & );
~ReprojectDepthWidget();

public slots:
void nextImage();
};

#endif
