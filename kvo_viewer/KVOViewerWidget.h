#include <QtGui>
#include <QtOpenGL>

class KVOViewerWidget : public QGLWidget
{
protected:
  void initializeGL();
  void resizeGL (int, int);
  void paintGL ();
  void keyPressEvent (QKeyEvent *);
};
