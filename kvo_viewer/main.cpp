#include <QtGui>
#include <QtOpenGL>

#include <iostream>

#include "KVOViewerWidget.h"
#include "TransferFunctionWidget.h"

int
main ( int argc, char * * argv )
{
  if ( argc != 2 )
  {
    std::cout << "Usage: " << argv[0] << " [filename].kvo" << std::endl;
    exit ( 1 );
  }

  QApplication a ( argc, argv );

  QMainWindow w;
  KVOViewerWidget * kvo = new KVOViewerWidget ( QGLFormat ( QGL::SampleBuffers ), &w, argv [1] );

  TransferFunctionWidget * tfA = new TransferFunctionWidget ( &w, Qt::gray );
  TransferFunctionWidget * tfR = new TransferFunctionWidget ( &w, Qt::red );
  TransferFunctionWidget * tfG = new TransferFunctionWidget ( &w, Qt::green );
  TransferFunctionWidget * tfB = new TransferFunctionWidget ( &w, Qt::blue );

  QDockWidget * sliceDock = new QDockWidget ( &w, 0 );
  sliceDock->setFeatures ( QDockWidget::NoDockWidgetFeatures );
  QScrollBar * sliceBar = new QScrollBar ( Qt::Vertical, sliceDock );
  sliceBar->setMinimum ( 0 );
  sliceBar->setMaximum ( 100 );
  sliceBar->setSliderPosition ( 0 );
  sliceDock->setWidget ( sliceBar );

  QDockWidget * scaleDock = new QDockWidget ( &w, 0 );
  scaleDock->setFeatures ( QDockWidget::NoDockWidgetFeatures );
  QScrollBar * scaleBar = new QScrollBar ( Qt::Horizontal, scaleDock );
  scaleBar->setMinimum ( 10 );
  scaleBar->setMaximum ( 1000 );
  scaleBar->setSliderPosition ( 100 );
  scaleDock->setWidget ( scaleBar );

  w.setCentralWidget ( kvo );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfR );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfG );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfB );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfA );
  w.addDockWidget ( Qt::LeftDockWidgetArea, sliceDock );
  w.addDockWidget ( Qt::BottomDockWidgetArea, scaleDock );

  QObject::connect ( tfR, SIGNAL ( transferFunctionChanged ( const QVector<float> & ) ), kvo,
    SLOT ( transferFunctionRChanged ( const QVector<float> & ) ) );
  QObject::connect ( tfG, SIGNAL ( transferFunctionChanged ( const QVector<float> & ) ), kvo,
    SLOT ( transferFunctionGChanged ( const QVector<float> & ) ) );
  QObject::connect ( tfB, SIGNAL ( transferFunctionChanged ( const QVector<float> & ) ), kvo,
    SLOT ( transferFunctionBChanged ( const QVector<float> & ) ) );
  QObject::connect ( tfA, SIGNAL ( transferFunctionChanged ( const QVector<float> & ) ), kvo,
    SLOT ( transferFunctionAChanged ( const QVector<float> & ) ) );
  QObject::connect ( sliceBar, SIGNAL ( valueChanged ( int ) ), kvo, SLOT ( sliceChanged ( int ) ) );
  QObject::connect ( scaleBar, SIGNAL ( valueChanged ( int ) ), kvo, SLOT ( scaleChanged ( int ) ) );

  a.setActiveWindow ( &w );
  w.show ();

  return a.exec ();
}
