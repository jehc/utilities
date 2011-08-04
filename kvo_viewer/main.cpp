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

  QDockWidget * controlsDock = new QDockWidget ( &w, 0 );
  QWidget * controlsWidget = new QWidget (controlsDock);
  controlsDock->setWidget (controlsWidget);
  QLayout * controlsLayout = new QHBoxLayout (controlsWidget);
  controlsWidget->setLayout (controlsLayout);

  QWidget * sliceWidget = new QWidget (controlsWidget);
  QLayout * sliceLayout = new QVBoxLayout (sliceWidget);
  sliceWidget->setLayout (sliceLayout);
  QLabel * sliceLabel = new QLabel ("S\nl\ni\nc\ne", sliceWidget);
  sliceLabel->setAlignment (Qt::AlignHCenter);
  sliceLayout->addWidget (sliceLabel);
  QSlider * sliceBar = new QSlider ( Qt::Vertical, sliceWidget );
  sliceLayout->addWidget (sliceBar);
  sliceBar->setMinimum ( 0 );
  sliceBar->setMaximum ( 100 );
  sliceBar->setSliderPosition ( 100 );
  sliceBar->setTickPosition (QSlider::TicksRight);
  controlsLayout->addWidget ( sliceWidget );

  QWidget * scaleWidget = new QWidget (controlsWidget);
  QLayout * scaleLayout = new QVBoxLayout (scaleWidget);
  scaleWidget->setLayout (scaleLayout);
  QLabel * scaleLabel = new QLabel ("A\nl\np\nh\na\n \ns\nc\na\nl\ni\nn\ng", scaleWidget);
  scaleLabel->setAlignment (Qt::AlignHCenter);
  scaleLayout->addWidget (scaleLabel);
  QSlider * scaleBar = new QSlider ( Qt::Vertical, controlsWidget );
  scaleLayout->addWidget (scaleBar);
  scaleBar->setMinimum ( 10 );
  scaleBar->setMaximum ( 1000 );
  scaleBar->setSliderPosition ( 100 );
  scaleBar->setTickPosition (QSlider::TicksRight);
  controlsLayout->addWidget ( scaleWidget );

  QWidget * numSlicesWidget = new QWidget (controlsWidget);
  QLayout * numSlicesLayout = new QVBoxLayout (numSlicesWidget);
  numSlicesWidget->setLayout (numSlicesLayout);
  QLabel * numSlicesLabel = new QLabel ("Q\nu\na\nl\ni\nt\ny", numSlicesWidget);
  numSlicesLabel->setAlignment (Qt::AlignHCenter);
  numSlicesLayout->addWidget (numSlicesLabel);
  QSlider * numSlicesBar = new QSlider ( Qt::Vertical, controlsWidget );
  numSlicesLayout->addWidget (numSlicesBar);
  numSlicesBar->setMinimum ( 1 );
  numSlicesBar->setMaximum ( 10000 );
  numSlicesBar->setSliderPosition ( 1000 );
  controlsLayout->addWidget ( numSlicesWidget );

  QWidget * othersWidget = new QWidget (controlsWidget);
  QLayout * othersLayout = new QVBoxLayout (othersWidget);
  othersWidget->setLayout (othersLayout);
  QCheckBox * realColorsCheckbox = new QCheckBox ("Real colors", othersWidget);
  othersLayout->addWidget (realColorsCheckbox);
  QCheckBox * lightingCheckbox = new QCheckBox ("Lighting", othersWidget);
  othersLayout->addWidget (lightingCheckbox);
  controlsLayout->addWidget (othersWidget);

  w.setCentralWidget ( kvo );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfR );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfG );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfB );
  w.addDockWidget ( Qt::TopDockWidgetArea, tfA );
  w.addDockWidget ( Qt::LeftDockWidgetArea, controlsDock );

  QObject::connect ( tfR, SIGNAL ( transferFunctionChanged ( const QVector<double> & ) ), kvo,
    SLOT ( transferFunctionRChanged ( const QVector<double> & ) ) );
  QObject::connect ( tfG, SIGNAL ( transferFunctionChanged ( const QVector<double> & ) ), kvo,
    SLOT ( transferFunctionGChanged ( const QVector<double> & ) ) );
  QObject::connect ( tfB, SIGNAL ( transferFunctionChanged ( const QVector<double> & ) ), kvo,
    SLOT ( transferFunctionBChanged ( const QVector<double> & ) ) );
  QObject::connect ( tfA, SIGNAL ( transferFunctionChanged ( const QVector<double> & ) ), kvo,
    SLOT ( transferFunctionAChanged ( const QVector<double> & ) ) );
  QObject::connect ( sliceBar, SIGNAL ( valueChanged ( int ) ), kvo, SLOT ( sliceChanged ( int ) ) );
  QObject::connect ( scaleBar, SIGNAL ( valueChanged ( int ) ), kvo, SLOT ( scaleChanged ( int ) ) );
  QObject::connect (numSlicesBar, SIGNAL(valueChanged(int)), kvo, SLOT(numSlicesChanged (int)));
  QObject::connect (realColorsCheckbox, SIGNAL(stateChanged(int)), kvo, SLOT(realColorsChanged(int)));
  QObject::connect (lightingCheckbox, SIGNAL(stateChanged(int)), kvo, SLOT(lightingChanged(int)));

  a.setActiveWindow ( &w );
  w.show ();

  return a.exec ();
}
