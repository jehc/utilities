#include <QApplication>
#include "ReprojectDepthWidget.h"
#include <iostream>
#include <QTimer>

int
main ( int argc, char * * argv )
{
  if ( argc < 3 )
  {
    std::cerr << "Usage: " << argv[0] << " <camera_calib> <image_pairs>" <<
    std::endl;
    return -1;
  }
  QApplication app ( argc, argv );
  std::string calibFilename ( argv[1] );
  std::string imagePairsFilename ( argv[2] );
  ReprojectDepthWidget widget ( calibFilename, imagePairsFilename );
  QTimer timer;
  timer.setInterval (0);
  QObject::connect (&timer, SIGNAL(timeout()), &widget, SLOT(nextImage()));
  app.setActiveWindow ( &widget );
  widget.show ();
  if (argc == 4 && !strcmp (argv[3], "auto"))
  {
    timer.start();
  }
  return app.exec ();
}
