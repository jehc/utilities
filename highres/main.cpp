#include <QtGui>

#include <iostream>

#include "HighResViewerWidget.h"

int
main ( int argc, char * * argv )
{
  if ( argc != 4 )
  {
    std::cout << "Usage: " << argv[0] << " [bundle.out] [list.txt] [tuning]" << std::endl;
    exit ( 1 );
  }

  QApplication a ( argc, argv );

  QMainWindow w;
  HighResViewerWidget * viewer = new HighResViewerWidget ( &w, argv [1], argv[2] , argv[3]);

  w.setCentralWidget (viewer );

  a.setActiveWindow ( &w );
  w.show ();

  return a.exec ();
}
