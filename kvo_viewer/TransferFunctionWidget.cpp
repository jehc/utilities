#include "TransferFunctionWidget.h"

#include <cassert>
#include <iostream>

TransferFunctionWidget::TransferFunctionWidget ( QWidget * parent, const QColor & color )
  : QDockWidget ( parent )
  , color ( color )
{
  QPolygonF points;

  points << QPointF ( 0, sizeHint ().height () )
         << QPointF ( sizeHint ().width (), 0 );

  hoverPoints = new HoverPoints ( this, HoverPoints::CircleShape );
  hoverPoints->setPoints ( points );
  hoverPoints->setPointLock ( 0, HoverPoints::LockToLeft );
  hoverPoints->setPointLock ( 1, HoverPoints::LockToRight );
  hoverPoints->setSortType ( HoverPoints::XSort );
  hoverPoints->setConnectionType ( HoverPoints::LineConnection );
  setSizePolicy ( QSizePolicy::Preferred, QSizePolicy::Fixed );
  connect ( hoverPoints, SIGNAL ( pointsChanged ( QPolygonF ) ), this, SLOT ( transferFunctionChanged () ) );
  transferFunction.resize ( 256 );
  transferFunctionChanged ();
  setFeatures ( QDockWidget::DockWidgetMovable );
}

void
TransferFunctionWidget::contextMenuEvent ( QContextMenuEvent * )
{
}

void
TransferFunctionWidget::paintEvent ( QPaintEvent * )
{
  QPainter p ( this );
  QLinearGradient gradient ( 0, 0, width (), 0 );

  gradient.setColorAt ( 0, Qt::black );
  gradient.setColorAt ( 1, color );
  p.setBrush ( gradient );
  p.setPen ( QColor ( 146, 146, 146 ) );
  p.drawRect ( 0, 0, width () - 1, height () - 1 );
}

void
TransferFunctionWidget::transferFunctionChanged ()
{
  resize ( width (), height () );
  int index = 0;
  const QPolygonF points = hoverPoints->points ();
  assert ( points.size () >= 2 );
  for ( int i = 0; i < 255; ++i )
  {
    float position = i * size ().width () / 255.0;
    while ( points.at ( index ).x () <= position )
    {
      ++index;
      if ( index >= points.size () )
      {
        std::cout << "Somethings wrong" << std::endl;
        return;
      }
    }
    assert ( index > 0 && index < points.size () );
    const QPointF & point1 = points.at ( index - 1 );
    const QPointF & point2 = points.at ( index );
    transferFunction [i] = 1 -
                           ( point1.y () +
     ( position - point1.x () ) * ( point2.y () - point1.y () ) / ( point2.x () - point1.x () ) ) / height ();
  }

  emit transferFunctionChanged ( transferFunction );
}
