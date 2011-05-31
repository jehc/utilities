#ifndef TRANSFER_FUNCTION_WIDGET_H
#define TRANSFER_FUNCTION_WIDGET_H

#include <QtGui>

#include "hoverpoints.h"

class TransferFunctionWidget : public QDockWidget
{
  Q_OBJECT

  HoverPoints * hoverPoints;
  QVector<float> transferFunction;  
  QColor color;

  void paintEvent(QPaintEvent *);
  void contextMenuEvent (QContextMenuEvent *);
 
public:
  TransferFunctionWidget (QWidget *, const QColor &);

public slots:
  void transferFunctionChanged ();

signals:
  void transferFunctionChanged (const QVector<float> &);
};

#endif
