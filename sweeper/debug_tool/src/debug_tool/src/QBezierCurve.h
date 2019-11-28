#ifndef QBEZIERCURVE_H
#define QBEZIERCURVE_H

#include <QPointF>
#include <QVector>

class QBezierCurve
{
public:
  struct QBezier
  {
    QPointF start;
    QPointF control;
    QPointF control2;
    QPointF end;
  };

public:
  QBezierCurve();

  void addPoint(const QPointF &);
  const QVector<QBezier> & getBezier();
  void calcBezier();
  void clear();

private:
  QVector<QPointF> points_;
  QVector<QBezier> bezier_;
};

#endif // QBEZIERCURVE_H
