#include "QBezierCurve.h"

#include <QLineF>

QBezierCurve::QBezierCurve()
{

}

void QBezierCurve::clear()
{
  points_.clear();
  bezier_.clear();
}

void QBezierCurve::addPoint(const QPointF &point)
{
  points_.push_back(point);
}

const QVector<QBezierCurve::QBezier> & QBezierCurve::getBezier()
{
  return bezier_;
}

void QBezierCurve::calcBezier()
{
  const int size_point = points_.size();
  if (size_point < 2) {
    return;
  }
  const float factor = 0.13;
  for (int i = 0; i < size_point - 1; ++i) {
    QBezier bezier;
    bezier.start = points_[i];
    bezier.end = points_[i + 1];

    // control 1
    QLineF linef, linef_fit;
    if (i == 0) {
      linef.setP1(points_[0]);
      linef.setP2(points_[1]);
    }
    else {
      linef.setP1(points_[i - 1]);
      linef.setP2(points_[i + 1]);
    }
    linef_fit.setP1(points_[i]);
    linef_fit.setAngle(linef.angle());
    linef_fit.setLength(linef.length() * factor);
    bezier.control = linef_fit.p2();

    // control2
    if (i == size_point - 2) {
      linef.setP1(points_[size_point - 1]);
      linef.setP2(points_[size_point - 2]);
    }
    else {
      linef.setP1(points_[i + 2]);
      linef.setP2(points_[i]);
    }
    linef_fit.setP1(points_[i + 1]);
    linef_fit.setAngle(linef.angle());
    linef_fit.setLength(linef.length() * factor);
    bezier.control2 = linef_fit.p2();

    bezier_.push_back(bezier);
  }
}
