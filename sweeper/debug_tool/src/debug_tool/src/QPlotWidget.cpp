#include <QPainter>
#include <QtMath>
#include "QPlotWidget.h"

QPlotWidget::QPlotWidget(QWidget *parent)
  :QWidget(parent)
{

}

void QPlotWidget::setName(const QString &title, const QString &x, const QString &y)
{
  m_strTitle = title;
  m_strXName = x;
  m_strYName = y;
}

void QPlotWidget::resizeEvent(QResizeEvent *)
{
  this->plot();
}

void QPlotWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.drawImage(0, 0, m_image);
}

void QPlotWidget::clearPoints()
{
  m_points.clear();
}

void QPlotWidget::addPoint(QSharedPointer<QPointF> point)
{
  const int size_point = m_points.size();
  if (size_point == 0 || point->x() > m_points.back()->x()) {
    m_points.push_back(point);
  }
  else {
    for (int i = 0; i < size_point; ++i) {
      if (point->x() < m_points[i]->x()) {
        m_points.insert(i, point);
        break;
      }
    }
  }
}

void QPlotWidget::plot()
{
  QRect rect = this->rect();
  m_image = QImage(rect.width(), rect.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(230, 230, 230));

  // title, x name, y name
  constexpr int text_h = 18;
  QRect rect_title = QRect(0, 0, rect.width(), text_h);

  QRect rect_curve = QRect(
        text_h * 2, text_h, rect.width() - text_h * 2, rect.height() - text_h * 2);
  rect_curve.adjust(0, 0, -2, 0);
  if (rect_curve.width() <= 5 || rect_curve.height() <= 5) {
    return;
  }

  painter.setFont(QFont("Times", 12));
  painter.drawText(rect_title, Qt::AlignCenter, m_strTitle);

  // axis
  int x_size = 5;
  double x_min = 0;
  double x_max = 100;
  int y_size = 6;
  double y_min = 0;
  double y_max = 2.0;
  const size_t size_pts = m_points.size();
  if (size_pts > 0) {
    x_min = m_points[0]->x();
    x_max = m_points[0]->x();
    y_min = m_points[0]->y();
    y_max = m_points[0]->y();

    for (const auto &ptf : m_points) {
      if (x_min > ptf->x()) x_min = ptf->x();
      if (x_max < ptf->x()) x_max = ptf->x();
      if (y_min > ptf->y()) y_min = ptf->y();
      if (y_max < ptf->y()) y_max = ptf->y();
    }
    x_max += 1;
  }

  // x axis range
  double x_unit = (x_max - x_min) / x_size;
  int bit = 0;
  int bound = (int)x_unit;
  while (bound < 10) {
    ++ bit;
    bound = (int)(x_unit * qPow(10, bit));
  }
  int bound_mod = (bound % 10);
  if (bound_mod > 0 && bound_mod < 5) {
    bound = bound / 10 * 10 + 5;
  }
  else if (bound_mod > 5) {
    bound = bound / 10 * 10 + 10;
  }
  x_unit = double(bound) / qPow(10, bit);
  x_min = x_unit * (int(x_min / x_unit));
  x_size = qCeil((x_max - x_min) / x_unit);
  x_max = x_min + x_unit * x_size;

  // y axis range
  double y_unit = (y_max - y_min) / y_size;
  if (y_unit > 1e-4) {
    bit = 0;
    bound = (int)y_unit;
    while (bound < 10) {
      ++ bit;
      bound = (int)(y_unit * qPow(10, bit));
    }
    bound_mod = (bound % 10);
    if (bound_mod > 0 && bound_mod < 5) {
      bound = bound / 10 * 10 + 5;
    }
    else if (bound_mod > 5) {
      bound = bound / 10 * 10 + 10;
    }
    y_unit = double(bound) / qPow(10, bit);
    y_min = y_unit * (int(y_min / y_unit) - 1);
    y_size += 2;
    y_max = y_min + y_unit * y_size;
  }

  painter.setPen(Qt::black);
  painter.drawRect(rect_curve);

  QPen pen;
  pen.setWidth(1);
  pen.setStyle(Qt::DashLine);
  painter.setPen(pen);
  double x_pix_unit = rect_curve.width() / x_size;
  double y_pix_unit = rect_curve.height() / y_size;
  for (int i = 1; i < x_size; ++i) {
    QLineF linef(
          rect_curve.bottomLeft().x() + i * x_pix_unit,
          rect_curve.bottomLeft().y(),
          rect_curve.topLeft().x() + i * x_pix_unit,
          rect_curve.topLeft().y()
          );
    painter.drawLine(linef);
    painter.drawText(linef.p1() + QPointF(0, text_h),
                     QString::number(x_min + i * x_unit));
  }
  for (int i = 1; i < y_size; ++i) {
    QLineF linef(
          rect_curve.topLeft().x(),
          rect_curve.topLeft().y() + i * y_pix_unit,
          rect_curve.topRight().x(),
          rect_curve.topRight().y() + i * y_pix_unit
          );
    painter.drawLine(linef);
    painter.drawText(linef.p1() - QPointF(text_h * 2, 0),
                     QString::number(y_min + (y_size - i) * y_unit));
  }

  // curve
  QPolygonF pgf;
  for (const auto &pt : m_points) {
    QPointF ptf = QPoint(
          rect_curve.x() + (pt->x() - x_min) / (x_unit * x_size) * rect_curve.width(),
          rect_curve.y() + (y_max - pt->y()) / (y_unit * y_size) * rect_curve.height()
          );
    pgf << ptf;
  }
  if (pgf.size() > 0) {
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawPolyline(pgf);
  }

  this->update();
}

