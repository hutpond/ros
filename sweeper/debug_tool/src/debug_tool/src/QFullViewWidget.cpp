#include <QtMath>
#include <QPainter>
#include <QPen>
#include <QMouseEvent>
#include "QFullViewWidget.h"

struct MapPoint
{
  quint64 index;
  double x;
  double y;
};

QFullViewWidget::QFullViewWidget(QWidget *parent)
  : QBaseShowWidget(parent)
{
  m_fOriginRatio = 1.2;
  m_fDisplayRatio = m_fOriginRatio;
  m_ptfTranslate = QPointF(0, 0);
}

void QFullViewWidget::mousePressEvent(QMouseEvent *e)
{
  m_ptfMouseMove = e->localPos();
  QTransform inverted = m_transform.inverted();
  QPointF ptf = inverted.map(m_ptfMouseMove);
  m_funPosition(ptf.x(), ptf.y(), 0, 0);
}

void QFullViewWidget::clearMapDatas()
{
  m_listReferences.clear();
  m_listVehicleLine.clear();
}

void QFullViewWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data,
                                      const QString &name, bool update)
{
  quint64 index = this->nameToIndex(name);
  this->addReference(data, index);
  this->addVehicleLine(data, index);

  if (update) {
    m_planningData = data;
    this->doUpdate(true);
  }
}

quint64 QFullViewWidget::nameToIndex(const QString &name)
{
  quint64 index = 0;
  if (name.isEmpty()) {
    return index;
  }

  int pos = name.indexOf('.');
  QString pre_name = name.mid(0, pos);
  QStringList list = pre_name.split('_');
  if (list.size() != 3) {
    return index;
  }
  // day
  index = list[0].mid(6).toULongLong() * qPow(10, 9);
  // h m s
  index += list[1].toULongLong() * qPow(10, 3);
  // ms
  index += list[2].toULongLong();

  return index;
}

int QFullViewWidget::findIndexPos(const QList<QSharedPointer<MapPoint>>& points, quint64 index)
{
  int low = 0;
  int heigh = points.size() - 1;
  int pos = -1;
  while (heigh - low > 1) {
    int pos = (heigh + low) / 2;
    if (index < points[pos]->index) {
      heigh = pos;
    }
    else if (index > points[pos]->index) {
      low = pos;
    }
    else {
      pos = -1;
      break;
    }
  }
  if (heigh - low == 1) {
    if (index == points[low]->index || index == points[heigh]->index) {
      pos = -1;
    }
    else {
      pos = low;
    }
  }
  return pos;
}

/*******************************************************
 * @brief 创建Setting对应的ToolBar
 * @param

 * @return
********************************************************/
int QFullViewWidget::isIndexValid(const QList<QSharedPointer<MapPoint>> &points, quint64 index)
{
  int ret = 0;
  auto size = points.size();
  if (size == 0) {
    ret = 1;
  }
  else if (points[0]->index == 0 && index == 0) {
    ret = 1;
  }
  else if (points[0]->index == 0 || index == 0) {
    ;
  }
  else if (index > points[size - 1]->index){
    ret = 1;
  }
  else if (index < points[0]->index){
    ret = 1;
  }
  else {
    int pos = this->findIndexPos(points, index);
    if (pos >= 0) {
      ret = 1;
    }
  }
  return ret;
}

double QFullViewWidget::xLocal2Global(const debug_tool::ads_PlanningData4Debug &data, double x)
{
  double global = data.vehicle_x + x;
  return global;
}

double QFullViewWidget::yLocal2Global(const debug_tool::ads_PlanningData4Debug &data, double y)
{
  double global = data.vehicle_y + y;
  return global;
}

void QFullViewWidget::addReference(const debug_tool::ads_PlanningData4Debug &data, quint64 index)
{
  QSharedPointer<MapPoint> reference;
  reference.reset(new MapPoint);
  reference->index = index;
  reference->x = this->xLocal2Global(data, data.reference_points[0].x);
  reference->y = this->yLocal2Global(data, data.reference_points[0].y);

  if (this->isIndexValid(m_listReferences, index) == 1) {
    m_listReferences.append(reference);
  }
}

void QFullViewWidget::addVehicleLine(const debug_tool::ads_PlanningData4Debug &data,
                                     quint64 index)
{
  QSharedPointer<MapPoint> vehicle;
  vehicle.reset(new MapPoint);
  vehicle->index = index;
  vehicle->x = data.vehicle_x;
  vehicle->y = data.vehicle_y;

  if (this->isIndexValid(m_listVehicleLine, index) == 1) {
    m_listVehicleLine.append(vehicle);
  }
}

void QFullViewWidget::calcMapRect()
{
  double x_min = 10000, x_max = -10000;
  double y_min = 10000, y_max = -10000;
  if (m_listReferences.size() == 0) {
    x_min = -1;
    x_max = 1;
    y_min = -1;
    y_max = 1;
  }
  else {
    for (const auto &pt : m_listReferences) {
      if (x_min > pt->x) x_min = pt->x;
      if (x_max < pt->x) x_max = pt->x;
      if (y_min > pt->y) y_min = pt->y;
      if (y_max < pt->y) y_max = pt->y;
    }
  }
  double map_width = x_max - x_min;
  double map_height = y_max - y_min;
  if (map_width < 1) map_width = 1;
  if (map_height < 1) map_height = 1;

  if (map_width / map_height < (double)m_rectPicture.width() / (double)m_rectPicture.height()) {
    map_width = map_height * (double)m_rectPicture.width() / (double)m_rectPicture.height();
  }
  else {
    map_height = map_width * (double)m_rectPicture.height() / (double)m_rectPicture.width();
  }
  map_width *= m_fDisplayRatio;
  map_height *= m_fDisplayRatio;

  double map_x = (x_min + x_max) / 2.0 - map_width / 2.0 - m_ptfTranslate.x();
  double map_y = (y_min + y_max) / 2.0 - map_height / 2.0 - m_ptfTranslate.y();
  m_rectfMap = QRectF(map_x, map_y, map_width, map_height);

  // 坐标转换
  m_transform.reset();
  m_transform.rotate(180, Qt::XAxis);
  m_transform.scale((double)m_rectPicture.width() / m_rectfMap.width(),
                    (double)m_rectPicture.height() / m_rectfMap.height());
  m_transform.translate(-m_rectfMap.x(), -(m_rectfMap.y() + m_rectfMap.height()));
}

void QFullViewWidget::drawImage()
{
  m_image = QImage(m_rectPicture.width(), m_rectPicture.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(230, 230, 230));
  this->drawMapBorder(painter);

  this->drawReferences(painter);
  this->drawVehicleLine(painter);
  this->drawVehicle(painter);
}

void QFullViewWidget::drawReferences(QPainter &painter)
{
  QPolygonF pgf;
  for (const auto &pt : m_listReferences) {
    pgf << QPointF(pt->x, pt->y);
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);
    painter.save();
    QPen pen;
    pen.setWidth(1);
    pen.setStyle(Qt::DotLine);
    pen.setBrush(Qt::black);
    painter.setPen(pen);
    painter.drawPolyline(pgf);
    painter.restore();
  }
}

void QFullViewWidget::drawVehicleLine(QPainter &painter)
{
  QPolygonF pgf;
  for (const auto &pt : m_listVehicleLine) {
    pgf << QPointF(pt->x, pt->y);
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);
    painter.save();
    QPen pen;
    pen.setWidth(2);
    pen.setStyle(Qt::SolidLine);
    pen.setBrush(Qt::green);
    painter.setPen(pen);
    painter.drawPolyline(pgf);
    painter.restore();
  }
}

void QFullViewWidget::drawVehicle(QPainter &painter)
{
  QPointF ptf = QPointF(m_planningData.vehicle_x, m_planningData.vehicle_y);
  ptf = m_transform.map(ptf);
  painter.save();
  QPen pen;
  pen.setWidth(2);
  pen.setColor(Qt::red);
  painter.setPen(pen);
  painter.drawEllipse(ptf, 3, 3);
  painter.restore();
}
