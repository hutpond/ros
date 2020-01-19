#include "QFullViewWidget.h"

#include <fstream>
#include <QtMath>
#include <QPainter>
#include <QPen>
#include <QMouseEvent>
#include <QtMath>

#include <eigen3/Eigen/Core>

struct MapPoint
{
  quint64 millsecond;
  double x;
  double y;
};

struct MapPoint3D
{
  double x;
  double y;
  double z;
};

void TransformRFU2ENU(const MapPoint3D *, const MapPoint3D *, const MapPoint3D *, MapPoint3D *);

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
  m_listVehicleLine.clear();
  m_listPlanningPoints.clear();
}

void QFullViewWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data,
                                      bool update)
{
  this->addVehicleLine(data);
  this->addPlanningPointLine(data);

  if (update) {
    m_planningData = data;
    this->doUpdate(true);
  }
}

int QFullViewWidget::findIndexPos(const QList<QSharedPointer<MapPoint>>& points, quint64 millsecond)
{
  const int size_points = points.size();
  if (size_points == 0) {
    return 0;
  }
  else if (size_points == 1 && millsecond == points[0]->millsecond) {
    return -1;
  }

  if (millsecond < points[0]->millsecond) {
    return 0;
  }
  else if (millsecond > points[size_points - 1]->millsecond) {
    return size_points;
  }

  int pos = -1;
  int low = 0;
  int high = points.size() - 1;
  while (high - low > 1) {
    int pos = (high + low) / 2;
    if (millsecond < points[pos]->millsecond) {
      high = pos;
    }
    else if (millsecond > points[pos]->millsecond) {
      low = pos;
    }
    else {
      pos = -1;
      break;
    }
  }
  if (high - low == 1) {
    if (millsecond == points[low]->millsecond || millsecond == points[high]->millsecond) {
      pos = -1;
    }
    else if (millsecond < points[low]->millsecond) {
      pos = low;
    }
    else {
      pos = high;
    }
  }
  return pos;
}

void QFullViewWidget::addVehicleLine(const debug_tool::ads_PlanningData4Debug &data)
{
  QSharedPointer<MapPoint> vehicle;
  vehicle.reset(new MapPoint);
  vehicle->millsecond = static_cast<quint64>(data.header.stamp.toSec() * 1000);
  vehicle->x = data.vehicle_enu_x;
  vehicle->y = data.vehicle_enu_y;

  int pos = this->findIndexPos(m_listVehicleLine, vehicle->millsecond);
  if (m_listVehicleLine.size() == 0 || pos == m_listVehicleLine.size()) {
    m_listVehicleLine.push_back(vehicle);
  }
  else if (pos >= 0) {
    m_listVehicleLine.insert(pos, vehicle);
  }
}

void QFullViewWidget::addPlanningPointLine(const debug_tool::ads_PlanningData4Debug &data)
{
  quint64 millsecond = static_cast<quint64>(data.header.stamp.toSec() * 1000);
  int pos = this->findIndexPos(m_listPlanningPoints, millsecond);
  if (pos < 0) {
    return;
  }

  MapPoint3D vehicleAngle, vehiclePos, localPos, enuPos;
  vehicleAngle.x = data.vehicle_pitch;
  vehicleAngle.y = data.vehicle_roll;
  vehicleAngle.z = data.vehicle_yaw;
  vehiclePos.x = data.vehicle_enu_x;
  vehiclePos.y = data.vehicle_enu_y;
  vehiclePos.z = data.vehicle_enu_z;
  localPos.x = data.planning_output.pose.position.x;
  localPos.y = data.planning_output.pose.position.y;
  localPos.z = data.planning_output.pose.position.z;

  TransformRFU2ENU(&vehiclePos, &vehicleAngle, &localPos, &enuPos);
  QSharedPointer<MapPoint> decision;
  decision.reset(new MapPoint);
  decision->millsecond = millsecond;
  decision->x = enuPos.x;
  decision->y = enuPos.y;

  if (m_listPlanningPoints.size() == 0 || m_listPlanningPoints.size() == pos) {
    m_listPlanningPoints.push_back(decision);
  }
  else {
    m_listPlanningPoints.insert(pos, decision);
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
  this->drawPlanningPointLine(painter);
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
  QPointF ptf = QPointF(m_planningData.vehicle_enu_x, m_planningData.vehicle_enu_y);
  ptf = m_transform.map(ptf);
  painter.save();
  QPen pen;
  pen.setWidth(2);
  pen.setColor(Qt::red);
  painter.setPen(pen);
  painter.drawEllipse(ptf, 3, 3);
  painter.restore();
}

void QFullViewWidget::drawPlanningPointLine(QPainter &painter)
{
  QPolygonF pgf;
  for (const auto &pt : m_listPlanningPoints) {
    pgf << QPointF(pt->x, pt->y);
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);
    painter.save();
    QPen pen;
    pen.setWidth(2);
    pen.setStyle(Qt::SolidLine);
    pen.setBrush(Qt::magenta);
    painter.setPen(pen);
    painter.drawPolyline(pgf);
    painter.restore();
  }
}

void QFullViewWidget::loadReferenceFile(const boost::filesystem::path &path)
{
  namespace fs = boost::filesystem;
  fs::path parent_path = path.parent_path();

  fs::directory_iterator end_iter;
  std::string name, name_sub;
  for (fs::directory_iterator it(path); it != end_iter; ++it) {
    std::stringstream ss;
    ss << *it;
    name_sub = ss.str();
    std::string substr = name_sub.substr(name_sub.size() - 4, 3);
    if (substr == "bin") {
      break;
    }
    name_sub.clear();
  }
  for (fs::directory_iterator it(parent_path); it != end_iter; ++it) {
    std::stringstream ss;
    ss << *it;
    name = ss.str();
    std::string substr = name.substr(name.size() - 4, 3);
    if (substr == "bin") {
      break;
    }
    name.clear();
  }

  if (name_sub.empty()) {
    name_sub = name;
  }
  if (name_sub.empty()) {
    return;
  }

  name = name_sub.substr(1, name_sub.size() - 2);
  if (!QFile::exists(QString::fromStdString(name))) {
    return;
  }

  m_listReferences.clear();
  std::ifstream in(name, std::ios::binary);
  int number = 0;
  in.read(reinterpret_cast<char*>(&number), sizeof(int));

  for (int i = 0; i < number; ++i) {
    QSharedPointer<MapPoint> point(new MapPoint);
    double value;
    int value_2;
    in.read(reinterpret_cast<char*>(&value_2), sizeof(value_2));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&point->x), sizeof(point->x));
    in.read(reinterpret_cast<char*>(&point->y), sizeof(point->y));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value), sizeof(value));
    in.read(reinterpret_cast<char*>(&value_2), sizeof(value_2));
    in.read(reinterpret_cast<char*>(&value_2), sizeof(value_2));
    in.read(reinterpret_cast<char*>(&value_2), sizeof(value_2));

    m_listReferences.push_back(point);
  }
  this->calcMapRect();
  in.close();
}

// 将角度转换成弧度
double DegreeToRadian(double value)
{
  return value * M_PI / 180;
}

// 将本车坐标系转换到ENU坐标系
void TransformRFU2ENU(const MapPoint3D *vehicalPos, const MapPoint3D *vehicleAngle,
                      const MapPoint3D *SLPoint, MapPoint3D *EnuPoint)
{
  double deltX = SLPoint->x;
  double deltY = SLPoint->y;
  double deltZ = SLPoint->z;
  double pitch = DegreeToRadian(vehicleAngle->x);
  double roll = DegreeToRadian(vehicleAngle->y);
  double yaw = DegreeToRadian(vehicleAngle->z);

  Eigen::MatrixXd Y(3, 1);
  Y(0, 0) = deltX;
  Y(1, 0) = deltY;
  Y(2, 0) = deltY;

  Eigen::MatrixXd R0(3, 3);

  R0 <<
        cos(M_PI / 2), sin(M_PI / 2), 0,
      -sin(M_PI / 2), cos(M_PI / 2), 0,
      0, 0, 1;

  Eigen::MatrixXd R(3, 3);

  R <<
       (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)), (-cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw)), (-sin(roll) * cos(pitch)),
      (cos(pitch) * sin(yaw)), (cos(pitch) * cos(yaw)), sin(pitch),
      (sin(roll) * cos(yaw) - cos(roll) * sin(pitch) * sin(yaw)), (-sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw)), cos(roll) * cos(pitch);

  Eigen::MatrixXd R2(3, 1);

  R2 = R.transpose() * R0.transpose() * Y;
  EnuPoint->x = R2(0, 0) + vehicalPos->x;
  EnuPoint->y = R2(1, 0) + vehicalPos->y;
  EnuPoint->z = R2(2, 0) + vehicalPos->z;
}
