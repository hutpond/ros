#ifndef QFULLVIEWWIDGET_H
#define QFULLVIEWWIDGET_H

#include <boost/filesystem.hpp>
#include "QBaseShowWidget.h"
#include "debug_tool/ads_PlanningData4Debug.h"

struct MapPoint;

class QFullViewWidget : public QBaseShowWidget
{
  Q_OBJECT

public:
  explicit QFullViewWidget(QWidget *parent = Q_NULLPTR);

  void clearMapDatas();
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &, bool);
  void loadReferenceFile(const boost::filesystem::path &);

protected:
  void mousePressEvent(QMouseEvent *);

protected:
  void calcMapRect();
  void drawImage();
  void drawReferences(QPainter &);
  void drawVehicleLine(QPainter &);
  void drawVehicle(QPainter &);
  void drawPlanningPointLine(QPainter &);

  int findIndexPos(const QList<QSharedPointer<MapPoint>>&, quint64);
  void addVehicleLine(const debug_tool::ads_PlanningData4Debug &);
  void addPlanningPointLine(const debug_tool::ads_PlanningData4Debug &);

protected:
  debug_tool::ads_PlanningData4Debug m_planningData;
  QList<QSharedPointer<MapPoint>> m_listReferences;
  QList<QSharedPointer<MapPoint>> m_listVehicleLine;
  QList<QSharedPointer<MapPoint>> m_listPlanningPoints;
};

#endif // QFULLVIEWWIDGET_H
