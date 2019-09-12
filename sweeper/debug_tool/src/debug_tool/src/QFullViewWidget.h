#ifndef QFULLVIEWWIDGET_H
#define QFULLVIEWWIDGET_H

#include "QBaseShowWidget.h"
#include "debug_tool/ads_PlanningData4Debug.h"

struct MapPoint;

class QFullViewWidget : public QBaseShowWidget
{
  Q_OBJECT

public:
  explicit QFullViewWidget(QWidget *parent = Q_NULLPTR);

  void clearMapDatas();
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &,
                       const QString &, bool);

protected:
  void mousePressEvent(QMouseEvent *);

protected:
  void calcMapRect();
  void drawImage();
  void drawReferences(QPainter &);
  void drawVehicleLine(QPainter &);
  void drawVehicle(QPainter &);

  quint64 nameToIndex(const QString &);
  int findIndexPos(const QList<QSharedPointer<MapPoint>>&, quint64);
  double xLocal2Global(const debug_tool::ads_PlanningData4Debug &, double);
  double yLocal2Global(const debug_tool::ads_PlanningData4Debug &, double);

  void addReference(const debug_tool::ads_PlanningData4Debug &, quint64);
  void addVehicleLine(const debug_tool::ads_PlanningData4Debug &, quint64);
  int isIndexValid(const QList<QSharedPointer<MapPoint>>&, quint64);

protected:
  debug_tool::ads_PlanningData4Debug m_planningData;
  QList<QSharedPointer<MapPoint>> m_listReferences;
  QList<QSharedPointer<MapPoint>> m_listVehicleLine;
};

#endif // QFULLVIEWWIDGET_H
