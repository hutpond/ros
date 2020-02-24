#ifndef QSHOWWIDGET_H
#define QSHOWWIDGET_H

#include "qbaseshowwidget.h"
#include "decision_studio/ads_DecisionData4Debug.h"

class QShowWidget : public QBaseShowWidget
{
  Q_OBJECT

public:
  explicit QShowWidget(QWidget * = Q_NULLPTR);
  void setData(const decision_studio::ads_DecisionData4Debug &);

protected:
  virtual void mousePressEvent(QMouseEvent *) override;

protected:
  virtual void drawImage() final;
  virtual void calcMapRect() final;

  void drawSweeper(QPainter &);
  void drawRoadSideFromWidth(QPainter &);
  void drawTrackTargetWithPoints(QPainter &);

  int findReferenceIndex(const double);
  int findReferenceIndex(const double, const double);
  void xyToSl(const QPointF &, double &, double &);
  void slToXy(const double, const double, QPointF &);

private:
  mutable decision_studio::ads_DecisionData4Debug m_decisionData;
};

#endif // QSHOWWIDGET_H
