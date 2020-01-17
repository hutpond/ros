#ifndef QDECISIONSTATEWIDGET_H
#define QDECISIONSTATEWIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QStateWidget;
class QFrameTimeWidget;

class QDecisionStateWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QDecisionStateWidget(QWidget *parent = nullptr);
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);
  void clearData();

protected:
  virtual void resizeEvent(QResizeEvent *) final;

signals:

public slots:

private:
  QStateWidget *m_pWdgState;
  QFrameTimeWidget *m_pWdgFrameTime;
};

#endif // QDECISIONSTATEWIDGET_H
