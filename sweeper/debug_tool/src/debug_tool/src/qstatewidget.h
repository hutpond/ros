#ifndef QSTATEWIDGET_H
#define QSTATEWIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QStateItem;

class QStateWidget : public QWidget
{
  Q_OBJECT

  enum {
    StateForward,
    StateFollow,
    StatePass,
    StateSafeStop,
    StateExitAuto,
    StateCount
  };

public:
  explicit QStateWidget(QWidget *parent = nullptr);
  void setData(const debug_tool::ads_PlanningData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

public slots:

private:
  QStateItem *m_pWdgItem[StateCount];
};

#endif // QSTATEWIDGET_H
