#ifndef QSTATEWIDGET_H
#define QSTATEWIDGET_H

#include <QWidget>
#include "decision_studio/ads_DecisionData4Debug.h"

class QStateItem;

class QStateWidget : public QWidget
{
  Q_OBJECT

  enum {
    StateUnknown,
    StateForward,
    StateLeftPass,
    StateRightPass,
    StateFollow,
    StateWait,
    StateSafeStop,
    StateExitAuto,
    StateCount
  };

public:
  explicit QStateWidget(QWidget *parent = nullptr);
  void setData(const decision_studio::ads_DecisionData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

public slots:

private:
  QStateItem *m_pWdgItem[StateCount];
};

#endif // QSTATEWIDGET_H
