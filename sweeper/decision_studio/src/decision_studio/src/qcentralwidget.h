#ifndef QCENTRALWIDGET_H
#define QCENTRALWIDGET_H

#include <QWidget>
#include "decision_studio/ads_DecisionData4Debug.h"

class QShowWidget;
class QStateWidget;
class DecisionSubscriber;

class QCentralWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QCentralWidget(QWidget *parent = nullptr);
  void setData(const decision_studio::ads_DecisionData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

signals:

public slots:

private:
  QShowWidget *m_pWdgShow;
  QStateWidget *m_pWdgState;
  DecisionSubscriber *m_pObjSubscriber;
};

#endif // QCENTRALWIDGET_H
