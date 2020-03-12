#ifndef QDECISIONSTATE_H
#define QDECISIONSTATE_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QStateItem;
class QLabel;

class QDecisionState : public QWidget
{
  Q_OBJECT
public:
  explicit QDecisionState(QWidget *parent = nullptr);
  void setData(const debug_tool::ads_PlanningData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  QString getDecisionText(uint8_t);

signals:

public slots:

private:
  QLabel *m_pLblName[10];
  QLabel *m_pLblDecision[10];
  QStateItem *m_pWdgItem[10];
};

#endif // QDECISIONSTATE_H
