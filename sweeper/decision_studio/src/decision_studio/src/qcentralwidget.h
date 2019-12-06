#ifndef QCENTRALWIDGET_H
#define QCENTRALWIDGET_H

#include <QWidget>
#include <boost/filesystem.hpp>
#include "decision_studio/ads_DecisionData4Debug.h"

class QShowWidget;
class QStateWidget;
class DecisionSubscriber;
class QReplayWidget;

class QCentralWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QCentralWidget(QWidget *parent = nullptr);
  void setData(const decision_studio::ads_DecisionData4Debug &);

  void openReplayDir(const QString &);
  void setSaveDataFlag(bool);
  void setLivingFlag(bool);
  void createSavePath(const boost::filesystem::path &path);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

signals:

public slots:

private:
  QShowWidget *m_pWdgShow;
  QStateWidget *m_pWdgState;
  QReplayWidget *m_pWdgReplay;
  DecisionSubscriber *m_pObjSubscriber;
};

#endif // QCENTRALWIDGET_H
