#ifndef QCENTRALWIDGET_H
#define QCENTRALWIDGET_H

#include <functional>
#include <QWidget>
#include <boost/filesystem.hpp>
#include "decision_studio/ads_DecisionData4Debug.h"

class QLabel;
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
  void setFunciton(std::function<void(const decision_studio::ads_DecisionData4Debug &)>);

  void openReplayDir(const QString &);
  void setSaveDataFlag(bool);
  void setLivingFlag(bool);
  void createSavePath(const boost::filesystem::path &path);

  void showMousePosition(float, float, float, float);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

signals:
  void replayFileName(const QString &);

public slots:

private:
  QShowWidget *m_pWdgShow;
  QStateWidget *m_pWdgState;
  QLabel *m_pLblPosition;
  QReplayWidget *m_pWdgReplay;
  DecisionSubscriber *m_pObjSubscriber;

  std::function<void(const decision_studio::ads_DecisionData4Debug &)> m_funciton;
};

#endif // QCENTRALWIDGET_H
