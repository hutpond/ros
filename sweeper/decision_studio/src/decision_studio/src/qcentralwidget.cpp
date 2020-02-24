#include "qcentralwidget.h"

#include <QLabel>
#include "qshowwidget.h"
#include "qreplaywidget.h"
#include "qstatewidget.h"
#include "decision_subscriber.h"

QCentralWidget::QCentralWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pWdgShow = new QShowWidget(this);
  m_pWdgState = new QStateWidget(this);
  m_pLblPosition = new QLabel(this);
  m_pWdgReplay = new QReplayWidget(this);
  m_pWdgReplay->hide();
  m_pObjSubscriber = new DecisionSubscriber(*this, this);

  connect(m_pWdgReplay, &QReplayWidget::clicked,
          m_pObjSubscriber, &DecisionSubscriber::onPlayClicked);

  connect(m_pObjSubscriber, SIGNAL(replayFileName(const QString &)),
          this, SIGNAL(replayFileName(const QString &)));
  connect(m_pObjSubscriber, SIGNAL(replayNextIndex()),
           m_pWdgReplay, SLOT(onSliderNextStep()));

  boost::function<void(float, float, float, float)> fun =
      boost::bind(&QCentralWidget::showMousePosition, this, _1, _2, _3, _4);
  m_pWdgShow->setFunPosition(fun);
}

void QCentralWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const float WDG_STATE_H_PF = 0.1f;
  const float WDG_STATE_W_PF = 0.25f;

  m_pWdgShow->setGeometry(0, 0, WIDTH, HEIGHT * (1.0 - WDG_STATE_H_PF));

  m_pWdgState->setGeometry(
        0, HEIGHT * (1.0 - WDG_STATE_H_PF), WIDTH * WDG_STATE_W_PF, HEIGHT * WDG_STATE_H_PF
        );

  const float LBL_POS_W_PF = 0.2f;
  m_pLblPosition->setGeometry(
        WIDTH * WDG_STATE_W_PF,
        HEIGHT * (1.0 - WDG_STATE_H_PF),
        WIDTH * LBL_POS_W_PF,
        HEIGHT * WDG_STATE_H_PF
        );

  m_pWdgReplay->setGeometry(
        WIDTH * (WDG_STATE_W_PF + LBL_POS_W_PF),
        HEIGHT * (1.0 - WDG_STATE_H_PF),
        WIDTH * (1 - WDG_STATE_W_PF - LBL_POS_W_PF),
        HEIGHT * WDG_STATE_H_PF
        );
}

void QCentralWidget::setData(const decision_studio::ads_DecisionData4Debug &data)
{
  m_pWdgShow->setData(data);
  m_pWdgState->setData(data);
  m_funciton(data);
}

void QCentralWidget::setFunciton(
    std::function<void(const decision_studio::ads_DecisionData4Debug &)> function)
{
  m_funciton = function;
}

void QCentralWidget::openReplayDir(const QString &path)
{
  m_pObjSubscriber->openReplayDir(path);
  m_pWdgReplay->setSliderSize(m_pObjSubscriber->sizeOfReplayFiles());
  m_pWdgReplay->show();
}

void QCentralWidget::setSaveDataFlag(bool flag)
{
  m_pObjSubscriber->setSaveFileFlag(flag);
}

void QCentralWidget::setLivingFlag(bool flag)
{
  m_pWdgReplay->setVisible(!flag);
  m_pObjSubscriber->setLiving(flag);
}

void QCentralWidget::createSavePath(const boost::filesystem::path &path)
{
  m_pObjSubscriber->createSavePath(path);
}

void QCentralWidget::showMousePosition(float x, float y, float s, float l)
{
  QString strText = QString("X:%1, Y:%2, S:%3, L%4").
      arg(x, 6, 'f', 2).
      arg(y, 6, 'f', 2).
      arg(s, 6, 'f', 2).
      arg(l, 6, 'f', 2);
  m_pLblPosition->setText(strText);
}
