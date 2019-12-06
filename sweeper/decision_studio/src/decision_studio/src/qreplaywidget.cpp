#include <QPushButton>
#include <QSlider>
#include "qreplaywidget.h"

QReplayWidget::QReplayWidget(QWidget *parent)
  : QWidget(parent)
  , m_bFlagPlay(true)
  , m_nVelocity(0)
{
  m_pSliderPlay = new QSlider(Qt::Horizontal, this);
  connect(m_pSliderPlay, &QSlider::valueChanged, this, &QReplayWidget::onSliderValueChanged);

  for (int i = 0 ; i < BtnCount; ++i) {
    m_pBtnPlay[i] = new QPushButton(this);
    connect(m_pBtnPlay[i], &QPushButton::clicked, this, &QReplayWidget::onBtnClicked);
  }
  m_pBtnPlay[BtnPrevious]->setIcon(QIcon(QStringLiteral(":image/previous.svg")));
  m_pBtnPlay[BtnNext]->setIcon(QIcon(QStringLiteral(":image/next.svg")));

  this->setPlayState(m_bFlagPlay);
  this->setPlayVelocity(m_nVelocity);
}

void QReplayWidget::setSliderSize(int size)
{
  m_pSliderPlay->setMinimum(0);
  m_pSliderPlay->setMaximum(size);
  m_pSliderPlay->setPageStep(1);
  m_pSliderPlay->setSingleStep(1);

  m_pSliderPlay->setValue(0);

  m_bFlagPlay = true;
  m_nVelocity = 0;
  this->setPlayState(m_bFlagPlay);
  this->setPlayVelocity(m_nVelocity);
}

void QReplayWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const float BTN_H_PF = 0.7;
  const float SLIDER_H_PF = 0.2;
  const float SLIDER_W_PF = 0.6;

  const float SPACE_X = (float)(WIDTH - HEIGHT * BTN_H_PF * BtnCount - WIDTH * SLIDER_W_PF) /
      (float)(BtnCount + 2);

  for (int i = 0; i < BtnCount; ++i) {
    m_pBtnPlay[i]->setGeometry(
          SPACE_X + i * (SPACE_X + HEIGHT * BTN_H_PF),
          HEIGHT * (1.0 - BTN_H_PF) / 2.0,
          HEIGHT * BTN_H_PF,
          HEIGHT * BTN_H_PF
          );
  }
  m_pSliderPlay->setGeometry(
        WIDTH * (1.0 - SLIDER_W_PF) - SPACE_X,
        HEIGHT * (1.0 - SLIDER_H_PF) / 2.0,
        WIDTH * SLIDER_W_PF,
        HEIGHT * SLIDER_H_PF
        );
}

void QReplayWidget::onBtnClicked()
{
  QObject *sender = this->sender();
  int index = -1;
  for (int i = 0; i < BtnCount; ++i) {
    if (m_pBtnPlay[i] == sender) {
      index = i;
      break;
    }
  }
  switch (index) {
  case BtnPlayPause:
    m_bFlagPlay = !m_bFlagPlay;
    this->setPlayState(m_bFlagPlay);
    break;
  case BtnPrevious:
  {
    int value = m_pSliderPlay->value();
    if (value > m_pSliderPlay->minimum()) {
      -- value;
      m_pSliderPlay->setValue(value);
      emit clicked(index, value);
    }
    break;
  }
  case BtnNext:
  {
    int value = m_pSliderPlay->value();
    if (value < m_pSliderPlay->maximum() - 1) {
      ++ value;
      m_pSliderPlay->setValue(value);
      emit clicked(index, value);
    }
    break;
  }
  case BtnPlayVelocity:
    ++ m_nVelocity;
    if (m_nVelocity == 4) {
      m_nVelocity = 0;
    }
    this->setPlayVelocity(m_nVelocity);
    break;
  default:
    break;
  }
}

void QReplayWidget::onSliderValueChanged(int value)
{
  emit clicked(BtnCount, value);
}

void QReplayWidget::setPlayState(bool flag)
{
  m_pBtnPlay[BtnPlayPause]->setIcon(
        flag ? QIcon(QStringLiteral(":image/pause.svg")) :
               QIcon(QStringLiteral(":image/play.svg")));
  m_pBtnPlay[BtnPrevious]->setEnabled(!flag);
  m_pBtnPlay[BtnNext]->setEnabled(!flag);
  m_pBtnPlay[BtnPlayVelocity]->setEnabled(flag);
  m_pSliderPlay->setEnabled(!flag);

  emit clicked(BtnPlayPause, flag ? 1 : 0);
}

void QReplayWidget::setPlayVelocity(int velocity)
{
  if (velocity == 0) {
    m_pBtnPlay[BtnPlayVelocity]->setIcon(QIcon(QStringLiteral(":image/velocity_1.svg")));
  }
  else if (velocity == 1) {
    m_pBtnPlay[BtnPlayVelocity]->setIcon(QIcon(QStringLiteral(":image/velocity_2.svg")));
  }
  else if (velocity == 2) {
    m_pBtnPlay[BtnPlayVelocity]->setIcon(QIcon(QStringLiteral(":image/velocity_3.svg")));
  }
  else if (velocity == 3) {
    m_pBtnPlay[BtnPlayVelocity]->setIcon(QIcon(QStringLiteral(":image/velocity_4.svg")));
  }
  emit clicked(BtnPlayVelocity, velocity);
}
