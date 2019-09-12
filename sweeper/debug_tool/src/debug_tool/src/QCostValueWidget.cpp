#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include "QCostValueWidget.h"

int QCostValueWidget::s_nValue[QPlanningCostWidget::Count] = {20, 20, 20, 20, 20, 20};
int QCostValueWidget::s_nOriginValue[QPlanningCostWidget::Count] = {-1, -1, -1, -1, -1, -1};

QCostValueWidget::QCostValueWidget(QWidget *parent)
  : QWidget(parent)
{
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    m_pSlider[i] = new QSlider(Qt::Horizontal, this);
    m_pSlider[i]->setRange(0, 100);
    m_pSlider[i]->setTickInterval(1);
    m_pSlider[i]->setValue(s_nValue[i]);
    connect(m_pSlider[i], &QSlider::valueChanged, this, &QCostValueWidget::onValueChanged);

    m_pLblValue[i] = new QLabel(this);
    m_pLblValue[i]->setText(QString::number(s_nValue[i]));
  }
  m_pBtnReset = new QPushButton("RESET", this);
  connect(m_pBtnReset, &QPushButton::clicked, this, &QCostValueWidget::onBtnReset);
  m_pLblSum = new QLabel("sum: 100", this);

  QGridLayout *layout = new QGridLayout;
  layout->addWidget(new QLabel("Safety"), 0, 0, 1, 2);
  layout->addWidget(m_pSlider[QPlanningCostWidget::Safety], 0, 2, 1, 9);
  layout->addWidget(m_pLblValue[QPlanningCostWidget::Safety], 0, 11, 1, 1);

  layout->addWidget(new QLabel("Lateral"), 1, 0, 1, 2);
  layout->addWidget(m_pSlider[QPlanningCostWidget::Lateral], 1, 2, 1, 9);
  layout->addWidget(m_pLblValue[QPlanningCostWidget::Lateral], 1, 11, 1, 1);

  layout->addWidget(new QLabel("Smoothness"), 2, 0, 1, 2);
  layout->addWidget(m_pSlider[QPlanningCostWidget::Smoothness], 2, 2, 1, 9);
  layout->addWidget(m_pLblValue[QPlanningCostWidget::Smoothness], 2, 11, 1, 1);

  layout->addWidget(new QLabel("Consistency"), 3, 0, 1, 2);
  layout->addWidget(m_pSlider[QPlanningCostWidget::Consistency], 3, 2, 1, 9);
  layout->addWidget(m_pLblValue[QPlanningCostWidget::Consistency], 3, 11, 1, 1);

  layout->addWidget(new QLabel("Garbage"), 4, 0, 1, 2);
  layout->addWidget(m_pSlider[QPlanningCostWidget::Garbage], 4, 2, 1, 9);
  layout->addWidget(m_pLblValue[QPlanningCostWidget::Garbage], 4, 11, 1, 1);

  layout->addWidget(m_pLblSum, 5, 2, 1, 3);
  layout->addWidget(m_pBtnReset, 5, 7, 1, 3);

  this->setLayout(layout);
}

void QCostValueWidget::onValueChanged(int)
{
  int sum = 0;
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    s_nValue[i] = m_pSlider[i]->value();
    m_pLblValue[i]->setText(QString::number(s_nValue[i]));
    sum += s_nValue[i];
  }
  m_pLblSum->setText(QString("sum: %1").arg(sum));
  emit costValueChanged();
}

void QCostValueWidget::onBtnReset()
{
  int sum = 0;
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    sum += s_nOriginValue[i];
  }
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    if (sum > 0) {
      s_nValue[i] = s_nOriginValue[i];
    }
    else {
      s_nValue[i] = 20;
    }
    m_pSlider[i]->setValue(s_nValue[i]);
  }
}

void QCostValueWidget::getCostValue(double value[])
{
  int sum = 0;
  int sum2 = 0;
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    sum += s_nValue[i];
    sum2 += s_nOriginValue[i];
  }
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    if (sum > 0) {
      value[i] = (double)s_nValue[i];
    }
    else if (sum2 > 0){
      value[i] = (double)s_nOriginValue[i];
    }
    else {
      value[i] = 20;
    }
  }
}

void QCostValueWidget::setOriginCostValue(double value[])
{
  int sum = 0;
  if (s_nOriginValue[1] == -1) {
    for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
      s_nOriginValue[i] = (int)value[i];
      sum += s_nOriginValue[i];
    }
  }
  if (sum > 0) {
    for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
      s_nValue[i] = s_nOriginValue[i];
    }
  }
}
