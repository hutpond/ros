#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include "QCostValueDialog.h"

int QCostValueDialog::s_nValue[QPlanningCostWidget::Count] = {0};

QCostValueDialog::QCostValueDialog(QWidget *parent)
  : QDialog(parent)
{
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    m_pSlider[i] = new QSlider(Qt::Horizontal, this);
    m_pSlider[i]->setRange(0, 100);
    m_pSlider[i]->setTickInterval(1);
    m_pSlider[i]->setValue(s_nValue[i]);
    connect(m_pSlider[i], &QSlider::valueChanged, this, &QCostValueDialog::onValueChanged);

    m_pLblValue[i] = new QLabel(this);
  }

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

  this->setLayout(layout);
}

void QCostValueDialog::showEvent(QShowEvent *)
{
  this->onValueChanged(0);
}

void QCostValueDialog::setCostValue(double value[])
{
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    m_pLblValue[i]->setText(QString("%1").arg(value[i], -1, 'f', 2));
  }
}

void QCostValueDialog::calcPerccent(double value[])
{
  int sum = 0;
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    s_nValue[i] = m_pSlider[i]->value();
    sum += m_pSlider[i]->value();
  }
  if (sum == 0) {
    return;
  }
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    value[i] = (double)m_pSlider[i]->value() / (double)sum;
  }
}

void QCostValueDialog::onValueChanged(int)
{
  double value[QPlanningCostWidget::Count];
  memset(value, 0, sizeof(double) * QPlanningCostWidget::Count);
  this->calcPerccent(value);
  this->setCostValue(value);
  emit costValue(value);
}
