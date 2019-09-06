#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QGridLayout>
#include "QCostValueDialog.h"

QCostValueDialog::QCostValueDialog(QWidget *parent)
  : QDialog(parent)
{
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    m_pEditValue[i] = new QLineEdit(this);
  }
  m_pBtnOk = new QPushButton("OK", this);
  m_pBtnCancel = new QPushButton("Cancel", this);
  connect(m_pBtnOk, &QPushButton::clicked, this, &QDialog::accept);
  connect(m_pBtnCancel, &QPushButton::clicked, this, &QDialog::reject);

  QGridLayout *layout = new QGridLayout;
  layout->addWidget(new QLabel("Safety"), 0, 0, 1, 2);
  layout->addWidget(m_pEditValue[QPlanningCostWidget::Safety], 0, 2, 1, 3);

  layout->addWidget(new QLabel("Lateral"), 1, 0, 1, 2);
  layout->addWidget(m_pEditValue[QPlanningCostWidget::Lateral], 1, 2, 1, 3);

  layout->addWidget(new QLabel("Smoothness"), 2, 0, 1, 2);
  layout->addWidget(m_pEditValue[QPlanningCostWidget::Smoothness], 2, 2, 1, 3);

  layout->addWidget(new QLabel("Consistency"), 3, 0, 1, 2);
  layout->addWidget(m_pEditValue[QPlanningCostWidget::Consistency], 3, 2, 1, 3);

  layout->addWidget(new QLabel("Garbage"), 4, 0, 1, 2);
  layout->addWidget(m_pEditValue[QPlanningCostWidget::Garbage], 4, 2, 1, 3);

  layout->addWidget(m_pBtnOk, 5, 1, 1, 1);
  layout->addWidget(m_pBtnCancel, 5, 3, 1, 1);

  this->setLayout(layout);
}

void QCostValueDialog::setCostValue(double value[])
{
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    m_pEditValue[i]->setText(QString::number(value[i]));
  }
}

void QCostValueDialog::getCostValue(double value[])
{
  for (int i = 1; i < QPlanningCostWidget::Count; ++i) {
    QString text = m_pEditValue[i]->text();
    value[i] = text.toDouble();
  }
}
