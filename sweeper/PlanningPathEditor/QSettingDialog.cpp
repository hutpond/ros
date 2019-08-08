/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QSettingDialog.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 设置对话框
********************************************************/
#include <QTabWidget>
#include <QPushButton>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include "QSettingDialog.h"

static constexpr qint32 BAUD[] = {4800, 9600, 19200, 38400, 57600, 115200};

QSettingDialog::QSettingDialog(QWidget *parent)
  : QDialog(parent)
{
  m_pTabWidget = new QTabWidget(this);
  m_pTabWidget->addTab(new QSerialSettingWidget(m_pTabWidget), QStringLiteral("Serial"));
  m_pBtnOk = new QPushButton(QStringLiteral("OK"), this);
  m_pBtnCancel = new QPushButton(QStringLiteral("Cancel"), this);
  connect(m_pBtnOk, &QPushButton::clicked, this, &QDialog::accept);
  connect(m_pBtnCancel, &QPushButton::clicked, this, &QDialog::reject);

  QHBoxLayout *hLayout = new QHBoxLayout;
  hLayout->addStretch(2);
  hLayout->addWidget(m_pBtnOk, 1);
  hLayout->addStretch(2);
  hLayout->addWidget(m_pBtnCancel, 1);
  hLayout->addStretch(2);

  // TODO stretch not work
  QVBoxLayout *vLayout = new QVBoxLayout;
  vLayout->addWidget(m_pTabWidget);
  vLayout->setStretchFactor(m_pTabWidget, 1);
  vLayout->addLayout(hLayout);
  vLayout->setStretchFactor(hLayout, 2);
  this->setLayout(vLayout);
}

void QSettingDialog::setSerialParam(const QString &device, qint32 baud)
{
  QSerialSettingWidget *pWdgSerial = dynamic_cast<QSerialSettingWidget *>(
        m_pTabWidget->currentWidget());
  if (pWdgSerial) {
    pWdgSerial->setSerialParam(device, baud);
  }
}

void QSettingDialog::getSerialParam(QString &device, qint32 &baud)
{
  QSerialSettingWidget *pWdgSerial = dynamic_cast<QSerialSettingWidget *>(
        m_pTabWidget->currentWidget());
  if (pWdgSerial) {
    pWdgSerial->getSerialParam(device, baud);
  }
}


QSerialSettingWidget::QSerialSettingWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pEditDevice = new QLineEdit(this);
  m_pCmbBaud = new QComboBox(this);
  for (const auto &item : BAUD) {
    m_pCmbBaud->addItem(QString::number(item));
  }

  QVBoxLayout *vLayout = new QVBoxLayout;
  QHBoxLayout *hLayout = new QHBoxLayout;
  hLayout->addWidget(new QLabel("Device"), 1);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pEditDevice, 4);
  hLayout->addStretch(3);
  vLayout->addStretch(1);
  vLayout->addLayout(hLayout, 1);
  vLayout->addStretch(1);

  hLayout = new QHBoxLayout;
  hLayout->addWidget(new QLabel("Baud"), 1);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pCmbBaud, 4);
  hLayout->addStretch(3);
  vLayout->addLayout(hLayout, 1);
  vLayout->addStretch(4);

  this->setLayout(vLayout);
}

void QSerialSettingWidget::setSerialParam(const QString &device, qint32 baud)
{
  m_pEditDevice->setText(device);
  int index = 0;
  for (const auto &item : BAUD) {
    if (baud == item) break;
    ++index;
  }
  if (index >= m_pCmbBaud->count()) {
    index = 1;
  }
  m_pCmbBaud->setCurrentIndex(index);
}

void QSerialSettingWidget::getSerialParam(QString &device, qint32 &baud)
{
  device = m_pEditDevice->text();
  QString text = m_pCmbBaud->currentText();
  baud = text.toInt();
}
