/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QSettingDialog.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 设置对话框
********************************************************/
#ifndef QSETTINGDIALOG_H
#define QSETTINGDIALOG_H

#include <QDialog>

class QTabWidget;
class QLineEdit;
class QComboBox;
class QCheckBox;

class QSettingDialog : public QDialog
{
  Q_OBJECT

public:
  QSettingDialog(QWidget *);
  void setSerialParam(const QString &, qint32, bool);
  void getSerialParam(QString &, qint32 &, bool &);

private:
  QTabWidget *m_pTabWidget;
  QPushButton *m_pBtnOk;
  QPushButton *m_pBtnCancel;
};

class QSerialSettingWidget : public QWidget
{
  Q_OBJECT

public:
  QSerialSettingWidget(QWidget *);
  void setSerialParam(const QString &, qint32, bool);
  void getSerialParam(QString &, qint32 &, bool &);

private:
  QLineEdit *m_pEditDevice;
  QComboBox *m_pCmbBaud;
  QCheckBox *m_pCheckBoxSelect;
};

#endif // QSETTINGDIALOG_H