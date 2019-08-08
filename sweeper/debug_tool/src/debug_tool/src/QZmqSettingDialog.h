/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QZmqSettingDialog.h
 * Author: liuzheng
 * Date: 2019/6/25
 * Description: zmq设置对话框
********************************************************/
#ifndef Q_ZMQ_SETTING_DIALOG_H
#define Q_ZMQ_SETTING_DIALOG_H

#include <QDialog>

class QLabel;
class QLineEdit;

class QZmqSettingDialog : public QDialog
{
	Q_OBJECT

public:
	QZmqSettingDialog(QWidget *parent);
	~QZmqSettingDialog();

protected:
	virtual void resizeEvent(QResizeEvent *);

private:
	void readIniFile();

private slots:
	void onBtnOkClicked();
	void onBtnCancelClicked();

protected:
	QLabel *m_pLblPlanning;
	QLabel *m_pLblPlanningIp;
	QLineEdit *m_pEditPlanningIp;
	QLabel *m_pLblPlanningPort;
	QLineEdit *m_pEditPlanningPort;

	QPushButton *m_pBtnOk;
	QPushButton *m_pBtnCancel;
};

#endif  // Q_ZMQ_SETTING_DIALOG_H
