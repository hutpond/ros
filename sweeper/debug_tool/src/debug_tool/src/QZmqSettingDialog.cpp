/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QZmqSettingDialog.cpp
 * Author: liuzheng
 * Date: 2019/6/25
 * Description: zmq设置对话框
********************************************************/
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include "QZmqSettingDialog.h"
#include "ReadDataManager.h"
#include "GlobalDefine.h"

QZmqSettingDialog::QZmqSettingDialog(QWidget *parent)
	: QDialog(parent)
{
	this->setFont(G_TEXT_FONT);

	m_pLblPlanning = new QLabel(QStringLiteral("Planning"), this);
	m_pLblPlanningIp = new QLabel(QStringLiteral("IP"), this);
	m_pLblPlanningPort = new QLabel(QStringLiteral("PORT"), this);
	m_pEditPlanningIp = new QLineEdit(this);
	m_pEditPlanningPort = new QLineEdit(this);

	m_pBtnOk = new QPushButton(QStringLiteral("OK"), this);
	m_pBtnCancel = new QPushButton(QStringLiteral("CANCEL"), this);

	connect(m_pBtnOk, &QPushButton::clicked, this, &QZmqSettingDialog::onBtnOkClicked);
	connect(m_pBtnCancel, &QPushButton::clicked, this, &QZmqSettingDialog::onBtnCancelClicked);
	this->readIniFile();
}

QZmqSettingDialog::~QZmqSettingDialog()
{
}

void QZmqSettingDialog::resizeEvent(QResizeEvent *)
{
	constexpr float SPACE_X_PF = 0.05;
	constexpr float SPACE_Y_PF = 0.08;
	constexpr float ITEM_H_PF = 0.06;
	constexpr float ITEM_NAME_W_PF = 0.12;
	constexpr float ITEM_IP_NAME_W_PF = 0.04;
	constexpr float ITEM_IP_VAL_W_PF = 0.25;
	constexpr float ITEM_PORT_NAME_PF = 0.08;
	constexpr float ITEM_PORT_VAL_W_PF = 0.15;

	const int WIDTH = this->width();
	const int HEIGHT = this->height();

	float fPosX = WIDTH * SPACE_X_PF;
	float fPosY = HEIGHT * SPACE_Y_PF;
	m_pLblPlanning->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_NAME_W_PF,
		HEIGHT * ITEM_H_PF
		);

	fPosX += WIDTH * (ITEM_NAME_W_PF + SPACE_X_PF);
	m_pLblPlanningIp->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_IP_NAME_W_PF,
		HEIGHT * ITEM_H_PF
	);

	fPosX += WIDTH * ITEM_IP_NAME_W_PF;
	m_pEditPlanningIp->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_IP_VAL_W_PF,
		HEIGHT * ITEM_H_PF
	);

	fPosX += WIDTH * (ITEM_IP_VAL_W_PF + SPACE_X_PF);
	m_pLblPlanningPort->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_PORT_NAME_PF,
		HEIGHT * ITEM_H_PF
	);

	fPosX += WIDTH * ITEM_PORT_NAME_PF;
	m_pEditPlanningPort->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_PORT_VAL_W_PF,
		HEIGHT * ITEM_H_PF
	);

	constexpr float ITEM_BTN_W_PF = 0.15;
	constexpr float SPACE_BTN_X_PF = (1.0 - ITEM_BTN_W_PF * 2.0) / 3.0;
	fPosX = WIDTH * SPACE_BTN_X_PF;
	fPosY = HEIGHT * (1.0 - SPACE_Y_PF * 2 - ITEM_H_PF);
	m_pBtnOk->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_BTN_W_PF,
		HEIGHT * ITEM_H_PF
	);

	fPosX += WIDTH * (ITEM_BTN_W_PF + SPACE_BTN_X_PF);
	fPosY = HEIGHT * (1.0 - SPACE_Y_PF * 2 - ITEM_H_PF);
	m_pBtnCancel->setGeometry(
		fPosX, fPosY,
		WIDTH * ITEM_BTN_W_PF,
		HEIGHT * ITEM_H_PF
	);
}

/*******************************************************
 * @brief 从配置文件读取zmq配置
 * @param

 * @return
********************************************************/
void QZmqSettingDialog::readIniFile()
{
	ServerAddress address[ReadDataManager::Count];
	int len = 0;
	ReadDataManager::instance()->getServerAddress(address, len);

	QString ip = QString::fromStdString(address[ReadDataManager::Planning].ip);
	short port = address[ReadDataManager::Planning].port;
	m_pEditPlanningIp->setText(ip);
	m_pEditPlanningPort->setText(QString::number(port));
}

/*******************************************************
 * @brief Button OK clicked响应槽函数
 * @param

 * @return
********************************************************/
void QZmqSettingDialog::onBtnOkClicked()
{
	ServerAddress address[ReadDataManager::Count];
	
	address[ReadDataManager::Planning].ip = m_pEditPlanningIp->text().toStdString();
	address[ReadDataManager::Planning].port = m_pEditPlanningPort->text().toShort();
	ReadDataManager::instance()->setServerAddress(address, ReadDataManager::Count);
	this->accept();
}

/*******************************************************
 * @brief Button CANCEL clicked响应槽函数
 * @param

 * @return
********************************************************/
void QZmqSettingDialog::onBtnCancelClicked()
{
	this->reject();
}