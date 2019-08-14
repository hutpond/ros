/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QReadDataObject.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 串口数据读取类
********************************************************/
#include <QtSerialPort/QSerialPort>
#include "QReadDataObject.h"

QReadDataObject::QReadDataObject(QObject *parent)
  : QObject(parent)
  , m_strDevice("")
  , m_nBaud(0)
  , m_bSerialSelect(false)
{
  m_pSerialPort = new QSerialPort(this);
  connect(m_pSerialPort, &QSerialPort::readyRead, this, &QReadDataObject::onReadData);
  m_pSerialPort->setDataBits(QSerialPort::Data8);
  m_pSerialPort->setStopBits(QSerialPort::OneStop);
  m_pSerialPort->setParity(QSerialPort::NoParity);
  //m_pSerialPort->setBreakEnabled(true);
  //m_pSerialPort->setDataTerminalReady(true);
  //m_pSerialPort->setFlowControl(QSerialPort::NoFlowControl);
  //m_pSerialPort->setReadBufferSize(1024);
  //m_pSerialPort->setRequestToSend(true);
}

/**
 * @brief 设置串口名称、波特率，并打开串口
 * @param device: 串口名
 * @param baud: 波特率
 *
 * @return true, 设置并打开成功
 */
bool QReadDataObject::openSerialPort(const QString &device, qint32 baud, bool select)
{
  if (m_strDevice == device && m_nBaud == baud && m_bSerialSelect == select) {
    return true;
  }
  m_bSerialSelect = select;
  m_strDevice = device;
  m_nBaud = baud;
  if (!m_bSerialSelect) {
    return false;
  }
  if (m_pSerialPort->isOpen()) {
    m_pSerialPort->close();
  }
  m_pSerialPort->setPortName(m_strDevice);
  if (!m_pSerialPort->setBaudRate(m_nBaud, QSerialPort::AllDirections)) {
    return false;
  }
  return m_pSerialPort->open(QIODevice::ReadOnly);
}

/**
 * @brief 获取串口参数
 *
 * @return
 */
void QReadDataObject::getSerailParam(QString &device, qint32 &baud, bool &select)
{
  device = m_strDevice;
  baud = m_nBaud;
  select = m_bSerialSelect;
}

/**
 * @brief 设置回调函数
 * @param fun: 回调函数
 *
 * @return
 */
void QReadDataObject::setSetDataFun(std::function<void(const DataPacket *)> fun)
{
  m_funSetData = fun;
}

/**
 * @brief 读串口槽函数
 *
 * @return
 */
void QReadDataObject::onReadData()
{
  qint64 length = m_pSerialPort->bytesAvailable();
  m_baData.append(m_pSerialPort->read(length));
  const int LENGTH = sizeof(DataPacket);
  while (m_baData.size() >= LENGTH) {
    this->decodeData();
  }
}

/**
 * @brief 解析串口数据
 *
 * @return
 */
void QReadDataObject::decodeData()
{
  // find header
  constexpr char HEADER[3] = {char(0xBD), char(0xBD), char(0x0B)};
  int dataLen = m_baData.size();
  int index = -1;
  for (int i = 0; i < dataLen - 3; ++i) {
    if (m_baData[i] == HEADER[i] && m_baData[i+1] == HEADER[i+1] && m_baData[i+2] == HEADER[i+2]) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    m_baData = m_baData.right(2);
    return;
  }
  else if (index > 0) {
    m_baData = m_baData.right(dataLen - index);
  }
  // length
  dataLen = m_baData.size();
  const int LENGTH = sizeof(DataPacket);
  if (dataLen < LENGTH) return;
  // check
  uint8_t *pData = (uint8_t *)(m_baData.data());
  uint8_t sum = pData[0];
  for (int i = 1; i < LENGTH - 1; ++i) {
    sum = sum ^ pData[i];
  }
  if (sum == pData[LENGTH - 1])
  {
    // copy data
    memcpy(&m_dataPacket, m_baData.data(), LENGTH);
    m_funSetData(&m_dataPacket);
  }
  // offset data
  if (dataLen > LENGTH) {
    m_baData = m_baData.right(dataLen - LENGTH);
  }
  else {
    m_baData.clear();
  }
}
