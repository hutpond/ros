/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QReadDataObject.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 串口数据读取类
********************************************************/
#ifndef QREADDATAOBJECT_H
#define QREADDATAOBJECT_H

#include <functional>
#include <QObject>

class QSerialPort;

#pragma pack(push, 1)
struct DataPacket
{
  int8_t cHeader[3];   // header 0xBD 0xBD 0x0B

  int16_t sRoll;       // 360 / 32768 deg
  int16_t sPitch;      // 360 / 32768 deg
  int16_t sYaw;        // 360 / 32768 deg

  int16_t sGyroRotX;   // 300 / 32768 deg / s
  int16_t sGyroRotY;   // 300 / 32768 deg / s
  int16_t sGyroRotZ;   // 300 / 32768 deg / s

  int16_t sAccelX;     // 12 / 32768 g
  int16_t sAccelY;     // 12 / 32768 g
  int16_t sAccelZ;     // 12 / 32768 g

  int32_t nLatitude;   // 1e-7 deg
  int32_t nLongitude;  // 1e-7 deg
  int32_t nAltitude;   // 1e-3 m

  int16_t sVelNorth;   // 1e2 / 32768 m / s
  int16_t sVelEast;    // 1e2 / 32768 m / s
  int16_t sVelUniver;  // 1e2 / 32768 m / s

  uint8_t ucStatus;    // bit 0-3 location velocity
  int8_t cNotUsed[6];  // not used
  int16_t sData[3];    // data
  uint32_t unGpsTime;  // 2.5e-4 second
  uint8_t ucType;
  uint8_t ucParity;    // check sum xor(0 - 56)

  inline double getAngleRoll() const
  {
    return this->sRoll * 360.0 / 32768;
  }

  inline double getAnglePitch() const
  {
    return this->sPitch * 360.0 / 32768;
  }

  inline double getAngleYaw() const
  {
    return this->sYaw * 360.0 / 32768;
  }

  inline double getGyroAngleVelocityX() const
  {
    return this->sGyroRotX * 300.0 / 32768;
  }

  inline double getGyroAngleVelocityY() const
  {
    return this->sGyroRotY * 300.0 / 32768;
  }

  inline double getGyroAngleVelocityZ() const
  {
    return this->sGyroRotZ * 300.0 / 32768;
  }

  inline double getAcceleratedVelocityX() const
  {
    return this->sAccelX * 12.0 / 32768;
  }

  inline double getAcceleratedVelocityY() const
  {
    return this->sAccelY * 12.0 / 32768;
  }

  inline double getAcceleratedVelocityZ() const
  {
    return this->sAccelZ * 12.0 / 32768;
  }

  inline double getLongitude() const
  {
    return this->nLongitude * 1e-7;
  }

  inline double getLatitude() const
  {
    return this->nLatitude * 1e-7;
  }

  inline double getAltitude() const
  {
    return this->nAltitude * 1e-3;
  }

  inline double getVelocityX() const
  {
    return this->sVelNorth * 1e2 / 32768;
  }

  inline double getVelocityY() const
  {
    return this->sVelEast * 1e2 / 32768;
  }

  inline double getVelocityZ() const
  {
    return this->sVelUniver * 1e2 / 32768;
  }

  inline int getStatus() const
  {
    return this->ucStatus;
  }

  inline long long getGpsTime() const
  {
    return this->unGpsTime * 2.5e-4 * 1000;    //ms
  }
};
#pragma pack(pop)

class QReadDataObject : public QObject
{
  Q_OBJECT
public:
  explicit QReadDataObject(QObject *parent = nullptr);

  bool openSerialPort(const QString &, qint32, bool);
  void getSerailParam(QString &, qint32 &, bool &);
  void setSetDataFun(std::function<void(const DataPacket *)>);

signals:

protected:
  void decodeData();

private slots:
  void onReadData();

private:
  QSerialPort *m_pSerialPort;
  QString m_strDevice;
  qint32 m_nBaud;
  bool m_bSerialSelect;
  QByteArray m_baData;
  DataPacket m_dataPacket;

  std::function<void(const DataPacket *)> m_funSetData;
};

#endif // QREADDATAOBJECT_H
