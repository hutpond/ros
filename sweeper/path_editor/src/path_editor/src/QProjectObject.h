/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QProjectObject.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 工程管理类
********************************************************/
#ifndef QPROJECTOBJECT_H
#define QPROJECTOBJECT_H

#include <iostream>
#include <QObject>
#include <QDialog>
#include <QtMath>
#include <eigen3/Eigen/Core>
#include "path_editor/ads_ins_data.h"

class QLineEdit;
class QPushButton;
class QOpenDriveObject;
struct DataPacket;

struct ProjectInfo
{
  QString m_strProjectName;
};

constexpr double MAX_POS = 10000;

struct InfoPacket
{
  double  m_dPitch; // degree, up,   [-90,  90]
  double  m_dRoll;  // degree, right,    [-180, 180]
  double  m_dYaw;   // degree, north to east,   [0,  360]

  double m_dGyroRotX;   // deg / s
  double m_dGyroRotY;   // deg / s
  double m_dGyroRotZ;   // deg / s

  double m_dAccelX;     // g
  double m_dAccelY;     // g
  double m_dAccelZ;     // g

  double  m_dLatitude;   // degree
  double  m_dLongitude;  // degree
  double  m_dAltitude;   // m

  double m_dVelNorth;   // m / s
  double m_dVelEast;    // m / s
  double m_dVelUniver;  // m / s

  long long  m_llMsGps; // GPS time ms
  char  m_cGpsState;
  char	m_cUpdateFlag; // flag of update，1: update, 0: not update
};

struct MapBinData
{
  int id;
  double x;
  double y;
  double z;

  double pitch;
  double roll;
  double yaw;

  double lat;
  double lon;
  double alt;

  bool clear;

  MapBinData()
    : x(0), y(0), z(0)
    , pitch(0), roll(0), yaw(0)
    , lat(0), lon(0), alt(0)
    , clear(false)
  {}

  MapBinData(double _x, double _y, double _z,
             double lat_, double lon_, double alt_,
             double pitch_, double roll_, double yaw_)
  {
    x = _x;
    y = _y;
    z = _z;
    lat = lat_;
    lon = lon_;
    alt = alt_;
    pitch = pitch_;
    roll = roll_;
    yaw = yaw_;
    clear = false;
  }

  MapBinData & operator=(const MapBinData &rhs) {
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
    this->lat = rhs.lat;
    this->lon = rhs.lon;
    this->alt = rhs.alt;
    this->pitch = rhs.pitch;
    this->roll = rhs.roll;
    this->yaw = rhs.yaw;
    return *this;
  }
  double distance(const MapBinData &data) {
    return qSqrt(
          (this->x - data.x) * (this->x - data.x) +
          (this->y - data.y) * (this->y - data.y) +
          (this->z - data.z) * (this->z - data.z)
          );
  }
  double angle() {
    QLineF line(0, 0, 0, 10);
    QLineF line2(0, 0, this->x, this->y);
    return line.angleTo(line2);
  }

};

class QProjectObject : public QObject
{
  Q_OBJECT
public:
  explicit QProjectObject(QObject *parent = nullptr);
  void createProject(const QString &, const QString &);
  void openProject(const QString &);
  void stopProject();
  void closeProject();
  void saveProject();
  void buildProject();
  void setImuData(const DataPacket *);

  const QList<QSharedPointer<MapBinData>> & getPathPoints() const;
  QList<QSharedPointer<MapBinData>> & getPathPoints();

protected slots:
  void onSetImuData(const path_editor::ads_ins_data::ConstPtr &);

protected:
  void createProjectFile(const QString &);

  Eigen::Matrix3d Mat_n2e(const Eigen::Vector3d &);
  Eigen::Vector3d Blh2Xyz(const Eigen::Vector3d &);
  Eigen::Vector3d Ecef2enu(const Eigen::Vector3d &, const Eigen::Vector3d &);

  void createPointsList();
  void saveImuData(const InfoPacket &);

signals:

public slots:

private:
  QOpenDriveObject *m_pObjOpenDrive;
  QList<QSharedPointer<MapBinData>> m_listReferensePoints;
  Eigen::Vector3d m_vector3dOrigin;

  QString m_strPathName;
  QString m_strProName;

  QString m_strImuPath;
  int m_nImuFileIndex;
};

class QCreateProjectDialog : public QDialog
{
  Q_OBJECT

public:
  QCreateProjectDialog(QWidget *);
  void getProjectName(QString &, QString &);

protected slots:
  void onBtnOk();
  void onBtnBrowser();

private:
  QLineEdit *m_pEditProjectName;
  QLineEdit *m_pEditProjectPath;
  QPushButton *m_pBtnBrowser;

  QPushButton *m_pBtnOK;
  QPushButton *m_pBtnCancel;
};

#endif // QPROJECTOBJECT_H
