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
struct Point
{
  double x;
  double y;
  double z;

  double lat;
  double lon;
  double height;

  Point()
    : x(0), y(0), z(0)
    , lat(0), lon(0), height(0)
  {}
  Point(double _x, double _y, double _z, double lat_, double lon_, double height_)
  {
    x = _x;
    y = _y;
    z = _z;
    lat = lat_;
    lon = lon_;
    height = height_;
  }
  Point & operator=(const Point &rhs) {
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
    return *this;
  }
  double distance(const Point &point) {
    return qSqrt(
          (this->x - point.x) * (this->x - point.x) +
          (this->y - point.y) * (this->y - point.y) +
          (this->z - point.z) * (this->z - point.z)
          );
  }
  double angle() {
    QLineF line(0, 0, 0, 10);
    QLineF line2(0, 0, this->x, this->y);
    return line.angleTo(line2);
  }
};

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

typedef struct MapCellData_{
  float fX;         //地理坐标位置X
  float fY;         //地理坐标位置Y
  float fIntensityAvg;    //激光反射强度均值
  float fIntensitySigma;  //激光反射方差
  float fHeightAvg;       //高度均值
  float fHeightSigma;     //高度方差
  float fMaxIntensity;    //反射强度最大值
  float fMaxHeight;       //高度最大值
  double dLon;          //经度
  double dLat;          //纬度
  char chLaneId;        //所在的车道ID
}MapCellData;

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

  const QList<QSharedPointer<Point>> & getPathPoints() const;
  QList<QSharedPointer<Point>> & getPathPoints();

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
  QList<QSharedPointer<Point>> m_listReferensePoints;
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
