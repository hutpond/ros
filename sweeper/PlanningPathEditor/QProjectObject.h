/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QProjectObject.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 工程管理类
********************************************************/
#ifndef QPROJECTOBJECT_H
#define QPROJECTOBJECT_H

#include <QObject>
#include <QDialog>
#include <QtMath>
#include <eigen3/Eigen/Core>

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

  Point()
    : x(0), y(0), z(0)
  {}
  Point(double _x, double _y, double _z)
  {
    x = _x;
    y = _y;
    z = _z;
  }
  Point & operator=(const Point &rhs) {
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;
    return *this;
  }
  bool equal(const Point &point) {
    constexpr double DIFF = 0.01;
    return ( (qAbs<double>(this->x - point.x) < DIFF) &&
             (qAbs<double>(this->y - point.y) < DIFF) &&
             (qAbs<double>(this->z - point.z) < DIFF) );
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
  void setImuData(const DataPacket *);

  const QList<QSharedPointer<Point>> & getPathPoints() const;
  QList<QSharedPointer<Point>> & getPathPoints();

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
