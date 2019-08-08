/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QProjectObject.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 工程管理类
********************************************************/
#include <memory>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QDir>
#include <QFileDialog>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QtMath>
#include <QMatrix4x4>
#include "QProjectObject.h"
#include "QReadDataObject.h"
#include "QOpenDriveObject.h"

QProjectObject::QProjectObject(QObject *parent)
  : QObject(parent)
  , m_strPathName("")
  , m_strProName("")
{
  m_pObjOpenDrive = new QOpenDriveObject(this);
}

/**
 * @brief 创建工程路径和工程文件
 * @param path: 工程保存路径
 * @param name: 工程名
 *
 * @return
 */
void QProjectObject::createProject(const QString &path, const QString &name)
{
  QString subpath = path;
  if (!subpath.endsWith('/')) subpath.append('/');
  subpath.append(name);
  if (!subpath.endsWith('/')) subpath.append('/');
  QDir dir(path);
  if (!dir.exists()) {
    dir.mkpath(path);
  }
  QDir subdir(subpath);
  if (subdir.exists(subpath)) {
    subdir.removeRecursively();
  }
  dir.mkpath(subpath);

  QString filename = subpath;
  filename.append(name);
  filename.append(".swe");
  this->createProjectFile(filename);

  m_strPathName = subpath;
  m_strProName = name;
  m_vector3dOrigin = Eigen::Vector3d(10000, 10000, 10000);

#ifdef TEST
  this->createPointsList();
#endif
}

/**
 * @brief 打开工程
 * @param name: 工程文件名
 *
 * @return
 */
void QProjectObject::openProject(const QString &name)
{
  m_strPathName = name;
  if (!m_strPathName.endsWith('/')) m_strPathName.append('/');
  int index = m_strPathName.lastIndexOf('/', m_strPathName.length() - 2);
  m_strProName = m_strPathName.mid(index + 1);
  m_strProName.chop(1);
  m_vector3dOrigin = Eigen::Vector3d(10000, 10000, 10000);

  m_listReferensePoints.clear();
  m_pObjOpenDrive->readOpenDriveFile(m_strPathName, m_strProName, m_listReferensePoints);
}

/**
 * @brief 创建工程文件
 * @param name: 工程文件名
 *
 * @return
 */
void QProjectObject::createProjectFile(const QString &name)
{
  QFile file(name);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return;

  QJsonObject root;
  root["version"] = "1.0";
  QJsonObject item;
  item["id"] = 0;
  item["distance"] = 0;
  QJsonArray road;
  road.append(item);
  root["road"] = road;

  QJsonDocument saveDoc(root);
  file.write(saveDoc.toJson());

  file.close();
}

/**
 * @brief 停止保存工程
 * @param
 *
 * @return
 */
void QProjectObject::stopProject()
{
  m_pObjOpenDrive->writeOpenDriveFile(m_strPathName, m_strProName, m_listReferensePoints);

  m_listReferensePoints.clear();
  m_strPathName.clear();
  m_strProName.clear();
  m_vector3dOrigin = Eigen::Vector3d(10000, 10000, 10000);
}

/**
 * @brief 关闭工程
 * @param
 *
 * @return
 */
void QProjectObject::closeProject()
{
  this->stopProject();
}

void QProjectObject::saveProject()
{
  m_pObjOpenDrive->writeOpenDriveFile(m_strPathName, m_strProName, m_listReferensePoints);
}

/**
 * @brief 添加数据
 * @param data: IMU数据
 *
 * @return
 */
void QProjectObject::setImuData(const DataPacket *data)
{
  if (m_strProName.isEmpty()) return;
  std::shared_ptr<InfoPacket> pInfo(new InfoPacket);

  pInfo->m_llMsGps = data->getGpsTime();
  pInfo->m_dAltitude = data->getAltitude();
  pInfo->m_dLatitude = data->getLatitude();
  pInfo->m_dLongitude = data->getLongitude();

  pInfo->m_dPitch = data->getAnglePitch();
  pInfo->m_dRoll = data->getAngleRoll();
  pInfo->m_dYaw = data->getAngleYaw();

  pInfo->m_dVelEast = data->getVelocityX();
  pInfo->m_dVelNorth = data->getVelocityY();
  pInfo->m_dVelUniver = data->getVelocityZ();

  pInfo->m_dGyroRotX = data->getGyroAngleVelocityX();
  pInfo->m_dGyroRotY = data->getGyroAngleVelocityY();
  pInfo->m_dGyroRotZ = data->getGyroAngleVelocityZ();

  pInfo->m_dAccelX = data->getAcceleratedVelocityX();
  pInfo->m_dAccelY = data->getAcceleratedVelocityY();
  pInfo->m_dAccelZ = data->getAcceleratedVelocityZ();

  pInfo->m_cGpsState = data->getStatus();
  pInfo->m_cUpdateFlag = 1;

  if (m_vector3dOrigin(0) > 360) {
    m_vector3dOrigin = Eigen::Vector3d(pInfo->m_dLatitude, pInfo->m_dLongitude, pInfo->m_dAltitude);
  }
  Eigen::Vector3d vector3d(pInfo->m_dLatitude, pInfo->m_dLongitude, pInfo->m_dAltitude);
  vector3d = this->Blh2Xyz(vector3d);
  vector3d = this->Ecef2enu(vector3d, m_vector3dOrigin);

  QSharedPointer<Point> point(new Point(vector3d(0), vector3d(1), vector3d(2)));
  if ( m_listReferensePoints.size() == 0) {
    m_listReferensePoints.push_back(point);
  }
  else{
    const auto &last = m_listReferensePoints.at(m_listReferensePoints.size() - 1);
    if (!last->equal(*point)) {
      m_listReferensePoints.push_back(point);
    }
  }
}

/**
 * @brief 地球坐标系转换为东北天标系的矩阵
 * @param pos: 原点经度、纬度、高度
 *
 * @return 转换矩阵
 */
Eigen::Matrix3d QProjectObject::Mat_n2e(const Eigen::Vector3d &pos)
{
  double si = std::sin(pos(0)), ci = std::cos(pos(0)),
      sj = std::sin(pos(1)), cj = std::cos(pos(1));
  Eigen::Matrix3d mat;
  mat <<  -sj, -si*cj,  ci*cj,
      cj, -si*sj,  ci*sj,
      0,   ci,     si ;  //Cen
  return mat;
}

/**
 * @brief 经纬度转换为地球坐标系坐标
 * @param pos: 经度、纬度、高度
 *
 * @return 地球坐标系坐标
 */
Eigen::Vector3d QProjectObject::Blh2Xyz(const Eigen::Vector3d &blh)
{
  double glv_re = 6378137.0;
  double f=(1.0/298.257);
  double e = sqrt(2*f-f*f);
  double glv_e2 = e*e;
  double sB = sin(blh(0)), cB = cos(blh(0)),
      sL = sin(blh(1)), cL = cos(blh(1)),
      N = glv_re/sqrt(1-glv_e2*sB*sB);
  return Eigen::Vector3d((N+blh(2))*cB*cL, (N+blh(2))*cB*sL, (N*(1-glv_e2)+blh(2))*sB);
}

/**
 * @brief 地球坐标系转换为东北天标系坐标
 * @param pos_e: 地球坐标系坐标
 * @param blh: 原点经度、纬度、高度
 *
 * @return 东北天标系坐标
 */
Eigen::Vector3d QProjectObject::Ecef2enu(const Eigen::Vector3d &pos_e, const Eigen::Vector3d &blh)
{
  return ( (Mat_n2e(blh).transpose()) * pos_e );
}

void QProjectObject::createPointsList()
{
  constexpr int SIZE = 500;
  for (int i = 0; i < SIZE; ++i) {
    double s = 3 + 0.01 * i;
    double angle = i * (2 * 3.14159265 * 3 / SIZE);
    double x = s * qSin(angle);
    double y = s * qCos(angle);
    QSharedPointer<Point> pt(new Point(x, y, 0));
    m_listReferensePoints.append(pt);
  }
}

/**
 * @brief 获取路径点
 * @param
 *
 * @return 路径点列表
 */
const QList<QSharedPointer<Point>> & QProjectObject::getPathPoints() const
{
  return m_listReferensePoints;
}

QList<QSharedPointer<Point>> & QProjectObject::getPathPoints()
{
  return m_listReferensePoints;
}


QCreateProjectDialog::QCreateProjectDialog(QWidget *parent)
  : QDialog(parent)
{
  m_pEditProjectName = new QLineEdit(this);
  m_pEditProjectName->setText(QStringLiteral("NewProject"));
  m_pEditProjectPath = new QLineEdit(this);
  m_pEditProjectPath->setText(QDir::currentPath());
  m_pBtnBrowser = new QPushButton(QStringLiteral("Browser"), this);
  connect(m_pBtnBrowser, &QPushButton::clicked, this, &QCreateProjectDialog::onBtnBrowser);

  m_pBtnOK = new QPushButton(QStringLiteral("OK"), this);
  m_pBtnCancel = new QPushButton(QStringLiteral("CACEL"), this);
  connect(m_pBtnOK, &QPushButton::clicked, this, &QCreateProjectDialog::onBtnOk);
  connect(m_pBtnCancel, &QPushButton::clicked, this, &QCreateProjectDialog::reject);

  QVBoxLayout *vLayout = new QVBoxLayout;
  QHBoxLayout *hLayout = new QHBoxLayout;
  hLayout->addWidget(new QLabel("Name"), 1);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pEditProjectName);
  hLayout->addStretch(4);
  vLayout->addLayout(hLayout);

  hLayout = new QHBoxLayout;
  hLayout->addWidget(new QLabel("Path"), 1);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pEditProjectPath);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pBtnBrowser);
  hLayout->addStretch(2);
  vLayout->addLayout(hLayout);

  hLayout = new QHBoxLayout;
  hLayout->addStretch(1);
  hLayout->addWidget(m_pBtnOK, 1);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pBtnCancel, 1);
  hLayout->addStretch(1);
  vLayout->addLayout(hLayout);

  this->setLayout(vLayout);
}

/**
 * @brief 获取工程路径、工程名
 * @param name: 工程路径
 * @param name: 工程名
 *
 * @return
 */
void QCreateProjectDialog::getProjectName(QString &path, QString &name)
{
  path = m_pEditProjectPath->text();
  name = m_pEditProjectName->text();
}

/**
 * @brief ok button clicked 响应槽函数
 * @param
 *
 * @return
 */
void QCreateProjectDialog::onBtnOk()
{
  QDir dir;
  QString path = m_pEditProjectPath->text();
  QString name = m_pEditProjectName->text();
  if (path.isEmpty() || name.isEmpty()) {
    return;
  }
  if (!dir.exists(path)) {
    return;
  }
  if (!path.endsWith('/')) {
    path.append("/");
  }
  path.append(name);
  if (dir.exists(path)) {
    return;
  }

  return QDialog::accept();
}

/**
 * @brief browser button clicked 响应槽函数
 * @param
 *
 * @return
 */
void QCreateProjectDialog::onBtnBrowser()
{
  QString dir = QFileDialog::getExistingDirectory(this->parentWidget(), tr("Create Project"),
                                                  "/home",
                                                  QFileDialog::ShowDirsOnly
                                                  | QFileDialog::DontResolveSymlinks);
  m_pEditProjectPath->setText(dir);
}
