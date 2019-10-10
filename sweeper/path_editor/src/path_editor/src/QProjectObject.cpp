/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QProjectObject.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 工程管理类
********************************************************/
#include <memory>
#include <fstream>
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
#include <QMessageBox>
#include "QProjectObject.h"
#include "gps.h"


QProjectObject::QProjectObject(QObject *parent)
  : QObject(parent)
  , m_strPathName("")
  , m_strProName("")
{
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

  this->closeProject();

  QString filename = subpath;
  filename.append(name);
  filename.append(".mpro");
  this->createProjectFile(filename);

  m_strPathName = subpath;
  m_strProName = name;
  m_vector3dOrigin = Eigen::Vector3d(10000, 10000, 10000);
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

  this->readMapFile();
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
  m_mapData.Clear();
}

/**
 * @brief 停止保存工程
 * @param
 *
 * @return
 */
void QProjectObject::stopProject()
{
  this->saveMapFile();
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
  this->saveMapFile();
}

void QProjectObject::buildProject()
{
}

void QProjectObject::onSetImuData(const path_editor::ads_ins_data::ConstPtr &data)
{
  if (m_strProName.isEmpty()) {
    return;
  }
  if (m_vector3dOrigin(0) > 360) {
    m_vector3dOrigin = Eigen::Vector3d(data->lon, data->lat, data->height);
    this->addLineSegmentPoint(data, true);
  }
  this->addLineSegmentPoint(data);
}

QVector3D QProjectObject::lla2Enu(const path_editor::ads_ins_data::ConstPtr &data)
{
  GpsTran gps_tran(m_vector3dOrigin(0), m_vector3dOrigin(1), m_vector3dOrigin(2));

  GpsDataType gps;
  NedDataType ned;
  gps.longitude = data->lon;
  gps.latitude  = data->lat;
  gps.altitude  = data->height;
  gps_tran.fromGpsToNed(ned, gps);

  QVector3D enu;
  enu.setX(ned.y_east);
  enu.setY(ned.x_north);
  enu.setZ(-ned.z_down);

  return enu;
}

const QString & QProjectObject::projectName()
{
  return m_strProName;
}

apollo::hdmap::Map & QProjectObject::mapData()
{
  return m_mapData;
}

const apollo::hdmap::Map & QProjectObject::mapData() const
{
  return m_mapData;
}

void QProjectObject::saveMapFile()
{
  QString fileName = m_strPathName + m_strProName + ".bin";

  std::fstream output(fileName.toStdString(), std::ios::out | std::ios::binary);
  m_mapData.SerializePartialToOstream(&output);
  output.close();
  output.sync();
}

void QProjectObject::readMapFile()
{
  m_mapData.Clear();
  QString fileName = m_strPathName + m_strProName + ".bin";
  std::fstream input(fileName.toStdString(), std::ios::in | std::ios::binary);
  m_mapData.ParseFromIstream(&input);
  input.close();
}

void QProjectObject::addLineSegmentPoint(const path_editor::ads_ins_data::ConstPtr &data, bool force)
{
  constexpr double MAX_DIS = 0.1;
  QVector3D enu = this->lla2Enu(data);
  if (!force && m_prePoint.distanceToPoint(enu) < MAX_DIS) {
    return;
  }

  const int size_lane = m_mapData.lane_size();
  if (size_lane == 0) {
    return;
  }
  apollo::hdmap::Lane *lane = m_mapData.mutable_lane(0);
  if (lane == nullptr) {
    return;
  }
  apollo::hdmap::CurveSegment *curve_segment = lane->mutable_central_curve()->mutable_segment(0);
  if (curve_segment == nullptr) {
    return;
  }
  apollo::hdmap::LineSegment *line_segment = curve_segment->mutable_line_segment();
  if (line_segment == nullptr) {
    return;
  }
  apollo::common::PointENU* pt = line_segment->add_point();
  pt->set_x(enu.x());
  pt->set_y(enu.y());
  pt->set_z(enu.z());
  pt->set_lon(data->lon);
  pt->set_lat(data->lat);
  pt->set_alt(data->height);
  pt->set_pitch(data->pitch);
  pt->set_roll(data->roll);
  pt->set_yaw(data->yaw);

  m_prePoint = enu;
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
  hLayout->addWidget(m_pBtnCancel, 1);
  hLayout->addStretch(1);
  hLayout->addWidget(m_pBtnOK, 1);
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
    int ret = QMessageBox::warning(
          this, tr("create project"),
          tr("工程已存在，是否覆盖?"),
          QMessageBox::Yes | QMessageBox::No,
          QMessageBox::No);
    if (ret != QMessageBox::Yes) return;

    dir.setPath(path);
    dir.removeRecursively();
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
