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
#include <QVector3D>
#include <eigen3/Eigen/Core>
#include "path_editor/ads_ins_data.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"


class QLineEdit;
class QPushButton;
class QOpenDriveObject;
struct DataPacket;

struct ProjectInfo
{
  QString m_strProjectName;
};

constexpr double MAX_POS = 10000;

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

  apollo::hdmap::Map & mapData();
  const apollo::hdmap::Map & mapData() const;
  const QString & projectName();

protected slots:
  void onSetImuData(const path_editor::ads_ins_data::ConstPtr &);

protected:
  void createProjectFile(const QString &);

  QVector3D lla2Enu(const path_editor::ads_ins_data::ConstPtr &data);

  void createPointsList();
  void saveMapFile();
  void readMapFile();

  void addLineSegmentPoint(const path_editor::ads_ins_data::ConstPtr &data, bool = false);

signals:

public slots:

private:
  apollo::hdmap::Map m_mapData;
  Eigen::Vector3d m_vector3dOrigin;
  QVector3D m_prePoint;

  QString m_strPathName;
  QString m_strProName;
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
