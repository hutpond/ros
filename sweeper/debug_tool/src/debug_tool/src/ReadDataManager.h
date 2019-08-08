/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: ReadDataManager.h
 * Author: liuzheng
 * Date: 2019/6/25
 * Description: zmq数据读写类，所有相关数据由该类读取管理
 * History: 2019/7/3, 添加perception读取zmq数据类
********************************************************/
#ifndef READ_DATA_MANAGER_H
#define READ_DATA_MANAGER_H

#include <QThread>
#include <QAtomicInt>
#include "jsoncpp/json/json.h"
#include "boost/filesystem.hpp"

class ReadDataManager : public QThread
{
  Q_OBJECT

public:
  static ReadDataManager * instance();

  void startRead();
  void stopRead();
  void setSaveJson(bool);

protected:
  void saveJsonFile(const std::string &);
  void testFromFile();

  virtual void run();

signals:
  void planningJson(const Json::Value &);

private:
  ReadDataManager();
  ~ReadDataManager();

  static ReadDataManager ms_instance;

  void *m_pContext;
  void *m_pZsPlanning;

  QAtomicInt m_nFlagPlanningRead;
  boost::filesystem::path m_fsPath;
};

class QPerceptionData : public QThread
{
  Q_OBJECT

public:
  static QPerceptionData * instance();

  void startRead();
  void stopRead();

protected:
  virtual void run();

signals:
  void perceptionJson(const Json::Value &);

private:
  explicit QPerceptionData(QObject *);
  ~QPerceptionData();

  static QPerceptionData ms_instance;

  void *m_pContext;
  void *m_pZsPlanning;

  QAtomicInt m_nFlagPerceptionRead;
};

#endif // READ_DATA_MANAGER_H

