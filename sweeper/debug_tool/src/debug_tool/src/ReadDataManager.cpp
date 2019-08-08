/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: ReadDataManager.cpp
 * Author: liuzheng
 * Date: 2019/6/25
 * Description: zmq数据读写类，所有相关数据由该类读取管理
 * History: 2019/7/3, 添加perception读取zmq数据类
********************************************************/
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <QSettings>
#include <boost/filesystem.hpp>
#include "zmq.h"
#include "ReadDataManager.h"
#include "GlobalDefine.h"

ReadDataManager ReadDataManager::ms_instance;

/*******************************************************
 * @brief instance获取函数，单例模式封装
 * @param

 * @return
********************************************************/
ReadDataManager * ReadDataManager::instance()
{
  return &ms_instance;
}

ReadDataManager::ReadDataManager()
  : QThread()
{
  namespace fs = boost::filesystem;
  m_fsPath = fs::current_path();
  m_fsPath /= "PlanningData";

  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
  m_fsPath /= ss.str();
  if (!fs::exists(m_fsPath)) {
    fs::create_directories(m_fsPath);
  }
}

ReadDataManager::~ReadDataManager()
{
}

/*******************************************************
 * @brief 开始所有读取zmq线程
 * @param
 
 * @return
********************************************************/
void ReadDataManager::startRead()
{
  m_pContext = zmq_ctx_new();
  m_pZsPlanning = zmq_socket(m_pContext, ZMQ_REP);
  zmq_bind(m_pZsPlanning, "tcp://*:5570");

  m_nFlagPlanningRead = 1;
  this->start();
}

/*******************************************************
 * @brief 停止zmq read线程
 * @param

 * @return
********************************************************/
void ReadDataManager::stopRead()
{
  m_nFlagPlanningRead = 0;
  zmq_close(m_pZsPlanning);
  zmq_ctx_term(m_pContext);
  this->wait();
}

/*******************************************************
 * @brief 规划模块数据处理线程函数，循环读取数据
 * @param

 * @return
********************************************************/
void ReadDataManager::run()
{
  while (m_nFlagPlanningRead == 1) {
    //接收消息;
    //setStatusMessage("start reveive json data");
    zmq_msg_t msgIn;
    zmq_msg_init(&msgIn);
    zmq_msg_recv(&msgIn, m_pZsPlanning, 0);
    if (m_nFlagPlanningRead == 0) break;
    char *pszRecv = static_cast<char*>(zmq_msg_data(&msgIn));
    std::string strRecv(pszRecv);
    zmq_msg_close(&msgIn);

    setStatusMessage("save json to file");
    this->saveJsonFile(strRecv);

    Json::Value jsonRoot;
    Json::Reader reader;
    bool bFlagRecv = false;
    if (reader.parse(strRecv, jsonRoot)) {
      bFlagRecv = true;
      Json::Value jsonType = jsonRoot["TYPE"];
      Json::Value jsonData = jsonRoot["DATA"];
      if (jsonType.isString() && jsonType.asString() == "MSG_PLANNING") {
        emit planningJson(jsonData);
      }
    }
    else {
      setStatusMessage("parse json data failed", 3);
    }

    //发送消息;
    //setStatusMessage("send response json data................");
    Json::Value jsonRep;
    jsonRep["TYPE"] = bFlagRecv ? "REP_OK" : "REP_FALSE";
    Json::FastWriter writer;
    std::string strRep = writer.write(jsonRep);
    size_t length = strRep.length();
    zmq_msg_t msgRep;
    zmq_msg_init_size(&msgRep, length);
    memcpy(zmq_msg_data(&msgRep), strRep.c_str(), length);
    zmq_msg_send(&msgRep, m_pZsPlanning, 0);
    zmq_msg_close(&msgRep);
  }
}

/*******************************************************
 * @brief 设置是否保存json到文件
 * @param save: true, 保存; false, 不保存

 * @return
********************************************************/
void ReadDataManager::setSaveJson(bool save)
{
  //m_nFlagSaveJson = save ? 1 : 0;
}

/*******************************************************
 * @brief 保存json到文件
 * @param json: json字符串

 * @return
********************************************************/
void ReadDataManager::saveJsonFile(const std::string &json)
{
  fs::path path = m_fsPath;

  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  auto duration_now = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
  int ms = duration_now.count() % 1000;

  std::time_t now_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S_") << ms;
  std::string strFileName = ss.str() + ".txt";
  path /= strFileName;
  strFileName = path.string();

  std::ofstream out(strFileName);
  out << json;
  out.close();
}

/*******************************************************
 * @brief 测试函数，从文件中读取json数据
 * @param

 * @return
********************************************************/
void ReadDataManager::testFromFile()
{
  namespace fs = boost::filesystem;
  fs::path path = boost::filesystem::current_path();
  path /= "JsonDataTest";
  if (!fs::exists(path)) {
    return;
  }

  for (auto &it : fs::directory_iterator(path)) {
    std::stringstream ss;
    ss << it;
    std::string strFileName = ss.str();
    std::ifstream in(strFileName);
    std::string strRecv((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());
    in.close();

    Json::Value jsonRoot;
    Json::Reader reader;
    if (reader.parse(strRecv, jsonRoot)) {
      Json::Value jsonType = jsonRoot["TYPE"];
      Json::Value jsonData = jsonRoot["DATA"];
      std::string str = jsonType.asString();
      if (jsonType.isString() && jsonType.asString() == "MSG_PLANNING") {
        emit planningJson(jsonData);
      }
    }
    else {
      setStatusMessage("parse json data failed", 3);
    }
    sleep(1);
  }
}


QPerceptionData QPerceptionData::ms_instance(nullptr);
QPerceptionData * QPerceptionData::instance()
{
  return &ms_instance;
}

QPerceptionData::QPerceptionData(QObject *parent)
  : QThread(parent)
{

}

QPerceptionData::~QPerceptionData()
{

}

/*******************************************************
 * @brief 开始读取zmq线程
 * @param

 * @return
********************************************************/
void QPerceptionData::startRead()
{
  m_pContext = zmq_ctx_new();
  m_pZsPlanning = zmq_socket(m_pContext, ZMQ_REP);
  zmq_bind(m_pZsPlanning, "tcp://*:7777");

  m_nFlagPerceptionRead = 1;
  this->start();
}

/*******************************************************
 * @brief 停止zmq read线程
 * @param

 * @return
********************************************************/
void QPerceptionData::stopRead()
{
  m_nFlagPerceptionRead = 0;
  zmq_close(m_pZsPlanning);
  zmq_ctx_term(m_pContext);
  this->wait();
}

void QPerceptionData::run()
{
  while (m_nFlagPerceptionRead == 1) {

    //接收消息;
    //setStatusMessage("start reveive json data");
    zmq_msg_t msgIn;
    zmq_msg_init(&msgIn);
    zmq_msg_recv(&msgIn, m_pZsPlanning, 0);
    if (m_nFlagPerceptionRead == 0) break;
    char *pszRecv = static_cast<char*>(zmq_msg_data(&msgIn));
    std::string strRecv(pszRecv);
    zmq_msg_close(&msgIn);

    Json::Value jsonRoot;
    Json::Reader reader;
    bool bFlagRecv = false;
    if (reader.parse(strRecv, jsonRoot)) {
      bFlagRecv = true;
      Json::Value jsonType = jsonRoot["TYPE"];
      Json::Value jsonData = jsonRoot["DATA"];
      if (jsonType.isString() && jsonType.asString() == "PUSH_UPDATE") {
        emit perceptionJson(jsonData);
      }
    }
    else {
      setStatusMessage("parse json data failed", 3);
    }

    //发送消息;
    //setStatusMessage("send response json data................");
    Json::Value jsonRep;
    jsonRep["TYPE"] = bFlagRecv ? "REP_OK" : "REP_FALSE";
    Json::FastWriter writer;
    std::string strRep = writer.write(jsonRep);
    size_t length = strRep.length();
    zmq_msg_t msgRep;
    zmq_msg_init_size(&msgRep, length);
    memcpy(zmq_msg_data(&msgRep), strRep.c_str(), length);
    zmq_msg_send(&msgRep, m_pZsPlanning, 0);
    zmq_msg_close(&msgRep);
  }
}
