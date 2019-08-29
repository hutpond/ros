/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/25
 * Description: Perception模块显示画面
********************************************************/
#include <iomanip>
#include <sstream>
#include <QFile>
#include <QTimerEvent>
#include "QPerceptionWidget.h"
#include "QPerceptionShowWidget.h"
#include "QPerceptionShow3DWidget.h"
#include "QPerceptionParamWidget.h"
#ifdef WIN64
# include "ReadDataManager.h"
#else
# include "QReadDataManagerRos.h"
#endif
#include "GlobalDefine.h"
#include "QDataDisplayDialog.h"
//#include "ads_perception_export.h"

QPerceptionWidget::QPerceptionWidget(QWidget *parent)
  : QBaseWidget(parent)
  , m_nSubShowIndex(PerceptionNone)
{
  //m_nPointCount = 0;
  //m_cloundPoints = new Point3D[16 * 2000 * 4];

  //m_fspath = createPathInCurrent("PerceptionData");
  m_pWdgShow = new QPerceptionShowWidget(this);
  m_pWdgShow3d = new QPerceptionShow3DWidget(this);
  m_pWdgParam = new QPerceptionParamWidget(this);
  connect(m_pWdgParam, &QPerceptionParamWidget::replayState,
          this, &QPerceptionWidget::onReplayState);
  connect(m_pWdgParam, &QPerceptionParamWidget::replayFrameOffset,
          this, &QPerceptionWidget::onSetFrameIndexReplay);
  connect(m_pWdgParam, &QPerceptionParamWidget::displayData,
          this, &QPerceptionWidget::onDisplayData);

#ifdef WIN64
  connect(QPerceptionData::instance(), SIGNAL(perceptionJson(const Json::Value &)),
          this, SLOT(onParseJsonData(const Json::Value &)), Qt::BlockingQueuedConnection);
  QPerceptionData::instance()->startRead();
#else
//  connect(QReadDataManagerRos::instance(), SIGNAL(perceptionJson(const Json::Value &)),
//          this, SLOT(onParseJsonData(const Json::Value &)), Qt::BlockingQueuedConnection);
#endif
}

QPerceptionWidget::~QPerceptionWidget()
{
//  if (m_cloundPoints) {
//    delete [] m_cloundPoints;
//    m_nPointCount = 0;
//  }
}

/*******************************************************
 * @brief 停止read perception data和显示线程
 * @param

 * @return
********************************************************/
void QPerceptionWidget::stopDisplay()
{
#ifdef WIN64
  QPerceptionData::instance()->stopRead();
#endif
}

void QPerceptionWidget::resizeEvent(QResizeEvent *)
{
  const int W_PERCENT = 65;
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  m_pWdgShow->setGeometry(
        0,
        0,
        WIDTH * W_PERCENT / 100,
        HEIGHT
        );
  m_pWdgShow3d->setGeometry(
        0,
        0,
        WIDTH * W_PERCENT / 100,
        HEIGHT
        );
  m_pWdgParam->setGeometry(
        WIDTH * W_PERCENT / 100,
        0,
        WIDTH * (100 - W_PERCENT) / 100,
        HEIGHT
        );
}

void QPerceptionWidget::timerEvent(QTimerEvent *e)
{
  int id = e->timerId();
  if (id == m_nTimerId) {
    if (!this->isVisible() || m_bFlagPauseReplay || m_nShowType != Replay ||
        m_itFile == m_listPlanningFiles.end()) {
      return;
    }
    PerceptionData data;
    this->readFromFile(*m_itFile, &data);
    if (Perception3D == m_nSubShowIndex) {
      m_pWdgShow3d->setPerceptionData(&data);
    }
    else {
      m_nSubShowIndex = Perception2D;
      m_pWdgShow->setPerceptionData(&data);
    }
    m_pWdgParam->setPerceptionData(&data);
    m_pWdgParam->setFrameOffset(1);
    ++m_itFile;
  }
}

/*******************************************************
 * @brief 设置显示画面
 * @param type: Perception2D，2D显示; Perception3D, 3D显示

 * @return
********************************************************/
void QPerceptionWidget::showPerception(int index)
{
  m_nShowType = LiveDisplay;
  m_nSubShowIndex = index;
  this->show();
  if (index == Perception2D) {
    m_pWdgShow3d->hide();
    m_pWdgShow->show();
  }
  else {
    m_pWdgShow->hide();
    m_pWdgShow3d->show();
  }
  m_pWdgParam->show();
  m_pWdgParam->setShowType(m_nShowType);
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
}

/*******************************************************
 * @brief 缩放图像显示
 * @param index: -1， 缩小, 0, 复原, 1, 放大

 * @return
********************************************************/
void QPerceptionWidget::setViewResolution(int index)
{
  m_pWdgShow->setViewResolution(index);
}

/*******************************************************
 * @brief 解析zmq收到的json字符串
 * @param text: json字符串

 * @return
********************************************************/
void QPerceptionWidget::onParseJsonData(const Json::Value &perception)
{
  Json::Value jsonData = perception["DATA"];
  Json::Value jsonGps = perception["GPS"];
  Json::Value jsonDataOriginal = perception["ORIGINAL_SENSE"];
  Json::Value jsonDataSense = perception["SENSE"];

  PerceptionData data;

  // sweeper
  data.m_rectSweeper.m_fCenterX = g_rectfSweeper.center().x();
  data.m_rectSweeper.m_fCenterY = g_rectfSweeper.center().y();
  data.m_rectSweeper.m_fWidth = g_rectfSweeper.width();
  data.m_rectSweeper.m_fLength = g_rectfSweeper.height();

  // road side
  Json::Value jsonCurl = jsonDataOriginal["CURB"];
  Line &lineLeft = data.m_roadSide.m_lineLeft;
  Json::Value jsonLineLeft = jsonCurl["LEFT"];
  if (jsonLineLeft.size() >= 2) {
    lineLeft.m_p1.m_fX = jsonLineLeft[0]["X"].asFloat();
    lineLeft.m_p1.m_fY = jsonLineLeft[0]["Y"].asFloat();
    lineLeft.m_p2.m_fX = jsonLineLeft[1]["X"].asFloat();
    lineLeft.m_p2.m_fY = jsonLineLeft[1]["Y"].asFloat();
  }
  Line &lineRight = data.m_roadSide.m_lineRight;
  Json::Value jsonLineRight = jsonCurl["RIGHT"];
  if (jsonLineLeft.size() >= 2) {
    lineRight.m_p1.m_fX = jsonLineRight[0]["X"].asFloat();
    lineRight.m_p1.m_fY = jsonLineRight[0]["Y"].asFloat();
    lineRight.m_p2.m_fX = jsonLineRight[1]["X"].asFloat();
    lineRight.m_p2.m_fY = jsonLineRight[1]["Y"].asFloat();
  }

  // lidar orginal
  data.m_lidarOriginal.clear();
  Json::Value jsonLidarOriginal = jsonDataOriginal["LIDAR_OBJECTS"];
  const int SIZE = jsonLidarOriginal.size();
  for (int i = 0; i < SIZE; ++i) {
    Json::Value item = jsonLidarOriginal[i];
    Rect rect;
    rect.m_fCenterX = item["X"].asFloat();
    rect.m_fCenterY = item["Y"].asFloat();
    rect.m_fWidth = item["W"].asFloat();
    rect.m_fLength = item["L"].asFloat();
    rect.m_fAngle = item["A"].asFloat();
    data.m_lidarOriginal.push_back(rect);
  }

  // radar orginal
  data.m_radarOriginal.clear();
  Json::Value jsonRadarOriginal = jsonDataOriginal["RADAR_OBJECTS"];
  const int SIZE_RO = jsonRadarOriginal.size();
  for (int i = 0; i < SIZE_RO; ++i) {
    Json::Value item = jsonRadarOriginal[i];
    Rect rect;
    rect.m_fCenterX = item["X"].asFloat();
    rect.m_fCenterY = item["Y"].asFloat();
    rect.m_fWidth = item["W"].asFloat();
    rect.m_fLength = item["L"].asFloat();
    rect.m_fAngle = item["A"].asFloat();
    data.m_radarOriginal.push_back(rect);
  }

  // ultrasonic
  Json::Value jsonUltra = jsonDataOriginal["ULTRASOUND_OBJECTS"];
  const int SIZE_U = qMin<int>(ULTRA_NUM, jsonUltra.size());
  for (int i = 0; i < SIZE_U; ++i) {
    data.m_ultrasonic[i].m_fDistance = 0;
    data.m_ultrasonic[i].m_bTriggered = false;
    data.m_ultrasonic[i].m_nId = jsonUltra[i]["POSITION"].asInt();
    if (jsonUltra[i]["AVAILABLE"].asBool()) {
      data.m_ultrasonic[i].m_fDistance = jsonUltra[i]["DISTANCE"].asFloat();
      data.m_ultrasonic[i].m_bTriggered = (data.m_ultrasonic[i].m_fDistance >= 0.01);
    }
  }

  // lidar
  data.m_lidar.clear();
  Json::Value jsonLidar = jsonDataSense["LIDAR_OBJECTS"];
  const int SIZE_L = jsonLidar.size();
  for (int i = 0; i < SIZE_L; ++i) {
    Json::Value item = jsonLidar[i];
    Rect rect;
    rect.m_fCenterX = item["X"].asFloat();
    rect.m_fCenterY = item["Y"].asFloat();
    rect.m_fWidth = item["W"].asFloat();
    rect.m_fLength = item["L"].asFloat();
    rect.m_fAngle = item["A"].asFloat();
    data.m_lidar.push_back(rect);
  }

  // radar
  data.m_radar.clear();
  Json::Value jsonRadar = jsonDataSense["RADAR_OBJECTS"];
  const int SIZE_R = jsonRadar.size();
  for (int i = 0; i < SIZE_R; ++i) {
    Json::Value item = jsonRadar[i];
    Rect rect;
    rect.m_fCenterX = item["X"].asFloat();
    rect.m_fCenterY = item["Y"].asFloat();
    rect.m_fWidth = item["W"].asFloat();
    rect.m_fLength = item["L"].asFloat();
    rect.m_fAngle = item["A"].asFloat();
    data.m_radar.push_back(rect);
  }

  this->writeToFile(&data);
  if (this->isVisible() && m_nShowType == LiveDisplay) {
    if (m_nSubShowIndex == Perception2D) {
      m_pWdgShow->setPerceptionData(&data);
    }
    else if (m_nSubShowIndex == Perception3D) {
      m_pWdgShow3d->setPerceptionData(&data);
    }
    m_pWdgParam->setPerceptionData(&data);
  }
}

/*******************************************************
 * @brief 将数据保存到文件
 * @param data: 数据

 * @return
********************************************************/
void QPerceptionWidget::writeToFile(const PerceptionData *data)
{
  std::string strFileName = createFileName(m_fspath, ".percep");;

  QString fileName = QString::fromLocal8Bit(strFileName.c_str());
  QFile file(fileName);
  file.open(QIODevice::WriteOnly);
  QDataStream out(&file);

  // sweeper
  out << data->m_rectSweeper.m_fCenterX << data->m_rectSweeper.m_fCenterY <<
         data->m_rectSweeper.m_fWidth << data->m_rectSweeper.m_fLength <<
         data->m_rectSweeper.m_fAngle;

  // road side
  out << data->m_roadSide.m_lineLeft.m_p1.m_fX << data->m_roadSide.m_lineLeft.m_p1.m_fY <<
         data->m_roadSide.m_lineLeft.m_p2.m_fX << data->m_roadSide.m_lineLeft.m_p2.m_fY <<
         data->m_roadSide.m_lineRight.m_p1.m_fX << data->m_roadSide.m_lineRight.m_p1.m_fY <<
         data->m_roadSide.m_lineRight.m_p2.m_fX << data->m_roadSide.m_lineRight.m_p2.m_fY;

  // lidar original
  int number = static_cast<int>(data->m_lidarOriginal.size());
  out << number;
  for (int i = 0; i < number; ++i) {
    const Rect &it = data->m_lidarOriginal[i];
    out << it.m_fCenterX << it.m_fCenterY << it.m_fWidth << it.m_fLength <<	it.m_fAngle;
  }

  // radar original
  number = static_cast<int>(data->m_radarOriginal.size());
  out << number;
  for (int i = 0; i < number; ++i) {
    const Rect &it = data->m_radarOriginal[i];
    out << it.m_fCenterX << it.m_fCenterY << it.m_fWidth << it.m_fLength << it.m_fAngle;
  }

  // ultrasonic
  for (int i = 0; i < ULTRA_NUM; ++i) {
    out << data->m_ultrasonic[i].m_nId << data->m_ultrasonic[i].m_bTriggered <<
           data->m_ultrasonic[i].m_fDistance;
  }

  // lidar
  number = static_cast<int>(data->m_lidar.size());
  out << number;
  for (int i = 0; i < number; ++i) {
    const Rect &it = data->m_lidar[i];
    out << it.m_fCenterX << it.m_fCenterY << it.m_fWidth << it.m_fLength << it.m_fAngle;
  }

  // radar
  number = static_cast<int>(data->m_radar.size());
  out << number;
  for (int i = 0; i < number; ++i) {
    const Rect &it = data->m_radar[i];
    out << it.m_fCenterX << it.m_fCenterY << it.m_fWidth << it.m_fLength << it.m_fAngle;
  }

  file.close();
  setStatusMessage("write planning data to file");
}

/*******************************************************
 * @brief 从文件中读取数据
 * @param name: 文件名
 * @param data: 数据

 * @return
********************************************************/
void QPerceptionWidget::readFromFile(const std::string &name, PerceptionData *data)
{
  QString fileName = QString::fromLocal8Bit(name.substr(1, name.length() - 2).c_str());
  QFile file(fileName);
  file.open(QIODevice::ReadOnly);
  QDataStream in(&file);

  // sweeper
  in >> data->m_rectSweeper.m_fCenterX >> data->m_rectSweeper.m_fCenterY >>
      data->m_rectSweeper.m_fWidth >> data->m_rectSweeper.m_fLength >>
      data->m_rectSweeper.m_fAngle;

  // road side
  in >> data->m_roadSide.m_lineLeft.m_p1.m_fX >> data->m_roadSide.m_lineLeft.m_p1.m_fY >>
      data->m_roadSide.m_lineLeft.m_p2.m_fX >> data->m_roadSide.m_lineLeft.m_p2.m_fY >>
      data->m_roadSide.m_lineRight.m_p1.m_fX >> data->m_roadSide.m_lineRight.m_p1.m_fY >>
      data->m_roadSide.m_lineRight.m_p2.m_fX >> data->m_roadSide.m_lineRight.m_p2.m_fY;

  // lidar original
  int sizetLO;
  in >> sizetLO;
  data->m_lidarOriginal.clear();
  for (int i = 0; i < sizetLO; ++i) {
    Rect rect;
    in >> rect.m_fCenterX >> rect.m_fCenterY >> rect.m_fWidth >> rect.m_fLength >>
        rect.m_fAngle;
    data->m_lidarOriginal.push_back(rect);
  }

  // radar original
  int sizetRO;
  in >> sizetRO;
  data->m_radarOriginal.clear();
  for (int i = 0; i < sizetRO; ++i) {
    Rect rect;
    in >> rect.m_fCenterX >> rect.m_fCenterY >> rect.m_fWidth >> rect.m_fLength >>
        rect.m_fAngle;
    data->m_radarOriginal.push_back(rect);
  }

  // ultrasonic
  for (int i = 0; i < ULTRA_NUM; ++i) {
    in >> data->m_ultrasonic[i].m_nId >> data->m_ultrasonic[i].m_bTriggered >>
        data->m_ultrasonic[i].m_fDistance;
  }

  // lidar
  int sizetL;
  in >> sizetL;
  data->m_lidar.clear();
  for (int i = 0; i < sizetL; ++i) {
    Rect rect;
    in >> rect.m_fCenterX >> rect.m_fCenterY >> rect.m_fWidth >> rect.m_fLength >>
        rect.m_fAngle;
    data->m_lidar.push_back(rect);
  }

  // radar
  int sizetO;
  in >> sizetO;
  data->m_radar.clear();
  for (int i = 0; i < sizetO; ++i) {
    Rect rect;
    in >> rect.m_fCenterX >> rect.m_fCenterY >> rect.m_fWidth >> rect.m_fLength >>
        rect.m_fAngle;
    data->m_radar.push_back(rect);
  }

  file.close();
}

/*******************************************************
 * @brief 设置显示画面
 * @param type: LiveDisplay，实时显示; Replay, 回放

 * @return
********************************************************/
void QPerceptionWidget::showType(int type)
{
  if (!this->isVisible()) {
    m_nShowType = TypeNone;
  }
  this->show();
  if (m_nShowType != type) {
    m_nShowType = type;
    if (m_nShowType == LiveDisplay) {
      if (m_nTimerId != 0) {
        killTimer(m_nTimerId);
        m_nTimerId = 0;
      }
    }
    else {
      this->showPerception(m_nSubShowIndex == PerceptionNone ? Perception2D : m_nSubShowIndex);
      m_nShowType = Replay;
      this->replay("PerceptionData");
      m_pWdgParam->setFrameCount(m_listPlanningFiles.size());
    }
    m_pWdgParam->setShowType(type);
  }
}

/*******************************************************
 * @brief 暂停replay时，定位到帧
 * @param index: 距当前帧的帧数，负数向前，正数向后

 * @return
********************************************************/
void QPerceptionWidget::onSetFrameIndexReplay(int index)
{
  if (index < 0) {
    while (index < 0 && m_itFile != m_listPlanningFiles.begin()) {
      ++index;
      --m_itFile;
    }
  }
  else if (index > 0) {
    while (index > 0 && m_itFile != m_listPlanningFiles.end()) {
      --index;
      ++m_itFile;
    }
  }
  std::vector<std::string>::iterator it = m_itFile;
  if (it == m_listPlanningFiles.end()) {
    --it;
  }
  PerceptionData data;
  this->readFromFile(*it, &data);
  m_pWdgShow->setPerceptionData(&data);
  m_pWdgParam->setPerceptionData(&data);
}


/*******************************************************
 * @brief data button clicked响应槽函数
 * @param

 * @return
********************************************************/
void QPerceptionWidget::onDisplayData()
{
  if (m_pDlgDataDisplay == NULL) {
    m_pDlgDataDisplay = new QDataDisplayDialog(this);
    QRect rect = this->rect();
    QPoint pt = rect.center();
    rect.setSize(QSize(rect.width() * 0.8, rect.height() * 0.8));
    rect.moveCenter(pt);
    m_pDlgDataDisplay->setGeometry(rect);
    m_pDlgDataDisplay->setWindowTitle(QStringLiteral("Perception Data"));
  }

  std::vector<std::string>::iterator it = m_itFile;
  if (it == m_listPlanningFiles.end()) --it;
  //std::string text = this->createDispayText(*it);
  //QString strQtText = QString::fromUtf8(text.data());
  //m_pDlgDataDisplay->setText(strQtText);
  //m_pDlgDataDisplay->show();
}

/*******************************************************
 * @brief 从文件生成显示字符串
 * @param name: 文件名

 * @return: 显示字符串
********************************************************/
std::string QPerceptionWidget::createDispayText(const std::string &name)
{
  PerceptionData data;
  this->readFromFile(name, &data);

  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "Sweeper" << std::endl;
  ss << "Length: " << data.m_rectSweeper.m_fWidth << "  Width: " << data.m_rectSweeper.m_fLength <<
        std::endl << std::endl;

  ss << "RoadSide" << std::endl;
  ss << "Left P1: (" << data.m_roadSide.m_lineLeft.m_p1.m_fX << ", " <<
        data.m_roadSide.m_lineLeft.m_p1.m_fY << ")  " << "P2: (" <<
        data.m_roadSide.m_lineLeft.m_p2.m_fX << ", " <<
        data.m_roadSide.m_lineLeft.m_p2.m_fY << ")  Right P1: (" <<
        data.m_roadSide.m_lineRight.m_p1.m_fX << ", " <<
        data.m_roadSide.m_lineRight.m_p1.m_fY << ")  " << "P2: (" <<
        data.m_roadSide.m_lineRight.m_p2.m_fX << ", " <<
        data.m_roadSide.m_lineRight.m_p2.m_fY << ")" << std::endl << std::endl;

  ss << "lidar, 原始(x,y,w,l,a), 融合(x,y,w,l,a)" << std::endl;
  int count = std::min<int>(data.m_lidarOriginal.size(), data.m_lidar.size());
  for (int i = 0; i < count; ++i) {
    ss << "   " << i << "   (" << data.m_lidarOriginal[i].m_fCenterX << ",  " <<
          data.m_lidarOriginal[i].m_fCenterY << ",  " <<
          data.m_lidarOriginal[i].m_fWidth << ",  " <<
          data.m_lidarOriginal[i].m_fLength << ",  " <<
          data.m_lidarOriginal[i].m_fAngle << ")    (" <<
          data.m_lidar[i].m_fCenterX << ",  " <<
          data.m_lidar[i].m_fCenterY << ",  " <<
          data.m_lidar[i].m_fWidth << ",  " <<
          data.m_lidar[i].m_fLength << ",  " <<
          data.m_lidar[i].m_fAngle << ")" << std::endl;
  }
  ss << std::endl;

  ss << "radar, 原始(x,y,w,l,a), 融合(x,y,w,l,a)" << std::endl;
  count = std::min<int>(data.m_radarOriginal.size(), data.m_radar.size());
  for (int i = 0; i < count; ++i) {
    ss << "   " << i << "   (" << data.m_radarOriginal[i].m_fCenterX << ",  " <<
          data.m_radarOriginal[i].m_fCenterY << ",  " <<
          data.m_radarOriginal[i].m_fWidth << ",  " <<
          data.m_radarOriginal[i].m_fLength << ",  " <<
          data.m_radarOriginal[i].m_fAngle << ")    (" <<
          data.m_radar[i].m_fCenterX << ",  " <<
          data.m_radar[i].m_fCenterY << ",  " <<
          data.m_radar[i].m_fWidth << ",  " <<
          data.m_radar[i].m_fLength << ",  " <<
          data.m_radar[i].m_fAngle << ")" << std::endl;
  }
  ss << std::endl;

  // ultrasonic
  ss << "ultrasonic, id, triggered, distance" << std::endl;
  for (int i = 0; i < ULTRA_NUM; ++i) {
    ss << "   " << data.m_ultrasonic[i].m_nId << "   " <<
          (data.m_ultrasonic[i].m_bTriggered ? "True" : "False") << "  " <<
          data.m_ultrasonic[i].m_fDistance << std::endl;
  }

  return ss.str();
}

/*******************************************************
 * @brief 从共享内存中读取点云数据
 * @param

 * @return
********************************************************/
void QPerceptionWidget::readCloundPoints()
{
  /*static dbAds::CLidarSharedData shared_data;

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzPtr;
  PointTLIPtr tliPtr;
  m_unPointCount = shared_data.read(xyzPtr, tliPtr);
  for (uint32_t i = 0; i < m_unPointCount; ++i) {
    m_cloundPoints[i].m_fX = xyzPtr->points[i].x;
    m_cloundPoints[i].m_fY = xyzPtr->points[i].y;
    m_cloundPoints[i].m_fZ = xyzPtr->points[i].z;
  }*/
}
