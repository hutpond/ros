/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionWidget.h
 * Author: liuzheng
 * Date: 2019/6/25
 * Description: Perception模块显示画面
********************************************************/
#ifndef Q_PERCEPTION_WIDGET_H
#define Q_PERCEPTION_WIDGET_H

#include <vector>
#include <boost/filesystem.hpp>
#include "jsoncpp/json/json.h"
#include "QBaseWidget.h"

class QPerceptionShowWidget;
class QPerceptionShow3DWidget;
class QPerceptionParamWidget;

struct Point
{
  float m_fX;
  float m_fY;
};

struct Point3D
{
  float m_fX;
  float m_fY;
  float m_fZ;
};

struct Line
{
  Point m_p1;
  Point m_p2;
};

struct Rect
{
  float m_fCenterX;
  float m_fCenterY;
  float m_fWidth;
  float m_fLength;
  float m_fAngle;
};

struct RoadSide
{
  Line m_lineLeft;
  Line m_lineRight;
};

struct Ultrasonic
{
  int m_nId;
  bool m_bTriggered;
  float m_fDistance;
};

static const int ULTRA_NUM = 8;
struct PerceptionData
{
  Rect m_rectSweeper;
  RoadSide m_roadSide;
  std::vector<Rect> m_lidarOriginal;
  std::vector<Rect> m_radarOriginal;
  Ultrasonic m_ultrasonic[ULTRA_NUM];
  std::vector<Rect> m_lidar;
  std::vector<Rect> m_radar;
};

class QPerceptionWidget : public QBaseWidget
{
  Q_OBJECT

public:
  enum
  {
    PerceptionNone = -1,
    Perception2D,
    Perception3D
  };

public:
  QPerceptionWidget(QWidget *parent);
  ~QPerceptionWidget();
  void showPerception(int);
  void stopDisplay();

  void showType(int);
  void setViewResolution(int);

protected:
  virtual void resizeEvent(QResizeEvent *);
  virtual void timerEvent(QTimerEvent *);

protected:
  void writeToFile(const PerceptionData *);
  void readFromFile(const std::string &, PerceptionData *);
  std::string createDispayText(const std::string &);

  void readCloundPoints();

private slots:
  void onParseJsonData(const Json::Value &);
  void onSetFrameIndexReplay(int);
  void onDisplayData();

private:
  QPerceptionShowWidget *m_pWdgShow;
  QPerceptionShow3DWidget *m_pWdgShow3d;
  QPerceptionParamWidget *m_pWdgParam;
  int m_nSubShowIndex;    // 子画面index, PerceptionNone, Perception2D, Perception3D

  uint32_t m_unPointCount;
  Point3D *m_cloundPoints;
};

#endif // Q_PERCEPTION_WIDGET_H
