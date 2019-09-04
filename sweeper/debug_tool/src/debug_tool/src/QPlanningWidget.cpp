/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划主界面，可以实时显示和回放
********************************************************/
#include <time.h>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <QFile>
#include <QTimerEvent>
#include <QTextBrowser>
#include <QStatusBar>
#include "boost/filesystem.hpp"
#include "QPlanningWidget.h"
#include "QPlanningShowWidget.h"
#include "QPlanningParamWidget.h"
#include "GlobalDefine.h"
#include "QDataDisplayDialog.h"
#include "QDebugToolMainWnd.h"

static constexpr int REPLAY_MSEC[3] = {50, 200, 400};

QPlanningWidget::QPlanningWidget(QWidget *parent)
  : QBaseWidget(parent)
{
  this->setFont(G_TEXT_FONT);
  m_pWdgShow = new QPlanningShowWidget(this);
  m_pWdgParam = new QPlanningParamWidget(this);
  connect(m_pWdgParam, &QPlanningParamWidget::replayState,
          this, &QPlanningWidget::onReplayState);
  connect(m_pWdgParam, &QPlanningParamWidget::replayFrameOffset,
          this, &QPlanningWidget::onSetFrameIndexReplay);
  connect(m_pWdgParam, &QPlanningParamWidget::replayInterIndex,
          this, &QPlanningWidget::onReplayInterIndex);

  boost::function<void(float, float, float, float)> fun = boost::bind(&QPlanningParamWidget::showMousePosition, m_pWdgParam,
                       _1, _2, _3, _4);
  m_pWdgShow->setFunPosition(fun);

  namespace fs = boost::filesystem;
  //m_fsPath = fs::current_path();
  m_fsPath = getenv("HOME");
  m_fsPath /= "PlanningData";

  char time_str[64] = {0};
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);

  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d",
          utcTime->tm_year + 1900,
          utcTime->tm_mon + 1,
          utcTime->tm_mday,
          utcTime->tm_hour,
          utcTime->tm_min,
          utcTime->tm_sec
          );

  m_fsPath /= time_str;
  if (!fs::exists(m_fsPath)) {
    fs::create_directories(m_fsPath);
  }

  connect(QReadDataManagerRos::instance(), SIGNAL(planningData(const debug_tool::ads_PlanningData4Debug &)),
          this, SLOT(onParsePlanningData(const debug_tool::ads_PlanningData4Debug &)));

  QReadDataManagerRos::instance()->start_subscribe();

  m_nIntervalMillSecs = REPLAY_MSEC[0];
}

QPlanningWidget::~QPlanningWidget()
{
}

void QPlanningWidget::resizeEvent(QResizeEvent *)
{
  const int W_PERCENT = 70;
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  m_pWdgShow->setGeometry(
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

void QPlanningWidget::timerEvent(QTimerEvent *e)
{
  int id = e->timerId();
  if (id == m_nTimerId) {
    if (!this->isVisible() || m_bFlagPauseReplay || m_nShowType != Replay ||
        m_itFile == m_listPlanningFiles.end()) {
      return;
    }
    debug_tool::ads_PlanningData4Debug data;
    //memset(&data, 0, sizeof(data));
    std::string name = *m_itFile;
    std::size_t index = name.find_last_of('/');
    name = name.substr(index + 1, name.length() - (index + 1) - 1);
    if (this->readFromJsonFile(*m_itFile, data)) {
      m_pWdgShow->setPlanningData(data);
      m_pWdgParam->setPlanningData(data);
      QDebugToolMainWnd::s_pTextBrowser->setPlainText(QString::fromStdString(data.debug_info));
      QDebugToolMainWnd::s_pDataDisplay->setPlanningData(data);
    }
    else {
      name += "-------- FAILED !!!!!!!!!!!!!!!!1";
    }
    QDebugToolMainWnd::s_pStatusBar->showMessage(QString::fromStdString(name));
    m_pWdgParam->setFrameOffset(1);
    ++m_itFile;
  }
}

/*******************************************************
 * @brief 停止显示线程
 * @param

 * @return
********************************************************/
void QPlanningWidget::stopDisplay()
{
#ifdef WIN64
    ReadDataManager::instance()->stopRead();
#else
  QReadDataManagerRos::instance()->stop_subscirbe();
#endif
  m_nShowType = TypeNone;
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
}

/*******************************************************
 * @brief 设置显示画面
 * @param type: LiveDisplay，实时显示; Replay, 回放

 * @return
********************************************************/
void QPlanningWidget::showType(int type, const QString &path)
{
  if (!this->isVisible()) {
    m_nShowType = TypeNone;
  }
  this->show();
  if (type == LiveDisplay) {
    if (m_nShowType != type) {
      m_nShowType = type;
      if (m_nTimerId != 0) {
        killTimer(m_nTimerId);
        m_nTimerId = 0;
      }
      m_pWdgParam->setShowType(type);
    }
  }
  else {
    m_nShowType = type;
    this->replayJson(path);
    m_pWdgParam->setFrameCount(m_listPlanningFiles.size());
    m_pWdgParam->setShowType(type);
  }
}

/*******************************************************
 * @brief 缩放图像显示
 * @param index: -1， 缩小, 0, 复原, 1, 放大

 * @return
********************************************************/
void QPlanningWidget::setViewResolution(int index)
{
  m_pWdgShow->setViewResolution(index);
}

/*******************************************************
 * @brief 暂停replay时，定位到帧
 * @param index: 距当前帧的帧数，负数向前，正数向后

 * @return
********************************************************/
void QPlanningWidget::onSetFrameIndexReplay(int index)
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
  if (m_itFile != m_listPlanningFiles.end()) {
    debug_tool::ads_PlanningData4Debug data;
    //memset(&data, 0, sizeof(data));
    std::string name = *m_itFile;
    std::size_t index = name.find_last_of('/');
    name = name.substr(index + 1, name.length() - (index + 1) - 1);
    if (this->readFromJsonFile(*m_itFile, data)) {
      m_pWdgShow->setPlanningData(data);
      m_pWdgParam->setPlanningData(data);
      QDebugToolMainWnd::s_pTextBrowser->setPlainText(QString::fromStdString(data.debug_info));
      QDebugToolMainWnd::s_pDataDisplay->setPlanningData(data);
    }
    else {
      name += "-------- FAILED !!!!!!!!!!!!!!!!1";
    }
    QDebugToolMainWnd::s_pStatusBar->showMessage(QString::fromStdString(name));
  }
}

void QPlanningWidget::replayJson(const QString &path)
{
  m_bFlagPauseReplay = false;
  m_listPlanningFiles.clear();
  this->fileList(path.toStdString(), m_listPlanningFiles);

  m_itFile = m_listPlanningFiles.begin();
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
  m_bFlagPauseReplay = false;
  m_nTimerId = startTimer(m_nIntervalMillSecs);
}

void QPlanningWidget::onReplayInterIndex(int index)
{
  m_nIntervalMillSecs = REPLAY_MSEC[index];
  if (m_nTimerId == 0) {
    return;
  }
  killTimer(m_nTimerId);
  m_nTimerId = startTimer(m_nIntervalMillSecs);
}

bool QPlanningWidget::readFromJsonFile(const std::string &name, debug_tool::ads_PlanningData4Debug &planningData)
{
  const std::string fileName = name.substr(1, name.length() - 2);

  FILE *pf = fopen(fileName.c_str(), "r");
  if (pf == NULL) {
    return false;
  }
  fseek(pf , 0 , SEEK_END);
  long size = ftell(pf);
  rewind(pf);
  char *buffer = (char*)malloc(size + 1);
  memset(buffer, 0, size + 1);
  if (buffer == NULL) {
    return false;
  }
  fread(buffer,1, size, pf);
  fclose(pf);

  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(buffer, root)) {
    delete buffer;
    return false;
  }
  delete buffer;
  this->parseDataFromJson(root, planningData);
  this->sortTrackTargets(planningData);

  return true;
}

void QPlanningWidget::onParsePlanningData(const debug_tool::ads_PlanningData4Debug &planningData)
{
  debug_tool::ads_PlanningData4Debug &data = const_cast<debug_tool::ads_PlanningData4Debug &>(planningData);
  this->sortTrackTargets(data);
  this->saveDataToJsonFile(data);
  if (this->isVisible() && m_nShowType == LiveDisplay) {
    m_pWdgShow->setPlanningData(data);
    m_pWdgParam->setPlanningData(data);
    QDebugToolMainWnd::s_pTextBrowser->setPlainText(QString::fromStdString(data.debug_info));
    QDebugToolMainWnd::s_pDataDisplay->setPlanningData(data);
  }
}

void QPlanningWidget::saveDataToJsonFile(const debug_tool::ads_PlanningData4Debug &planningData)
{
  // car
  Json::Value carStatus;
  carStatus["LATITUDE"] = planningData.vehicle_latitude;
  carStatus["LONGITUDE"] = planningData.vehicle_longitude;
  carStatus["ALTITUDE"] = planningData.vehicle_altitude;
  carStatus["ENU_X"] = planningData.vehicle_x;
  carStatus["ENU_Y"] = planningData.vehicle_y;
  carStatus["ENU_Z"] = planningData.vehicle_z;
  carStatus["S"] = planningData.vehicle_s;
  carStatus["L"] = planningData.vehicle_l;
  carStatus["VEHICLE_WIDTH"] = planningData.vehicle_width;
  carStatus["VEHICLE_LENGTH"] = planningData.vehicle_length;
  carStatus["HEAD_DISTANCE"] = planningData.head_distance;

  // radar 28 target
  const debug_tool::ads_Radar28fTargetColl &radar28Result = planningData.radar28f_results;
  Json::Value radar28Trargets, radar28Trarget;
  int sizeTarget28 = qMin<int>(100, radar28Result.object_count);
  radar28Trargets["OBJECT_COUNT"] = radar28Result.object_count;
  for (int i = 0; i < sizeTarget28; ++i) {
    Json::Value item;
    const debug_tool::ads_RadarTarget &target = radar28Result.radar_objects[i];
    item["ID"] = target.id;
    item["RANGE"] = target.range;
    item["RANGE_LAT"] = target.range_lat;
    item["RANGE_LON"] = target.range_lon;
    item["ANGLE"] = target.angle;
    item["VEL"] = target.vel;
    item["V_LAT"] = target.v_lat;
    item["V_LON"] = target.v_lon;
    item["STATUS"] = target.status;
    item["W"] = target.w;
    item["L"] = target.l;
    item["DEVID"] = target.devid;

    radar28Trarget.append(item);
  }
  radar28Trargets["RADAR28_OBJECTS"] = radar28Trarget;

  // radar 73 target
  const debug_tool::ads_Radar73fTargetColl &radar73Result = planningData.radar73f_results;
  Json::Value radar73Trargets, radar73Trarget;
  int sizeTarget73 = qMin<int>(100, radar73Result.object_count);
  radar73Trargets["OBJECT_COUNT"] = radar73Result.object_count;
  for (int i = 0; i < sizeTarget73; ++i) {
    Json::Value item;
    const debug_tool::ads_RadarTarget &target = radar73Result.radar_objects[i];
    item["ID"] = target.id;
    item["RANGE"] = target.range;
    item["RANGE_LAT"] = target.range_lat;
    item["RANGE_LON"] = target.range_lon;
    item["ANGLE"] = target.angle;
    item["VEL"] = target.vel;
    item["V_LAT"] = target.v_lat;
    item["V_LON"] = target.v_lon;
    item["STATUS"] = target.status;
    item["W"] = target.w;
    item["L"] = target.l;
    item["DEVID"] = target.devid;

    radar73Trarget.append(item);
  }
  radar73Trargets["RADAR73_OBJECTS"] = radar73Trarget;

  // UltraSonic
  const debug_tool::ads_UltraSonicTargetColl &ultrasonicResult = planningData.ultrasonic_results;
  Json::Value ultrasonicTrargets, ultrasonicTrarget;
  int sizeTarget = qMin<int>(8, ultrasonicResult.object_count);
  ultrasonicTrargets["OBJECT_COUNT"] = static_cast<int>(ultrasonicResult.object_count);
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item;
    const debug_tool::ads_UltraSonicTarget &target = ultrasonicResult.us_objects[i];

    item["POS_ID"] = static_cast<int>(target.radar_pos_id);
    item["DISTANCE"] = target.distance;
    ultrasonicTrarget.append(item);
  }
  ultrasonicTrargets["US_OBJECTS"] = ultrasonicTrarget;

  // Track Target
  const debug_tool::ads_TrackTargetColl &trackResult = planningData.fusion_results;
  Json::Value trackTrargets, trackTrarget;
  sizeTarget = qMin<int>(250, trackResult.object_count);
  trackTrargets["OBJECT_COUNT"] = trackResult.object_count;
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item;
    const debug_tool::ads_TrackTarget &target = trackResult.track_objects[i];

    item["TRACK_ID"] = static_cast<int32_t>(target.TRACK_ID);
    item["X"] = target.X;
    item["Y"] = target.Y;
    item["SX"] = target.SX;
    item["ANGLE"] = target.ANGLE;
    item["W"] = target.W;
    item["L"] = target.L;
    item["H"] = target.H;
    item["P1_X"] = target.P1_X;
    item["P1_Y"] = target.P1_Y;
    item["P2_X"] = target.P2_X;
    item["P2_Y"] = target.P2_Y;
    item["P3_X"] = target.P3_X;
    item["P3_Y"] = target.P3_Y;
    item["P4_X"] = target.P4_X;
    item["P4_Y"] = target.P4_Y;
    item["STATUS"] = target.STATUS;

    trackTrarget.append(item);
  }
  trackTrargets["TRACK_OBJECTS"] = trackTrarget;

  // garbage
  Json::Value garbage;
  const auto &garbage_result = planningData.garbage_detection_results.result;
  auto garbage_size = garbage_result.size();
  using size_g = decltype (garbage_size);
  for (size_g i = 0; i < garbage_size; ++i) {
    Json::Value item;
    item["ID"] = garbage_result[i].id;
    item["SIZE"] = garbage_result[i].size;
    item["ANGLE"] = garbage_result[i].angle;
    item["DISTANCE"] = garbage_result[i].distance;
    garbage.append(item);
  }

  // decision
  Json::Value decisionState;
  decisionState["DECISION"] = static_cast<int>(planningData.decision);
  decisionState["ULTRASONIC_DECISION"] = static_cast<int>(planningData.ultrasonic_decision);
  decisionState["RADAR28_DECISION"] = static_cast<int>(planningData.radar28f_decision);
  decisionState["RADAR73_DECISION"] = static_cast<int>(planningData.radar73f_decision);
  decisionState["TRACK_TARGET_DECISION"] = static_cast<int>(planningData.track_target_decision);

  Json::Value decision_targets;
  for (int i = 0; i < 4; ++i) {
    Json::Value item;
    item["sensor_type"] = static_cast<int>(planningData.decision_targets[i].sensor_type);
    item["x"] = planningData.decision_targets[i].x;
    item["y"] = planningData.decision_targets[i].y;
    item["angle"] = planningData.decision_targets[i].angle;
    item["width"] = planningData.decision_targets[i].width;
    item["length"] = planningData.decision_targets[i].length;
    item["s"] = planningData.decision_targets[i].s;
    item["l"] = planningData.decision_targets[i].l;
    item["sl_width"] = planningData.decision_targets[i].sl_width;
    item["sl_length"] = planningData.decision_targets[i].sl_length;
    item["p1_x"] = planningData.decision_targets[i].p1_x;
    item["p1_y"] = planningData.decision_targets[i].p1_y;
    item["p2_x"] = planningData.decision_targets[i].p2_x;
    item["p2_y"] = planningData.decision_targets[i].p2_y;
    item["p3_x"] = planningData.decision_targets[i].p3_x;
    item["p3_y"] = planningData.decision_targets[i].p3_y;
    item["p4_x"] = planningData.decision_targets[i].p4_x;
    item["p4_y"] = planningData.decision_targets[i].p4_y;
    decision_targets.append(item);
  }
  decisionState["DECISION_TARGETS"] = decision_targets;

  // planning
  Json::Value trajectory;
  Json::Value planning_output;
  //planning_output["HEADER"] = planningData.planning_output.header
  planning_output["DECISION"] = planningData.planning_output.decision;
  planning_output["VELOCITY"] = planningData.planning_output.velocity;
  planning_output["POSE_POSITION_X"] = planningData.planning_output.pose.position.x;
  planning_output["POSE_POSITION_Y"] = planningData.planning_output.pose.position.y;
  trajectory["PLANNING_OUTPUT"] = planning_output;

  Json::Value json_candidates;
  const auto &val_candidates = planningData.planning_trajectory_candidates;
  for (int i = 0; i < 10; ++i) {
    Json::Value item, splines;
    item["ID"] = static_cast<int>(val_candidates[i].id);
    item["COST"] = val_candidates[i].cost;
    item["SAFETY_COST"] = val_candidates[i].safety_cost;
    item["LATERAL_COST"] = val_candidates[i].lateral_cost;
    item["SMOOTHNESS_COST"] = val_candidates[i].smoothness_cost;
    item["CONSISTENCY_COST"] = val_candidates[i].consistency_cost;
    item["GARBAGE_COST"] = val_candidates[i].garbage_cost;

    int size_candidates_splines = static_cast<int>(val_candidates[i].splines.size());
    for (int j = 0; j < size_candidates_splines; ++ j) {
      Json::Value spline;

      const auto &val_candidates_spine = val_candidates[i].splines[j];
      spline["XB_X"] = val_candidates_spine.xb.x;
      spline["XB_Y"] = val_candidates_spine.xb.y;
      spline["XB_Z"] = val_candidates_spine.xb.z;
      spline["XB_W"] = val_candidates_spine.xb.w;
      spline["YB_X"] = val_candidates_spine.yb.x;
      spline["YB_Y"] = val_candidates_spine.yb.y;
      spline["YB_Z"] = val_candidates_spine.yb.z;
      spline["YB_W"] = val_candidates_spine.yb.w;

      splines.append(spline);
    }
    item["SPLINES"] = splines;
    json_candidates.append(item);
  }
  trajectory["TRAJECTORY_CANDIDATES"] = json_candidates;

  Json::Value json_trajectory, json_trajectory_splines;
  const auto &val_trajectory = planningData.planning_trajectory;
  json_trajectory["ID"] = static_cast<int>(val_trajectory.id);
  json_trajectory["COST"] = val_trajectory.cost;
  json_trajectory["SAFETY_COST"] = val_trajectory.safety_cost;
  json_trajectory["LATERAL_COST"] = val_trajectory.lateral_cost;
  json_trajectory["SMOOTHNESS_COST"] = val_trajectory.smoothness_cost;
  json_trajectory["CONSISTENCY_COST"] = val_trajectory.consistency_cost;
  json_trajectory["GARBAGE_COST"] = val_trajectory.garbage_cost;
  int size_trajectory_splines = static_cast<int>(val_trajectory.splines.size());
  for (int i = 0; i < size_trajectory_splines; ++ i) {
    Json::Value spline;

    spline["XB_X"] = val_trajectory.splines[i].xb.x;
    spline["XB_Y"] = val_trajectory.splines[i].xb.y;
    spline["XB_Z"] = val_trajectory.splines[i].xb.z;
    spline["XB_W"] = val_trajectory.splines[i].xb.w;
    spline["YB_X"] = val_trajectory.splines[i].yb.x;
    spline["YB_Y"] = val_trajectory.splines[i].yb.y;
    spline["YB_Z"] = val_trajectory.splines[i].yb.z;
    spline["YB_W"] = val_trajectory.splines[i].yb.w;

    json_trajectory_splines.append(spline);
  }
  json_trajectory["SPLINES"] = json_trajectory_splines;
  trajectory["PLANNING_TRAJECTORY"] = json_trajectory;

  // road info
  Json::Value roadInfo;
  roadInfo["LEFT_HALF_ROAD_WIDTH"] = planningData.left_half_road_width;
  roadInfo["RIGHT_HALF_ROAD_WIDTH"] = planningData.right_half_road_width;
  roadInfo["MIN_SAFE_DISTANCE"] = planningData.safe_dis1;
  roadInfo["SAFE_DISTANCE"] = planningData.safe_dis2;
  roadInfo["MAX_PLANNING_DISTANCE"] = planningData.max_planning_distance;
  roadInfo["LEFT_ROAD_BOUNDARY_AVAILABLE"] = planningData.left_road_boundary_available;
  roadInfo["RIGHT_ROAD_BOUNDARY_AVAILABLE"] = planningData.right_road_boundary_available;
  Json::Value leftRoadBoundary, rightRoadBoundary;
  for (size_t i = 0; i < 5; ++i) {
    Json::Value itemLeft = planningData.left_road_boundary[i];
    leftRoadBoundary.append(itemLeft);
    Json::Value itemRight = planningData.right_road_boundary[i];
    rightRoadBoundary.append(itemRight);
  }
  roadInfo["LEFT_ROAD_BOUNDARY"] = leftRoadBoundary;
  roadInfo["RIGHT_ROAD_BOUNDARY"] = rightRoadBoundary;
  roadInfo["LEFT_ROAD_BOUNDARY_START_S"] = planningData.left_road_boundary_start_s;
  roadInfo["LEFT_ROAD_BOUNDARY_END_S"] = planningData.left_road_boundary_end_s;
  roadInfo["RIGHT_ROAD_BOUNDARY_START_S"] = planningData.right_road_boundary_start_s;
  roadInfo["RIGHT_ROAD_BOUNDARY_END_S"] = planningData.right_road_boundary_end_s;

  Json::Value ads_pubcurb;
  ads_pubcurb["CURB_L_FOUND"] = planningData.curb.curb_L_FOUND;
  ads_pubcurb["Point_L1_X"] = planningData.curb.Point_L1.x;
  ads_pubcurb["Point_L1_Y"] = planningData.curb.Point_L1.y;
  ads_pubcurb["Point_L2_X"] = planningData.curb.Point_L2.x;
  ads_pubcurb["Point_L2_Y"] = planningData.curb.Point_L2.y;
  ads_pubcurb["CURB_R_FOUND"] = planningData.curb.curb_R_FOUND;
  ads_pubcurb["Point_R1_X"] = planningData.curb.Point_R1.x;
  ads_pubcurb["Point_R1_Y"] = planningData.curb.Point_R1.y;
  ads_pubcurb["Point_R2_X"] = planningData.curb.Point_R2.x;
  ads_pubcurb["Point_R2_Y"] = planningData.curb.Point_R2.y;
  roadInfo["ADS_PUBCURB"] = ads_pubcurb;

  roadInfo["NUM_LEFT_ROAD_BOUNDARY_SPLINES"] = planningData.num_left_road_boundary_splines;
  const int SIZE_LEFT_ROAD_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_left_road_boundary_splines), 100);
  Json::Value left_road_splines;
  for (int i = 0; i < SIZE_LEFT_ROAD_SPLINES; ++ i) {
    Json::Value item;
    item["XB_X"] = planningData.left_road_boundary_splines[i].xb.x;
    item["XB_Y"] = planningData.left_road_boundary_splines[i].xb.y;
    item["XB_Z"] = planningData.left_road_boundary_splines[i].xb.z;
    item["XB_W"] = planningData.left_road_boundary_splines[i].xb.w;
    item["YB_X"] = planningData.left_road_boundary_splines[i].yb.x;
    item["YB_Y"] = planningData.left_road_boundary_splines[i].yb.y;
    item["YB_Z"] = planningData.left_road_boundary_splines[i].yb.z;
    item["YB_W"] = planningData.left_road_boundary_splines[i].yb.w;
    left_road_splines.append(item);
  }
  roadInfo["LEFT_ROAD_BOUNDARY_SPLINES"] = left_road_splines;
  roadInfo["NUM_RIGHT_ROAD_BOUNDARY_SPLINES"] = planningData.num_right_road_boundary_splines;
  const int SIZE_RIGHT_ROAD_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_right_road_boundary_splines), 100);
  Json::Value right_road_splines;
  for (int i = 0; i < SIZE_RIGHT_ROAD_SPLINES; ++ i) {
    Json::Value item;
    item["XB_X"] = planningData.right_road_boundary_splines[i].xb.x;
    item["XB_Y"] = planningData.right_road_boundary_splines[i].xb.y;
    item["XB_Z"] = planningData.right_road_boundary_splines[i].xb.z;
    item["XB_W"] = planningData.right_road_boundary_splines[i].xb.w;
    item["YB_X"] = planningData.right_road_boundary_splines[i].yb.x;
    item["YB_Y"] = planningData.right_road_boundary_splines[i].yb.y;
    item["YB_Z"] = planningData.right_road_boundary_splines[i].yb.z;
    item["YB_W"] = planningData.right_road_boundary_splines[i].yb.w;
    right_road_splines.append(item);
  }
  roadInfo["RIGHT_ROAD_BOUNDARY_SPLINES"] = right_road_splines;

  // referene
  Json::Value referenceLine;
  Json::Value referencePoints;
  sizeTarget = qMin<int>(100, planningData.num_reference_points);
  referenceLine["NUM_REFERENCE_POINTS"] = static_cast<int>(planningData.num_reference_points);
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item;
    const debug_tool::ads_ReferencePoint &reference = planningData.reference_points[i];
    item["ID"] = static_cast<int>(reference.id);
    item["L"] = reference.l;
    item["S"] = reference.s;
    item["X"] = reference.x;
    item["Y"] = reference.y;
    referencePoints.append(item);
  }
  referenceLine["REFERENCE_POINTS"] = referencePoints;

  referenceLine["NUM_REFERENCE_SPLINES"] = static_cast<int>(planningData.num_reference_splines);
  const int SIZE_REFERENCE_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_reference_splines), 100);
  Json::Value reference_splines;
  for (int i = 0; i < SIZE_REFERENCE_SPLINES; ++ i) {
    Json::Value item;
    item["XB_X"] = planningData.reference_splines[i].xb.x;
    item["XB_Y"] = planningData.reference_splines[i].xb.y;
    item["XB_Z"] = planningData.reference_splines[i].xb.z;
    item["XB_W"] = planningData.reference_splines[i].xb.w;
    item["YB_X"] = planningData.reference_splines[i].yb.x;
    item["YB_Y"] = planningData.reference_splines[i].yb.y;
    item["YB_Z"] = planningData.reference_splines[i].yb.z;
    item["YB_W"] = planningData.reference_splines[i].yb.w;
    reference_splines.append(item);
  }
  referenceLine["REFERENCE_SPLINES"] = reference_splines;

  Json::Value data;
  data["CAR_STATUS"] = carStatus;
  data["RADAR28_TRARGETS"] = radar28Trargets;
  data["RADAR73_TRARGETS"] = radar73Trargets;
  data["ULTRASONIC_TRARGETS"] = ultrasonicTrargets;
  data["TRACK_TRARGETS"] = trackTrargets;
  data["LITTERSENSOR_DATA"] = garbage;
  data["DECISION_STATE"] = decisionState;
  data["TRAJECTORY"] = trajectory;
  data["ROAD_INFO"] = roadInfo;
  data["REFERENCE_LINE"] = referenceLine;
  data["DEBUG_INFO"] = planningData.debug_info;

  char time_str[64] = {0};
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d_%03d",
          utcTime->tm_year + 1900,
          utcTime->tm_mon + 1,
          utcTime->tm_mday,
          utcTime->tm_hour,
          utcTime->tm_min,
          utcTime->tm_sec,
          static_cast<int>((tv.tv_usec / 1000) % 1000)
          );

  fs::path path = m_fsPath;
  std::string strFileName = time_str;
  strFileName += ".txt";
  path /= strFileName;
  strFileName = path.string();

  std::ofstream out(strFileName.c_str());
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(data, &out);
  out.close();
}

void QPlanningWidget::parseDataFromJson(
    const Json::Value &data,
    debug_tool::ads_PlanningData4Debug &planningData)
{
  //memset(&planningData, 0, sizeof(planningData));
  Json::Value carStatus = data["CAR_STATUS"];
  Json::Value radar28Trargets = data["RADAR28_TRARGETS"];
  Json::Value radar73Trargets = data["RADAR73_TRARGETS"];
  Json::Value ultrasonicTrargets = data["ULTRASONIC_TRARGETS"];
  Json::Value trackTrargets = data["TRACK_TRARGETS"];
  Json::Value garbage = data["LITTERSENSOR_DATA"];
  Json::Value decisionState = data["DECISION_STATE"];
  Json::Value trajectory = data["TRAJECTORY"];
  Json::Value roadInfo = data["ROAD_INFO"];
  Json::Value referenceLine = data["REFERENCE_LINE"];

  // car
  planningData.vehicle_latitude = carStatus["LATITUDE"].asDouble();
  planningData.vehicle_longitude = carStatus["LONGITUDE"].asDouble();
  planningData.vehicle_altitude = carStatus["ALTITUDE"].asDouble();
  planningData.vehicle_x = carStatus["ENU_X"].asDouble();
  planningData.vehicle_y = carStatus["ENU_Y"].asDouble();
  planningData.vehicle_z = carStatus["ENU_Z"].asDouble();
  planningData.vehicle_s = carStatus["S"].asDouble();
  planningData.vehicle_l = carStatus["L"].asDouble();
  planningData.vehicle_width = carStatus["VEHICLE_WIDTH"].asDouble();
  planningData.vehicle_length = carStatus["VEHICLE_LENGTH"].asDouble();
  planningData.head_distance = carStatus["HEAD_DISTANCE"].asDouble();

  // radar 28 target
  debug_tool::ads_Radar28fTargetColl &radar28Result = planningData.radar28f_results;
  radar28Result.object_count = radar28Trargets["OBJECT_COUNT"].asInt();
  Json::Value radar28Trarget = radar28Trargets["RADAR28_OBJECTS"];
  int sizeTarget28 = qMin<int>(100, radar28Result.object_count);
  for (int i = 0; i < sizeTarget28; ++i) {
    Json::Value item = radar28Trarget[i];
    debug_tool::ads_RadarTarget &target = radar28Result.radar_objects[i];
    target.id = item["ID"].asInt();
    target.range = item["RANGE"].asDouble();
    target.range_lat = item["RANGE_LAT"].asDouble();
    target.range_lon = item["RANGE_LON"].asDouble();
    target.angle = item["ANGLE"].asDouble();
    target.vel = item["VEL"].asDouble();
    target.v_lat = item["V_LAT"].asDouble();
    target.v_lon = item["V_LON"].asDouble();
    target.status = item["STATUS"].asInt();
    target.w = item["W"].asDouble();
    target.l = item["L"].asDouble();
    target.devid = item["DEVID"].asInt();
  }

  // radar 73 target
  debug_tool::ads_Radar73fTargetColl &radar73Result = planningData.radar73f_results;
  radar73Result.object_count = radar73Trargets["OBJECT_COUNT"].asInt();
  Json::Value radar73Trarget = radar73Trargets["RADAR73_OBJECTS"];
  int sizeTarget73 = qMin<int>(100, radar73Result.object_count);
  for (int i = 0; i < sizeTarget73; ++i) {
    Json::Value item = radar73Trarget[i];
    debug_tool::ads_RadarTarget &target = radar73Result.radar_objects[i];
    target.id = item["ID"].asInt();
    target.range = item["RANGE"].asDouble();
    target.range_lat = item["RANGE_LAT"].asDouble();
    target.range_lon = item["RANGE_LON"].asDouble();
    target.angle = item["ANGLE"].asDouble();
    target.vel = item["VEL"].asDouble();
    target.v_lat = item["V_LAT"].asDouble();
    target.v_lon = item["V_LON"].asDouble();
    target.status = item["STATUS"].asInt();
    target.w = item["W"].asDouble();
    target.l = item["L"].asDouble();
    target.devid = item["DEVID"].asInt();
  }

  // UltraSonic
  debug_tool::ads_UltraSonicTargetColl &ultrasonicResult = planningData.ultrasonic_results;
  ultrasonicResult.object_count = static_cast<int8_t>(
        ultrasonicTrargets["OBJECT_COUNT"].asInt());
  Json::Value ultrasonicTrarget = ultrasonicTrargets["US_OBJECTS"];
  int sizeTarget = qMin<int>(8, ultrasonicResult.object_count);
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item = ultrasonicTrarget[i];
    debug_tool::ads_UltraSonicTarget &target = ultrasonicResult.us_objects[i];

    target.radar_pos_id = static_cast<int8_t>(item["POS_ID"].asInt());
    target.distance = item["DISTANCE"].asFloat();
  }

  // Track Target
  debug_tool::ads_TrackTargetColl &trackResult = planningData.fusion_results;
  trackResult.object_count = trackTrargets["OBJECT_COUNT"].asInt();
  Json::Value trackTrarget = trackTrargets["TRACK_OBJECTS"];
  sizeTarget = qMin<int>(250, trackResult.object_count);
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item = trackTrarget[i];
    debug_tool::ads_TrackTarget &target = trackResult.track_objects[i];

    target.TRACK_ID = static_cast<int64_t>(item["TRACK_ID"].asInt());
    target.X = item["X"].asDouble();
    target.Y = item["Y"].asDouble();
    target.SX = item["SX"].asFloat();
    target.ANGLE = item["ANGLE"].asFloat();
    target.W = item["W"].asFloat();
    target.L = item["L"].asFloat();
    target.H = item["H"].asFloat();
    target.P1_X = item["P1_X"].asFloat();
    target.P1_Y = item["P1_Y"].asFloat();
    target.P2_X = item["P2_X"].asFloat();
    target.P2_Y = item["P2_Y"].asFloat();
    target.P3_X = item["P3_X"].asFloat();
    target.P3_Y = item["P3_Y"].asFloat();
    target.P4_X = item["P4_X"].asFloat();
    target.P4_Y = item["P4_Y"].asFloat();
    target.STATUS = item["STATUS"].asFloat();
  }

  // garbage
  auto &garbage_result = planningData.garbage_detection_results.result;
  garbage_result.resize(1);
  auto gargage_item = garbage_result[0];
  using garbage_type = decltype (gargage_item);
  const int garbage_size = static_cast<int>(garbage.size());
  garbage_result.clear();
  for (int i = 0; i < garbage_size; ++i) {
    Json::Value item = garbage[i];
    garbage_type result;
    result.id = static_cast<decltype(result.id)>(item["ID"].asInt());
    result.size = item["SIZE"].asFloat();
    result.angle = item["ANGLE"].asFloat();
    result.distance = item["DISTANCE"].asFloat();
    garbage_result.push_back(result);
  }

  // decision
  planningData.decision = static_cast<int8_t>(decisionState["DECISION"].asInt());
  planningData.ultrasonic_decision = static_cast<int8_t>(
        decisionState["ULTRASONIC_DECISION"].asInt());
  planningData.radar28f_decision = static_cast<int8_t>(decisionState["RADAR28_DECISION"].asInt());
  planningData.radar73f_decision = static_cast<int8_t>(decisionState["RADAR73_DECISION"].asInt());
  planningData.track_target_decision = static_cast<int8_t>(decisionState["TRACK_TARGET_DECISION"].asInt());
  const int SIZE_DECISION =
      qBound<int>(0, static_cast<int>(decisionState["DECISION_TARGETS"].size()), 4);
  for (int i = 0; i < SIZE_DECISION; ++i) {
    Json::Value item = decisionState["DECISION_TARGETS"][i];
    planningData.decision_targets[i].sensor_type = static_cast<uint8_t>(item["sensor_type"].asInt());
    planningData.decision_targets[i].x = item["x"].asDouble();
    planningData.decision_targets[i].y = item["y"].asDouble();
    planningData.decision_targets[i].angle = item["angle"].asDouble();
    planningData.decision_targets[i].width = item["width"].asDouble();
    planningData.decision_targets[i].length = item["length"].asDouble();
    planningData.decision_targets[i].s = item["s"].asDouble();
    planningData.decision_targets[i].l = item["l"].asDouble();
    planningData.decision_targets[i].sl_width = item["sl_width"].asDouble();
    planningData.decision_targets[i].sl_length = item["sl_length"].asDouble();
    planningData.decision_targets[i].p1_x = item["p1_x"].asDouble();
    planningData.decision_targets[i].p1_y = item["p1_y"].asDouble();
    planningData.decision_targets[i].p2_x = item["p2_x"].asDouble();
    planningData.decision_targets[i].p2_y = item["p2_y"].asDouble();
    planningData.decision_targets[i].p3_x = item["p3_x"].asDouble();
    planningData.decision_targets[i].p3_y = item["p3_y"].asDouble();
    planningData.decision_targets[i].p4_x = item["p4_x"].asDouble();
    planningData.decision_targets[i].p4_y = item["p4_y"].asDouble();
  }

  // planning
  planningData.planning_output.decision = static_cast<uint8_t>(trajectory["PLANNING_OUTPUT"]["DECISION"].asInt());
  planningData.planning_output.velocity = trajectory["PLANNING_OUTPUT"]["VELOCITY"].asFloat();
  planningData.planning_output.pose.position.x = trajectory["PLANNING_OUTPUT"]["POSE_POSITION_X"].asDouble();
  planningData.planning_output.pose.position.y = trajectory["PLANNING_OUTPUT"]["POSE_POSITION_Y"].asDouble();

  auto &val_candidates = planningData.planning_trajectory_candidates;
  for (int i = 0; i < 10; ++i) {
    Json::Value item = trajectory["TRAJECTORY_CANDIDATES"][i];
    val_candidates[i].id = static_cast<uint8_t>(item["ID"].asInt());
    val_candidates[i].cost = item["COST"].asDouble();
    val_candidates[i].safety_cost = item["SAFETY_COST"].asDouble();
    val_candidates[i].lateral_cost = item["LATERAL_COST"].asDouble();
    val_candidates[i].smoothness_cost = item["SMOOTHNESS_COST"].asDouble();
    val_candidates[i].consistency_cost = item["CONSISTENCY_COST"].asDouble();
    val_candidates[i].garbage_cost = item["GARBAGE_COST"].asDouble();

    auto &val_candidates_spines = val_candidates[i].splines;
    Json::Value json_splines = item["SPLINES"];
    val_candidates_spines.clear();
    int size_candidates_splines = static_cast<int>(json_splines.size());
    for (int j = 0; j < size_candidates_splines; ++ j) {
      debug_tool::ads_Spline_<std::allocator<void>> val_spine;
      val_spine.xb.x = json_splines[j]["XB_X"].asDouble();
      val_spine.xb.y = json_splines[j]["XB_Y"].asDouble();
      val_spine.xb.z = json_splines[j]["XB_Z"].asDouble();
      val_spine.xb.w = json_splines[j]["XB_W"].asDouble();
      val_spine.yb.x = json_splines[j]["YB_X"].asDouble();
      val_spine.yb.y = json_splines[j]["YB_Y"].asDouble();
      val_spine.yb.z = json_splines[j]["YB_Z"].asDouble();
      val_spine.yb.w = json_splines[j]["YB_W"].asDouble();

      val_candidates_spines.push_back(val_spine);
    }
  }

  auto &val_trajectory = planningData.planning_trajectory;
  Json::Value json_trajectory = trajectory["PLANNING_TRAJECTORY"];
  val_trajectory.id = static_cast<uint8_t>(json_trajectory["ID"].asInt());
  val_trajectory.cost = json_trajectory["COST"].asDouble();
  val_trajectory.safety_cost = json_trajectory["SAFETY_COST"].asDouble();
  val_trajectory.lateral_cost = json_trajectory["LATERAL_COST"].asDouble();
  val_trajectory.smoothness_cost = json_trajectory["SMOOTHNESS_COST"].asDouble();
  val_trajectory.consistency_cost = json_trajectory["CONSISTENCY_COST"].asDouble();
  val_trajectory.garbage_cost = json_trajectory["GARBAGE_COST"].asDouble();

  auto &val_trajectory_spines = val_trajectory.splines;
  Json::Value json_trajectory_spines = json_trajectory["SPLINES"];
  val_trajectory_spines.clear();
  int size_trajectory_splines = static_cast<int>(json_trajectory_spines.size());
  for (int j = 0; j < size_trajectory_splines; ++ j) {
    debug_tool::ads_Spline_<std::allocator<void>> val_spine;
    val_spine.xb.x = json_trajectory_spines[j]["XB_X"].asDouble();
    val_spine.xb.y = json_trajectory_spines[j]["XB_Y"].asDouble();
    val_spine.xb.z = json_trajectory_spines[j]["XB_Z"].asDouble();
    val_spine.xb.w = json_trajectory_spines[j]["XB_W"].asDouble();
    val_spine.yb.x = json_trajectory_spines[j]["YB_X"].asDouble();
    val_spine.yb.y = json_trajectory_spines[j]["YB_Y"].asDouble();
    val_spine.yb.z = json_trajectory_spines[j]["YB_Z"].asDouble();
    val_spine.yb.w = json_trajectory_spines[j]["YB_W"].asDouble();

    val_trajectory_spines.push_back(val_spine);
  }

  // road info
  planningData.left_half_road_width = roadInfo["LEFT_HALF_ROAD_WIDTH"].asDouble();
  planningData.right_half_road_width = roadInfo["RIGHT_HALF_ROAD_WIDTH"].asDouble();
  planningData.safe_dis1 = roadInfo["MIN_SAFE_DISTANCE"].asDouble();
  planningData.safe_dis2 = roadInfo["SAFE_DISTANCE"].asDouble();
  planningData.max_planning_distance = roadInfo["MAX_PLANNING_DISTANCE"].asDouble();
  planningData.left_road_boundary_available = roadInfo["LEFT_ROAD_BOUNDARY_AVAILABLE"].asBool();
  planningData.right_road_boundary_available = roadInfo["RIGHT_ROAD_BOUNDARY_AVAILABLE"].asBool();
  int SIZE_BOUND = qBound<int>(0, static_cast<int>(roadInfo["LEFT_ROAD_BOUNDARY"].size()), 5);
  for (int i = 0; i < SIZE_BOUND; ++i) {
    planningData.left_road_boundary[i] = roadInfo["LEFT_ROAD_BOUNDARY"][i].asDouble();
  }
  SIZE_BOUND = qBound<int>(0, static_cast<int>(roadInfo["RIGHT_ROAD_BOUNDARY"].size()), 5);
  for (int i = 0; i < SIZE_BOUND; ++i) {
    planningData.right_road_boundary[i] = roadInfo["RIGHT_ROAD_BOUNDARY"][i].asDouble();
  }
  planningData.left_road_boundary_start_s = roadInfo["LEFT_ROAD_BOUNDARY_START_S"].asDouble();
  planningData.left_road_boundary_end_s = roadInfo["LEFT_ROAD_BOUNDARY_END_S"].asDouble();
  planningData.right_road_boundary_start_s = roadInfo["RIGHT_ROAD_BOUNDARY_START_S"].asDouble();
  planningData.right_road_boundary_end_s = roadInfo["RIGHT_ROAD_BOUNDARY_END_S"].asDouble();

  Json::Value ads_pubcurb = roadInfo["ADS_PUBCURB"];
  planningData.curb.curb_L_FOUND = ads_pubcurb["CURB_L_FOUND"].asBool();
  planningData.curb.Point_L1.x = ads_pubcurb["Point_L1_X"].asDouble();
  planningData.curb.Point_L1.y = ads_pubcurb["Point_L1_Y"].asDouble();
  planningData.curb.Point_L2.x = ads_pubcurb["Point_L2_X"].asDouble();
  planningData.curb.Point_L2.y = ads_pubcurb["Point_L2_Y"].asDouble();
  planningData.curb.curb_R_FOUND = ads_pubcurb["CURB_R_FOUND"].asBool();
  planningData.curb.Point_R1.x = ads_pubcurb["Point_R1_X"].asDouble();
  planningData.curb.Point_R1.y = ads_pubcurb["Point_R1_Y"].asDouble();
  planningData.curb.Point_R2.x = ads_pubcurb["Point_R2_X"].asDouble();
  planningData.curb.Point_R2.y = ads_pubcurb["Point_R2_Y"].asDouble();

  planningData.num_left_road_boundary_splines = static_cast<int8_t>(
        roadInfo["NUM_LEFT_ROAD_BOUNDARY_SPLINES"].asInt());
  const int SIZE_LEFT_ROAD_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_left_road_boundary_splines), 100);
  for (int i = 0; i < SIZE_LEFT_ROAD_SPLINES; ++ i) {
    Json::Value item = roadInfo["LEFT_ROAD_BOUNDARY_SPLINES"][i];
    planningData.left_road_boundary_splines[i].xb.x = item["XB_X"].asDouble();
    planningData.left_road_boundary_splines[i].xb.y = item["XB_Y"].asDouble();
    planningData.left_road_boundary_splines[i].xb.z = item["XB_Z"].asDouble();
    planningData.left_road_boundary_splines[i].xb.w = item["XB_W"].asDouble();
    planningData.left_road_boundary_splines[i].yb.x = item["YB_X"].asDouble();
    planningData.left_road_boundary_splines[i].yb.y = item["YB_Y"].asDouble();
    planningData.left_road_boundary_splines[i].yb.z = item["YB_Z"].asDouble();
    planningData.left_road_boundary_splines[i].yb.w = item["YB_W"].asDouble();
  }
  planningData.num_right_road_boundary_splines = static_cast<int8_t>(
        roadInfo["NUM_RIGHT_ROAD_BOUNDARY_SPLINES"].asInt());
  const int SIZE_RIGHT_ROAD_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_right_road_boundary_splines), 100);
  for (int i = 0; i < SIZE_RIGHT_ROAD_SPLINES; ++ i) {
    Json::Value item = roadInfo["RIGHT_ROAD_BOUNDARY_SPLINES"][i];
    planningData.right_road_boundary_splines[i].xb.x = item["XB_X"].asDouble();
    planningData.right_road_boundary_splines[i].xb.y = item["XB_Y"].asDouble();
    planningData.right_road_boundary_splines[i].xb.z = item["XB_Z"].asDouble();
    planningData.right_road_boundary_splines[i].xb.w = item["XB_W"].asDouble();
    planningData.right_road_boundary_splines[i].yb.x = item["YB_X"].asDouble();
    planningData.right_road_boundary_splines[i].yb.y = item["YB_Y"].asDouble();
    planningData.right_road_boundary_splines[i].yb.z = item["YB_Z"].asDouble();
    planningData.right_road_boundary_splines[i].yb.w = item["YB_W"].asDouble();
  }

  // referene
  Json::Value referencePoints = referenceLine["REFERENCE_POINTS"];
  planningData.num_reference_points = static_cast<int8_t>(
        referenceLine["NUM_REFERENCE_POINTS"].asInt());
  sizeTarget = qMin<int>(100, planningData.num_reference_points);
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item = referencePoints[i];
    debug_tool::ads_ReferencePoint &reference = planningData.reference_points[i];
    reference.id = static_cast<int8_t>(item["ID"].asInt());
    reference.l = item["L"].asDouble();
    reference.s = item["S"].asDouble();
    reference.x = item["X"].asDouble();
    reference.y = item["Y"].asDouble();
  }

  planningData.num_reference_splines = static_cast<int8_t>(
        referenceLine["NUM_REFERENCE_SPLINES"].asInt());
  const int SIZE_REFERENCE_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_reference_splines), 100);
  Json::Value reference_splines = referenceLine["REFERENCE_SPLINES"];
  for (int i = 0; i < SIZE_REFERENCE_SPLINES; ++ i) {
    Json::Value item = reference_splines[i];
    planningData.reference_splines[i].xb.x = item["XB_X"].asDouble();
    planningData.reference_splines[i].xb.y = item["XB_Y"].asDouble();
    planningData.reference_splines[i].xb.z = item["XB_Z"].asDouble();
    planningData.reference_splines[i].xb.w = item["XB_W"].asDouble();
    planningData.reference_splines[i].yb.x = item["YB_X"].asDouble();
    planningData.reference_splines[i].yb.y = item["YB_Y"].asDouble();
    planningData.reference_splines[i].yb.z = item["YB_Z"].asDouble();
    planningData.reference_splines[i].yb.w = item["YB_W"].asDouble();
  }

  // debug info
  Json::Value debugInfo = data["DEBUG_INFO"];
  planningData.debug_info = debugInfo.asString();
}

void QPlanningWidget::sortTrackTargets(debug_tool::ads_PlanningData4Debug &data)
{
  typedef boost::array< ::debug_tool::ads_TrackTarget_<std::allocator<void>> , 250>  \
      _track_objects_type;
  _track_objects_type &TRACKS = data.fusion_results.track_objects;
  const int SIZE = static_cast<int>(data.fusion_results.object_count);

  debug_tool::ads_TrackTarget_<std::allocator<void>> temp;
  for (int i = 0; i < SIZE; ++i) {
    double x_min = TRACKS[i].X;
    for (int j = i + 1; j < SIZE; ++j) {
      if (x_min > TRACKS[j].X) {

        memcpy(&temp, &TRACKS[i], sizeof(temp));
        memcpy(&TRACKS[i], &TRACKS[j], sizeof(temp));
        memcpy(&TRACKS[j], &temp, sizeof(temp));
        x_min = TRACKS[i].X;
      }
    }
  }
}

