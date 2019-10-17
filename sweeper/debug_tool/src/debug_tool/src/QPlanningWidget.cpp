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
#include "QDataDisplayWidget.h"
#include "QDebugToolMainWnd.h"
#include "QPlanningCostWidget.h"
#include "QFullViewWidget.h"
#include "QCostValueWidget.h"
#include "QReadDataManagerRos.h"
#include "splines.h"

QPlanningWidget::QPlanningWidget(QWidget *parent)
  : QBaseWidget(parent)
{
  this->setFont(G_TEXT_FONT);
  m_pWdgParam = new QPlanningParamWidget(this);
  m_pWdgParam->setShowType(LivePlay);
  connect(m_pWdgParam, &QPlanningParamWidget::replayState,
          this, &QPlanningWidget::onReplayState);
  connect(m_pWdgParam, &QPlanningParamWidget::replayFrameOffset,
          this, &QPlanningWidget::onSetFrameIndexReplay);
  connect(m_pWdgParam, &QPlanningParamWidget::costValueChanged,
          this, &QPlanningWidget::onCostValueChanged);

  boost::function<void(float, float, float, float)> fun = boost::bind(&QPlanningParamWidget::showMousePosition, m_pWdgParam,
                       _1, _2, _3, _4);
  for (int i = 0; i < 2; ++i) {
    m_pWdgShow[i] = new QPlanningShowWidget(this);
    m_pWdgShow[i]->setFunPosition(fun);
    connect(m_pWdgShow[i], &QBaseShowWidget::saveDataToFile,
        this, &QPlanningWidget::onSaveDataToFile);
  }
  m_pWdgShow[1]->setCostType(QPlanningShowWidget::NEW_COST);
  m_nShowType = LivePlay;
  m_nShowView = LocalView;

  m_pWdgFullView = new QFullViewWidget(this);
  m_pWdgFullView->setFunPosition(fun);
  m_pWdgFullView->hide();

  namespace fs = boost::filesystem;
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

  connect(QReadDataManagerRos::instance(), SIGNAL(planningData(const debug_tool::ads_PlanningData4Debug &)),
          this, SLOT(onParsePlanningData(const debug_tool::ads_PlanningData4Debug &)));

  QReadDataManagerRos::instance()->start_subscribe();

  m_nReplaySpeedIndex = 1;
  memset(m_dCostValue, 0, sizeof(double) * QPlanningCostWidget::Count);
}

QPlanningWidget::~QPlanningWidget()
{
}

void QPlanningWidget::timerEvent(QTimerEvent *e)
{
  int id = e->timerId();
  if (id == m_nTimerId) {
    if (m_bFlagPauseReplay) {
      return;
    }
    if (m_itFile == m_listPlanningFiles.end()) {
      return;
    }
    debug_tool::ads_PlanningData4Debug data;
    std::string name = *m_itFile;
    std::size_t index = name.find_last_of('/');
    name = name.substr(index + 1, name.length() - (index + 1) - 1);
    if (this->readFromJsonFile(*m_itFile, data)) {
      this->setPlanningData(data, QString::fromStdString(name));
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
 * @brief 暂停replay时，定位到帧
 * @param index: 距当前帧的帧数，负数向前，正数向后

 * @return
********************************************************/
void QPlanningWidget::onSetFrameIndexReplay(int frame)
{
  if (frame < 0) {
    while (frame < 0 && m_itFile != m_listPlanningFiles.begin()) {
      ++frame;
      --m_itFile;
    }
  }
  else if (frame > 0) {
    while (frame > 0 && m_itFile != m_listPlanningFiles.end()) {
      --frame;
      ++m_itFile;
    }
  }
  debug_tool::ads_PlanningData4Debug data;
  std::string full_name;
  if (m_itFile != m_listPlanningFiles.end()) {
    full_name = *m_itFile;
  }
  else {
    full_name = m_listPlanningFiles.back();
  }
  std::size_t pos = full_name.find_last_of('/');
  std::string file_name = full_name.substr(pos + 1, full_name.length() - (pos + 1) - 1);
  if (this->readFromJsonFile(full_name, data)) {
    this->setPlanningData(data, QString::fromStdString(file_name));
  }
  else {
    file_name += "-------- FAILED !!!!!!!!!!!!!!!!1";
  }
  QDebugToolMainWnd::s_pStatusBar->showMessage(QString::fromStdString(file_name));
}

void QPlanningWidget::onCostValueChanged()
{
  if (m_nShowType == RePlay) {
    this->onSetFrameIndexReplay(0);
  }
}

bool QPlanningWidget::readFromJsonFile(const std::string &name, debug_tool::ads_PlanningData4Debug &planningData)
{
  const std::string fileName = name.substr(1, name.length() - 2);
  m_strJsonFile = fileName;

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
  if (!fs::exists(m_fsPath)) {
    fs::create_directories(m_fsPath);
  }

  fs::path path = m_fsPath;
  std::string strFileName = QBaseWidget::dataFileName();
  path /= strFileName;
  strFileName = path.string();

  debug_tool::ads_PlanningData4Debug &data = const_cast<debug_tool::ads_PlanningData4Debug &>(planningData);
  this->sortTrackTargets(data);
  this->saveDataToJsonFile(strFileName, data);
  if (m_nShowType == LivePlay) {
    this->setPlanningData(data, "");
  }
}

void QPlanningWidget::saveDataToJsonFile(const std::string &strFileName,
                                         const debug_tool::ads_PlanningData4Debug &planningData)
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
  const auto &radar28Result = planningData.radar28f_results;
  Json::Value radar28Trargets, radar28Trarget;
  int sizeTarget28 = radar28Result.size();
  radar28Trargets["OBJECT_COUNT"] = sizeTarget28;
  for (int i = 0; i < sizeTarget28; ++i) {
    Json::Value item;
    const debug_tool::ads_RadarTarget &target = radar28Result[i];
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
  const auto &radar73Result = planningData.radar73f_results;
  Json::Value radar73Trargets, radar73Trarget;
  int sizeTarget73 = radar73Result.size();
  radar73Trargets["OBJECT_COUNT"] = sizeTarget73;
  for (int i = 0; i < sizeTarget73; ++i) {
    Json::Value item;
    const debug_tool::ads_RadarTarget &target = radar73Result[i];
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
  const auto &ultrasonicResult = planningData.ultrasonic_results;
  Json::Value ultrasonicTrargets, ultrasonicTrarget;
  int sizeTarget = ultrasonicResult.size();
  ultrasonicTrargets["OBJECT_COUNT"] = static_cast<int>(sizeTarget);
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item;
    const debug_tool::ads_UltraSonicTarget &target = ultrasonicResult[i];

    item["POS_ID"] = static_cast<int>(target.radar_pos_id);
    item["DISTANCE"] = target.distance;
    ultrasonicTrarget.append(item);
  }
  ultrasonicTrargets["US_OBJECTS"] = ultrasonicTrarget;

  // Track Target
  const auto &trackResult = planningData.fusion_results;
  Json::Value trackTrargets, trackTrarget;
  sizeTarget = trackResult.size();
  trackTrargets["OBJECT_COUNT"] = sizeTarget;
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item;
    const debug_tool::ads_TrackTarget &target = trackResult[i];

    item["TRACK_ID"] = static_cast<int32_t>(target.TRACK_ID);
    item["X"] = target.X;
    item["Y"] = target.Y;
    item["SX"] = target.SX;
    item["ANGLE"] = target.ANGLE;
    item["W"] = target.W;
    item["L"] = target.L;
    item["H"] = target.H;
    item["DEVICE_SOURCE"] = static_cast<int>(target.DEVICE_SOURCE);
    item["MOTION_STATUS"] = static_cast<int>(target.MOTION_STATUS);
    item["TARGET_TYPE"] = static_cast<int>(target.TARGET_TYPE);

    Json::Value json_points;
    for (const auto &point : target.edge_points) {
      Json::Value json_point;
      json_point["POINT_X"] = point.x;
      json_point["POINT_Y"] = point.y;
      json_point["POINT_Z"] = point.z;
      json_points.append(json_point);
    }
    item["POINTS"] = json_points;

    trackTrarget.append(item);
  }
  trackTrargets["TRACK_OBJECTS"] = trackTrarget;

  // garbage
  Json::Value garbage;
  const auto &garbage_result = planningData.garbage_detection_results;
  auto garbage_size = garbage_result.size();
  using size_g = decltype (garbage_size);
  for (size_g i = 0; i < garbage_size; ++i) {
    Json::Value item;
    item["ID"] = garbage_result[i].id;
    item["SIZE"] = garbage_result[i].size;
    item["ANGLE"] = garbage_result[i].angle;
    item["DISTANCE"] = garbage_result[i].distance;
    item["LENGTH"] = garbage_result[i].length;
    item["WIDTH"] = garbage_result[i].width;
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
  planning_output["DECISION"] = planningData.planning_output.decision;
  planning_output["VELOCITY"] = planningData.planning_output.velocity;
  planning_output["POSE_POSITION_X"] = planningData.planning_output.pose.position.x;
  planning_output["POSE_POSITION_Y"] = planningData.planning_output.pose.position.y;
  trajectory["PLANNING_OUTPUT"] = planning_output;

  // planning cost weight
  Json::Value planning_cost_weight;
  planning_cost_weight["SAFETY_COST_WEIGHT"] = planningData.safety_cost_weight;
  planning_cost_weight["LATERAL_COST_WEIGHT"] = planningData.lateral_cost_weight;
  planning_cost_weight["SMOOTHNESS_COST_WEIGHT"] = planningData.smoothness_cost_weight;
  planning_cost_weight["CONSISTENCY_COST_WEIGHT"] = planningData.consistency_cost_weight;
  planning_cost_weight["GARBAGE_COST_WEIGHT"] = planningData.garbage_cost_weight;
  trajectory["PLANNING_COST_WEIGHT"] = planning_cost_weight;

  // planning candidate
  Json::Value json_candidates;
  const auto &val_candidates = planningData.planning_trajectory_candidates;
  const int size_candidates = val_candidates.size();
  for (int i = 0; i < size_candidates; ++i) {
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

  // planning trajectory
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
  roadInfo["LEFT_HALF_ROAD_WIDTH"] = planningData.left_road_width;
  roadInfo["RIGHT_HALF_ROAD_WIDTH"] = planningData.right_road_width;
  roadInfo["MIN_SAFE_DISTANCE"] = planningData.safe_dis1;
  roadInfo["SAFE_DISTANCE"] = planningData.safe_dis2;
  roadInfo["MAX_PLANNING_DISTANCE"] = planningData.max_planning_distance;
  roadInfo["LEFT_ROAD_BOUNDARY_AVAILABLE"] = planningData.left_road_boundary_available;
  roadInfo["RIGHT_ROAD_BOUNDARY_AVAILABLE"] = planningData.right_road_boundary_available;
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

  const int SIZE_LEFT_ROAD_SPLINES = planningData.left_road_boundary_splines.size();
  roadInfo["NUM_LEFT_ROAD_BOUNDARY_SPLINES"] = SIZE_LEFT_ROAD_SPLINES;
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
  const int SIZE_RIGHT_ROAD_SPLINES = planningData.right_road_boundary_splines.size();
  roadInfo["NUM_RIGHT_ROAD_BOUNDARY_SPLINES"] = SIZE_RIGHT_ROAD_SPLINES;
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
  sizeTarget = planningData.reference_points.size();
  referenceLine["NUM_REFERENCE_POINTS"] = sizeTarget;
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

  const int SIZE_REFERENCE_SPLINES = planningData.reference_splines.size();
  referenceLine["NUM_REFERENCE_SPLINES"] = SIZE_REFERENCE_SPLINES;
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
  auto &radar28Result = planningData.radar28f_results;
  Json::Value radar28Trarget = radar28Trargets["RADAR28_OBJECTS"];
  int sizeTarget28 = radar28Trargets["OBJECT_COUNT"].asInt();
  for (int i = 0; i < sizeTarget28; ++i) {
    Json::Value item = radar28Trarget[i];
    debug_tool::ads_RadarTarget target;
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

    radar28Result.push_back(target);
  }

  // radar 73 target
  auto &radar73Result = planningData.radar73f_results;
  int sizeTarget73 = radar73Trargets["OBJECT_COUNT"].asInt();
  Json::Value radar73Trarget = radar73Trargets["RADAR73_OBJECTS"];
  for (int i = 0; i < sizeTarget73; ++i) {
    Json::Value item = radar73Trarget[i];
    debug_tool::ads_RadarTarget target;
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

    radar73Result.push_back(target);
  }

  // UltraSonic
  auto &ultrasonicResult = planningData.ultrasonic_results;
  Json::Value ultrasonicTrarget = ultrasonicTrargets["US_OBJECTS"];
  int sizeTarget = ultrasonicTrargets["OBJECT_COUNT"].asInt();
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item = ultrasonicTrarget[i];
    debug_tool::ads_UltraSonicTarget target;

    target.radar_pos_id = static_cast<int8_t>(item["POS_ID"].asInt());
    target.distance = item["DISTANCE"].asFloat();

    ultrasonicResult.push_back(target);
  }

  // Track Target
  auto &trackResult = planningData.fusion_results;
  Json::Value trackTrarget = trackTrargets["TRACK_OBJECTS"];
  sizeTarget = trackTrargets["OBJECT_COUNT"].asInt();
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item = trackTrarget[i];
    debug_tool::ads_TrackTarget target;

    target.TRACK_ID = static_cast<int64_t>(item["TRACK_ID"].asInt());
    target.X = item["X"].asDouble();
    target.Y = item["Y"].asDouble();
    target.SX = item["SX"].asFloat();
    target.ANGLE = item["ANGLE"].asFloat();
    target.W = item["W"].asFloat();
    target.L = item["L"].asFloat();
    target.H = item["H"].asFloat();
    target.DEVICE_SOURCE = static_cast<int8_t>(item["DEVICE_SOURCE"].asInt());
    target.MOTION_STATUS = static_cast<int8_t>(item["MOTION_STATUS"].asInt());
    target.TARGET_TYPE = static_cast<int8_t>(item["TARGET_TYPE"].asInt());

    const int size_point = item["POINTS"].size();
    for (int j = 0; j < size_point; ++j) {
      Json::Value json_point = item["POINTS"][j];
      ::geometry_msgs::Point32_<std::allocator<void>> point;
      point.x = json_point["POINT_X"].asDouble();
      point.y = json_point["POINT_Y"].asDouble();
      point.z = json_point["POINT_Z"].asDouble();

      target.edge_points.push_back(point);
    }

    trackResult.push_back(target);
  }

  // garbage
  auto &garbage_result = planningData.garbage_detection_results;
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
    result.length = item["LENGTH"].asFloat();
    result.width = item["WIDTH"].asFloat();
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

  // planning cost weight
  Json::Value planning_cost_weight = trajectory["PLANNING_COST_WEIGHT"];
  planningData.safety_cost_weight = planning_cost_weight["SAFETY_COST_WEIGHT"].asDouble();
  planningData.lateral_cost_weight = planning_cost_weight["LATERAL_COST_WEIGHT"].asDouble();
  planningData.smoothness_cost_weight = planning_cost_weight["SMOOTHNESS_COST_WEIGHT"].asDouble();
  planningData.consistency_cost_weight = planning_cost_weight["CONSISTENCY_COST_WEIGHT"].asDouble();
  planningData.garbage_cost_weight = planning_cost_weight["GARBAGE_COST_WEIGHT"].asDouble();

  // planning candidate
  auto &val_candidates = planningData.planning_trajectory_candidates;
  val_candidates.clear();
  const int size_candidates = trajectory["TRAJECTORY_CANDIDATES"].size();
  for (int i = 0; i < size_candidates; ++i) {
    debug_tool::ads_planning_trajectory_<std::allocator<void>> candidates;
    Json::Value item = trajectory["TRAJECTORY_CANDIDATES"][i];
    candidates.id = static_cast<uint8_t>(item["ID"].asInt());
    candidates.cost = item["COST"].asDouble();
    candidates.safety_cost = item["SAFETY_COST"].asDouble();
    candidates.lateral_cost = item["LATERAL_COST"].asDouble();
    candidates.smoothness_cost = item["SMOOTHNESS_COST"].asDouble();
    candidates.consistency_cost = item["CONSISTENCY_COST"].asDouble();
    candidates.garbage_cost = item["GARBAGE_COST"].asDouble();

    auto &val_candidates_spines = candidates.splines;
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
    val_candidates.push_back(candidates);
  }

  // planning trajectory
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
  planningData.left_road_width = roadInfo["LEFT_HALF_ROAD_WIDTH"].asDouble();
  planningData.right_road_width = roadInfo["RIGHT_HALF_ROAD_WIDTH"].asDouble();
  planningData.safe_dis1 = roadInfo["MIN_SAFE_DISTANCE"].asDouble();
  planningData.safe_dis2 = roadInfo["SAFE_DISTANCE"].asDouble();
  planningData.max_planning_distance = roadInfo["MAX_PLANNING_DISTANCE"].asDouble();
  planningData.left_road_boundary_available = roadInfo["LEFT_ROAD_BOUNDARY_AVAILABLE"].asBool();
  planningData.right_road_boundary_available = roadInfo["RIGHT_ROAD_BOUNDARY_AVAILABLE"].asBool();
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

  const int SIZE_LEFT_ROAD_SPLINES = roadInfo["LEFT_ROAD_BOUNDARY_SPLINES"].size();
  for (int i = 0; i < SIZE_LEFT_ROAD_SPLINES; ++ i) {
    Json::Value item = roadInfo["LEFT_ROAD_BOUNDARY_SPLINES"][i];
    debug_tool::ads_Spline spline;
    spline.xb.x = item["XB_X"].asDouble();
    spline.xb.y = item["XB_Y"].asDouble();
    spline.xb.z = item["XB_Z"].asDouble();
    spline.xb.w = item["XB_W"].asDouble();
    spline.yb.x = item["YB_X"].asDouble();
    spline.yb.y = item["YB_Y"].asDouble();
    spline.yb.z = item["YB_Z"].asDouble();
    spline.yb.w = item["YB_W"].asDouble();

    planningData.left_road_boundary_splines.push_back(spline);
  }
  const int SIZE_RIGHT_ROAD_SPLINES = roadInfo["RIGHT_ROAD_BOUNDARY_SPLINES"].size();
  for (int i = 0; i < SIZE_RIGHT_ROAD_SPLINES; ++ i) {
    Json::Value item = roadInfo["RIGHT_ROAD_BOUNDARY_SPLINES"][i];
    debug_tool::ads_Spline spline;

    spline.xb.x = item["XB_X"].asDouble();
    spline.xb.y = item["XB_Y"].asDouble();
    spline.xb.z = item["XB_Z"].asDouble();
    spline.xb.w = item["XB_W"].asDouble();
    spline.yb.x = item["YB_X"].asDouble();
    spline.yb.y = item["YB_Y"].asDouble();
    spline.yb.z = item["YB_Z"].asDouble();
    spline.yb.w = item["YB_W"].asDouble();

    planningData.right_road_boundary_splines.push_back(spline);
  }

  // referene
  Json::Value referencePoints = referenceLine["REFERENCE_POINTS"];
  sizeTarget = referenceLine["REFERENCE_POINTS"].size();
  for (int i = 0; i < sizeTarget; ++i) {
    Json::Value item = referencePoints[i];
    debug_tool::ads_ReferencePoint reference;
    reference.id = static_cast<int8_t>(item["ID"].asInt());
    reference.l = item["L"].asDouble();
    reference.s = item["S"].asDouble();
    reference.x = item["X"].asDouble();
    reference.y = item["Y"].asDouble();

    planningData.reference_points.push_back(reference);
  }

  Json::Value reference_splines = referenceLine["REFERENCE_SPLINES"];
  const int SIZE_REFERENCE_SPLINES = reference_splines.size();
  for (int i = 0; i < SIZE_REFERENCE_SPLINES; ++ i) {
    Json::Value item = reference_splines[i];
    debug_tool::ads_Spline spline;

    spline.xb.x = item["XB_X"].asDouble();
    spline.xb.y = item["XB_Y"].asDouble();
    spline.xb.z = item["XB_Z"].asDouble();
    spline.xb.w = item["XB_W"].asDouble();
    spline.yb.x = item["YB_X"].asDouble();
    spline.yb.y = item["YB_Y"].asDouble();
    spline.yb.z = item["YB_Z"].asDouble();
    spline.yb.w = item["YB_W"].asDouble();

    planningData.reference_splines.push_back(spline);
  }

  // debug info
  Json::Value debugInfo = data["DEBUG_INFO"];
  planningData.debug_info = debugInfo.asString();
}

void QPlanningWidget::sortTrackTargets(debug_tool::ads_PlanningData4Debug &data)
{
  auto &tracks = data.fusion_results;

  using TypeTrack = decltype(tracks[0]);
  std::sort(tracks.begin(), tracks.end(), [](const TypeTrack &track,
            const TypeTrack &track2) {
    return (track.edge_points[0].x) < (track2.edge_points[0].x);
  });
}

void QPlanningWidget::setPlanningData(debug_tool::ads_PlanningData4Debug &data,
                                      const QString &name)
{
  double cost_value[QPlanningCostWidget::Count] = {0};
  cost_value[QPlanningCostWidget::Safety] = data.safety_cost_weight;
  cost_value[QPlanningCostWidget::Lateral] = data.lateral_cost_weight;
  cost_value[QPlanningCostWidget::Smoothness] = data.smoothness_cost_weight;
  cost_value[QPlanningCostWidget::Consistency] = data.consistency_cost_weight;
  cost_value[QPlanningCostWidget::Garbage] = data.garbage_cost_weight;
  QCostValueWidget::setOriginCostValue(cost_value);

  debug_tool::ads_PlanningData4Debug data_cost = this->calcPlanningPathWitCost(data);
  if (m_nShowView == LocalView) {
    m_pWdgShow[0]->setPlanningData(data);
    m_pWdgShow[1]->setPlanningData(data_cost);
    m_pWdgFullView->setPlanningData(data, name, false);
  }
  else {
    m_pWdgFullView->setPlanningData(data, name, true);
  }
  m_pWdgParam->setPlanningData(data, data_cost);
  QDebugToolMainWnd::s_pTextBrowser->setPlainText(QString::fromStdString(data.debug_info));
  QDebugToolMainWnd::s_pDataDisplay->setPlanningData(data);
  QDebugToolMainWnd::s_pWdgPlanningCost->setPlanningData(data);
}

bool QPlanningWidget::ObstacleCollisionCheck(
    const debug_tool::ads_planning_trajectory &path,
    const debug_tool::ads_TrackTarget &target,
    double head_distance,
    double vehicle_width,
    double tolerance
    )
{
  std::vector<SplineLib::Vec2f> points;
  double x[4], y[4];

  for (const auto &point : target.edge_points) {
    SplineLib::Vec2f spline_point = { point.x, point.y };
    points.push_back(spline_point);
  }

  for (int i = 0; i < 4; i++)
  {
    int segmentNum = ceil(sqrt(pow(x[(i + 1) % 4] - x[i], 2) + pow(y[(i + 1) % 4] - y[i], 2)) / vehicle_width / 2);
    for (int j = 1; j < segmentNum; j++)
    {
      double lambda = (double) j / (segmentNum - j);
      SplineLib::Vec2f point = { (x[i] + lambda * x[(i + 1) % 4]) / (1 + lambda), (y[i] + lambda * y[(i + 1) % 4]) / (1 + lambda) };
      points.push_back(point);
    }
  }

  int splines_num = path.splines.size();
  SplineLib::cSpline2 *splines = new SplineLib::cSpline2[splines_num];
  for (int i = 0; i < splines_num; ++i) {
    splines[i].xb.x = path.splines[i].xb.x;
    splines[i].xb.y = path.splines[i].xb.y;
    splines[i].xb.z = path.splines[i].xb.z;
    splines[i].xb.w = path.splines[i].xb.w;

    splines[i].yb.x = path.splines[i].yb.x;
    splines[i].yb.y = path.splines[i].yb.y;
    splines[i].yb.z = path.splines[i].yb.z;
    splines[i].yb.w = path.splines[i].yb.w;
  }
  bool ret = false;
  for (int i = 0; i < points.size(); i++)
  {
    SplineLib::Vec2f qp = points.at(i);
    int index;
    double t = SplineLib::FindClosestPoint(qp, splines_num, splines, &index);
    SplineLib::Vec2f cp = SplineLib::Position(splines[index], t);
    double distance = sqrt(pow(cp.x - qp.x, 2) + pow(cp.y - qp.y, 2));
    double threshold = qp.x > head_distance ? vehicle_width / 2 + tolerance : vehicle_width / 2;
    if (distance < threshold) {
      ret = true;
      break;
    }
  }
  delete []splines;

  return ret;
}

void QPlanningWidget::onSaveDataToFile(const debug_tool::ads_PlanningData4Debug &data)
{
  this->saveDataToJsonFile(m_strJsonFile, data);
  std::size_t pos = m_strJsonFile.find_last_of('/');
  std::string file_name = m_strJsonFile.substr(pos + 1, m_strJsonFile.length() - (pos + 1) - 1);

  debug_tool::ads_PlanningData4Debug dataNew = data;
  this->setPlanningData(dataNew, QString::fromStdString(file_name));
}

bool QPlanningWidget::RoadBoundaryCheck(
    const debug_tool::ads_planning_trajectory &path,
    const std::vector< ::debug_tool::ads_ReferencePoint_<std::allocator<void>> > &references,
    double left_road_width,
    double right_road_width,
    double vehicle_width,
    double tolerance)
{
  std::vector<SplineLib::Vec2f> points;

  for (int i = 0; i < 21; i++)
  {
    double x1 = references[i].x;
    double y1 = references[i].y;
    double x2 = references[i + 1].x;
    double y2 = references[i + 1].y;
    double l2_norm = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    double leftX = x1 + left_road_width * (y1 - y2) / l2_norm;
    double leftY = y1 + left_road_width * (x2 - x1) / l2_norm;
    SplineLib::Vec2f leftPoint = { leftX, leftY };
    points.push_back(leftPoint);
    double rightX = x1 - right_road_width * (y1 - y2) / l2_norm;
    double rightY = y1 - right_road_width * (x2 - x1) / l2_norm;
    SplineLib::Vec2f rightPoint = { rightX, rightY };
    points.push_back(rightPoint);
  }

  int splines_num = path.splines.size();
  SplineLib::cSpline2 *splines = new SplineLib::cSpline2[splines_num];
  for (int i = 0; i < splines_num; ++i) {
    splines[i].xb.x = path.splines[i].xb.x;
    splines[i].xb.y = path.splines[i].xb.y;
    splines[i].xb.z = path.splines[i].xb.z;
    splines[i].xb.w = path.splines[i].xb.w;

    splines[i].yb.x = path.splines[i].yb.x;
    splines[i].yb.y = path.splines[i].yb.y;
    splines[i].yb.z = path.splines[i].yb.z;
    splines[i].yb.w = path.splines[i].yb.w;
  }
  bool ret = false;
  for (int i = 0; i < points.size(); i++)
  {
    SplineLib::Vec2f qp = points.at(i);
    int index;
    double t = SplineLib::FindClosestPoint(qp, splines_num, splines, &index);
    SplineLib::Vec2f cp = SplineLib::Position(splines[index], t);
    double distance = sqrt(pow(cp.x - qp.x, 2) + pow(cp.y - qp.y, 2));
    if (distance < vehicle_width / 2 + tolerance) {
      ret = true;
      break;
    }
  }
  delete []splines;

  return ret;
}

debug_tool::ads_PlanningData4Debug QPlanningWidget::calcPlanningPathWitCost(
    const debug_tool::ads_PlanningData4Debug &data)
{
  debug_tool::ads_PlanningData4Debug planningData = data;
  auto &candidates = planningData.planning_trajectory_candidates;
  int size_candidates = candidates.size();
  double value[QPlanningCostWidget::Count];
  QCostValueWidget::getCostValue(value);
  for (int i = 0; i < size_candidates; ++ i) {
    candidates[i].cost = value[QPlanningCostWidget::Safety] * candidates[i].safety_cost
        + value[QPlanningCostWidget::Lateral] * candidates[i].lateral_cost
        + value[QPlanningCostWidget::Smoothness] * candidates[i].smoothness_cost
        + value[QPlanningCostWidget::Consistency] * candidates[i].consistency_cost
        + value[QPlanningCostWidget::Garbage] * candidates[i].garbage_cost;
  }
  using type_candidates = decltype(planningData.planning_trajectory);
  std::sort(candidates.begin(), candidates.end(), [](const type_candidates &val,
            const type_candidates &val2){
    return val.cost < val2.cost;
  });

  int index = -1;
  int targets_count = planningData.fusion_results.size();
  const auto &targets = planningData.fusion_results;
  for (int i = 0; i < size_candidates; ++ i) {
    bool check = false;
    for (int j = 0; j < targets_count; ++j) {
      check = this->ObstacleCollisionCheck(
            candidates[i], targets[j],
            planningData.head_distance, planningData.vehicle_width, 0.75
            );
      if (check) break;
    }
    if (check) continue;
    check = this->RoadBoundaryCheck(
          candidates[i], planningData.reference_points,
          planningData.left_road_width,
          planningData.right_road_width,
          planningData.vehicle_width,
          0
          );
    if (!check) {
      index = i;
      break;
    }
  }

  if (index != -1) {
    planningData.planning_trajectory = candidates[index];
  }
  else {
    planningData.planning_trajectory.id = 0;
    planningData.planning_trajectory.cost = 0;
    planningData.planning_trajectory.safety_cost = 0;
    planningData.planning_trajectory.lateral_cost = 0;
    planningData.planning_trajectory.smoothness_cost = 0;
    planningData.planning_trajectory.consistency_cost = 0;
    planningData.planning_trajectory.garbage_cost = 0;
    planningData.planning_trajectory.splines.clear();
  }

  return planningData;
}


