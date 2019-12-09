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
  m_nShowView = LocalViewVehicle;

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
  // vehicle
  Json::Value carStatus, frontAxleCenter, rearAxleCenter, headPoint, rearPoint, hingePoint;

  frontAxleCenter["id"] = planningData.front_axle_center.id;
  frontAxleCenter["x"] = planningData.front_axle_center.x;
  frontAxleCenter["y"] = planningData.front_axle_center.y;
  frontAxleCenter["enu_x"] = planningData.front_axle_center.enu_x;
  frontAxleCenter["enu_y"] = planningData.front_axle_center.enu_y;
  frontAxleCenter["s"] = planningData.front_axle_center.s;
  frontAxleCenter["l"] = planningData.front_axle_center.l;
  frontAxleCenter["left_road_width"] = planningData.front_axle_center.left_road_width;
  frontAxleCenter["right_road_width"] = planningData.front_axle_center.right_road_width;

  rearAxleCenter["id"] = planningData.rear_axle_center.id;
  rearAxleCenter["x"] = planningData.rear_axle_center.x;
  rearAxleCenter["y"] = planningData.rear_axle_center.y;
  rearAxleCenter["enu_x"] = planningData.rear_axle_center.enu_x;
  rearAxleCenter["enu_y"] = planningData.rear_axle_center.enu_y;
  rearAxleCenter["s"] = planningData.rear_axle_center.s;
  rearAxleCenter["l"] = planningData.rear_axle_center.l;
  rearAxleCenter["left_road_width"] = planningData.rear_axle_center.left_road_width;
  rearAxleCenter["right_road_width"] = planningData.rear_axle_center.right_road_width;

  headPoint["id"] = planningData.head_point.id;
  headPoint["x"] = planningData.head_point.x;
  headPoint["y"] = planningData.head_point.y;
  headPoint["enu_x"] = planningData.head_point.enu_x;
  headPoint["enu_y"] = planningData.head_point.enu_y;
  headPoint["s"] = planningData.head_point.s;
  headPoint["l"] = planningData.head_point.l;
  headPoint["left_road_width"] = planningData.head_point.left_road_width;
  headPoint["right_road_width"] = planningData.head_point.right_road_width;

  rearPoint["id"] = planningData.rear_point.id;
  rearPoint["x"] = planningData.rear_point.x;
  rearPoint["y"] = planningData.rear_point.y;
  rearPoint["enu_x"] = planningData.rear_point.enu_x;
  rearPoint["enu_y"] = planningData.rear_point.enu_y;
  rearPoint["s"] = planningData.rear_point.s;
  rearPoint["l"] = planningData.rear_point.l;
  rearPoint["left_road_width"] = planningData.rear_point.left_road_width;
  rearPoint["right_road_width"] = planningData.rear_point.right_road_width;

  hingePoint["id"] = planningData.hinge_point.id;
  hingePoint["x"] = planningData.hinge_point.x;
  hingePoint["y"] = planningData.hinge_point.y;
  hingePoint["enu_x"] = planningData.hinge_point.enu_x;
  hingePoint["enu_y"] = planningData.hinge_point.enu_y;
  hingePoint["s"] = planningData.hinge_point.s;
  hingePoint["l"] = planningData.hinge_point.l;
  hingePoint["left_road_width"] = planningData.hinge_point.left_road_width;
  hingePoint["right_road_width"] = planningData.hinge_point.right_road_width;

  carStatus["front_axle_center"] = frontAxleCenter;
  carStatus["rear_axle_center"] = rearAxleCenter;
  carStatus["head_point"] = headPoint;
  carStatus["rear_point"] = rearPoint;
  carStatus["hinge_point"] = hingePoint;
  carStatus["front_vehicle_length"] = planningData.front_vehicle_length;
  carStatus["front_vehicle_width"] = planningData.front_vehicle_width;
  carStatus["rear_vehicle_length"] = planningData.rear_vehicle_length;
  carStatus["rear_vehicle_width"] = planningData.rear_vehicle_width;
  carStatus["steering_angle"] = planningData.steering_angle;

  // referene
  Json::Value referenceLine, referencePoints, referenceSplines;
  for (const auto &point : planningData.reference_points) {
    Json::Value item;
    item["id"] = point.id;
    item["x"] = point.x;
    item["y"] = point.y;
    item["enu_x"] = point.enu_x;
    item["enu_y"] = point.enu_y;
    item["s"] = point.s;
    item["l"] = point.l;
    item["left_road_width"] = point.left_road_width;
    item["right_road_width"] = point.right_road_width;
    referencePoints.append(item);
  }
  referenceLine["reference_points"] = referencePoints;

  for (const auto &spline : planningData.reference_splines) {
    Json::Value item;
    item["XB_X"] = spline.xb.x;
    item["XB_Y"] = spline.xb.y;
    item["XB_Z"] = spline.xb.z;
    item["XB_W"] = spline.xb.w;
    item["YB_X"] = spline.yb.x;
    item["YB_Y"] = spline.yb.y;
    item["YB_Z"] = spline.yb.z;
    item["YB_W"] = spline.yb.w;
    referenceSplines.append(item);
  }
  referenceLine["reference_splines"] = referenceSplines;

  // radar
  Json::Value radarData, radarPoint, radarResult;

  radarPoint["id"] = planningData.radar_Point.id;
  radarPoint["x"] = planningData.radar_Point.x;
  radarPoint["y"] = planningData.radar_Point.y;
  radarPoint["enu_x"] = planningData.radar_Point.enu_x;
  radarPoint["enu_y"] = planningData.radar_Point.enu_y;
  radarPoint["s"] = planningData.radar_Point.s;
  radarPoint["l"] = planningData.radar_Point.l;
  radarPoint["left_road_width"] = planningData.radar_Point.left_road_width;
  radarPoint["right_road_width"] = planningData.radar_Point.right_road_width;

  for (const auto &target : planningData.radar_results) {
    Json::Value item;
    item["id"] = target.id;
    item["range"] = target.range;
    item["range_lat"] = target.range_lat;
    item["range_lon"] = target.range_lon;
    item["angle"] = target.angle;
    item["vel"] = target.vel;
    item["v_lat"] = target.v_lat;
    item["v_lon"] = target.v_lon;
    item["status"] = target.status;
    item["w"] = target.w;
    item["l"] = target.l;
    item["devid"] = target.devid;
    radarResult.append(item);
  }

  radarData["radar_point"] = radarPoint;
  radarData["radar_results"] = radarResult;

  // ultrasonic
  Json::Value ultrasonicData, ultrasonicPoint, ultrasonicResults;

  for (const auto &point : planningData.ultrasonic_points) {
    Json::Value item;
    item["id"] = point.id;
    item["x"] = point.x;
    item["y"] = point.y;
    item["enu_x"] = point.enu_x;
    item["enu_y"] = point.enu_y;
    item["s"] = point.s;
    item["l"] = point.l;
    item["left_road_width"] = point.left_road_width;
    item["right_road_width"] = point.right_road_width;
    ultrasonicPoint.append(item);
  }

  for (const auto &target : planningData.ultrasonic_results) {
    Json::Value item;
    item["radar_pos_id"] = static_cast<int>(target.radar_pos_id);
    item["distance"] = target.distance;
    ultrasonicResults.append(item);
  }
  ultrasonicData["ultrasonic_points"] = ultrasonicPoint;
  ultrasonicData["ultrasonic_results"] = ultrasonicResults;

  // track target
  Json::Value fusionResults;
  for (const auto &target : planningData.fusion_results) {
    Json::Value item;

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

    Json::Value edgePoints;
    for (const auto &point : target.edge_points) {
      Json::Value childItem;
      childItem["x"] = point.x;
      childItem["y"] = point.y;
      childItem["z"] = point.z;
      edgePoints.append(childItem);
    }
    item["edge_points"] = edgePoints;

    fusionResults.append(item);
  }

  // gargage
  Json::Value garbageDetectionResults;
  for (const auto &result : planningData.garbage_detection_results) {
    Json::Value item;
    item["id"] = static_cast<int>(result.id);
    item["size"] = result.size;
    item["angle"] = result.angle;
    item["distance"] = result.distance;
    item["length"] = result.length;
    item["width"] = result.width;
    garbageDetectionResults.append(item);
  }

  // decision
  Json::Value decisionState;
  decisionState["decision"] = static_cast<int>(planningData.decision);
  decisionState["radar_decision"] = static_cast<int>(planningData.radar_decision);
  decisionState["ultrasonic_decision"] = static_cast<int>(planningData.ultrasonic_decision);
  decisionState["track_target_decision"] = static_cast<int>(planningData.track_target_decision);

  // cost
  Json::Value costWeight;
  costWeight["safety_cost_weight"] = planningData.safety_cost_weight;
  costWeight["lateral_cost_weight"] = planningData.lateral_cost_weight;
  costWeight["smoothness_cost_weight"] = planningData.smoothness_cost_weight;
  costWeight["consistency_cost_weight"] = planningData.consistency_cost_weight;
  costWeight["garbage_cost_weight"] = planningData.garbage_cost_weight;

  // planning trajectory candidates
  Json::Value planningTrajectoryCandidates;
  for (const auto &trajectory : planningData.planning_trajectory_candidates) {
    Json::Value item, itemSplines;
    item["id"] = static_cast<int>(trajectory.id);
    item["cost"] = trajectory.cost;
    item["safety_cost"] = trajectory.safety_cost;
    item["lateral_cost"] = trajectory.lateral_cost;
    item["smoothness_cost"] = trajectory.smoothness_cost;
    item["consistency_cost"] = trajectory.consistency_cost;
    item["garbage_cost"] = trajectory.garbage_cost;

    for (const auto &spline : trajectory.splines) {
      Json::Value chile_item;

      chile_item["xb_x"] = spline.xb.x;
      chile_item["xb_y"] = spline.xb.y;
      chile_item["xb_z"] = spline.xb.z;
      chile_item["xb_w"] = spline.xb.w;
      chile_item["yb_x"] = spline.yb.x;
      chile_item["yb_y"] = spline.yb.y;
      chile_item["yb_z"] = spline.yb.z;
      chile_item["yb_w"] = spline.yb.w;

      itemSplines.append(chile_item);
    }
    item["splines"] = itemSplines;
    planningTrajectoryCandidates.append(item);
  }

  // planning trajectory
  Json::Value planningTrajectory, planningTrajectorySplines;
  planningTrajectory["id"] = static_cast<int>(planningData.planning_trajectory.id);
  planningTrajectory["cost"] = planningData.planning_trajectory.cost;
  planningTrajectory["safety_cost"] = planningData.planning_trajectory.safety_cost;
  planningTrajectory["lateral_cost"] = planningData.planning_trajectory.lateral_cost;
  planningTrajectory["smoothness_cost"] = planningData.planning_trajectory.smoothness_cost;
  planningTrajectory["consistency_cost"] = planningData.planning_trajectory.consistency_cost;
  planningTrajectory["garbage_cost"] = planningData.planning_trajectory.garbage_cost;

  for (const auto &spline : planningData.planning_trajectory.splines) {
    Json::Value item;

    item["xb_x"] = spline.xb.x;
    item["xb_y"] = spline.xb.y;
    item["xb_z"] = spline.xb.z;
    item["xb_w"] = spline.xb.w;
    item["yb_x"] = spline.yb.x;
    item["yb_y"] = spline.yb.y;
    item["yb_z"] = spline.yb.z;
    item["yb_w"] = spline.yb.w;

    planningTrajectorySplines.append(item);
  }
  planningTrajectory["splines"] = planningTrajectorySplines;

  // planning output
  Json::Value planningOutput;
  planningOutput["decision"] = static_cast<int>(planningData.planning_output.decision);
  planningOutput["velocity"] = planningData.planning_output.velocity;
  planningOutput["pose_position_x"] = planningData.planning_output.pose.position.x;
  planningOutput["pose_position_y"] = planningData.planning_output.pose.position.y;

  // all data
  Json::Value data;
  data["car_status"] = carStatus;
  data["reference_line"] = referenceLine;
  data["radar_data"] = radarData;
  data["ultrasonic_data"] = ultrasonicData;
  data["fusion_results"] = fusionResults;
  data["garbage_detection_results"] = garbageDetectionResults;
  data["decision_state"] = decisionState;
  data["cost_weight"] = costWeight;
  data["planning_trajectory_candidates"] = planningTrajectoryCandidates;
  data["planning_trajectory"] = planningTrajectory;
  data["planning_output"] = planningOutput;
  data["debug_info"] = planningData.debug_info;

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

  // all data
  Json::Value carStatus = data["car_status"];
  Json::Value referenceLine = data["reference_line"];
  Json::Value radarData = data["radar_data"];
  Json::Value ultrasonicData = data["ultrasonic_data"];
  Json::Value fusionResults = data["fusion_results"];
  Json::Value garbageDetectionResults = data["garbage_detection_results"];
  Json::Value decisionState = data["decision_state"];
  Json::Value costWeight = data["cost_weight"];
  Json::Value planningTrajectoryCandidates = data["planning_trajectory_candidates"];
  Json::Value planningTrajectory = data["planning_trajectory"];
  Json::Value planningOutput = data["planning_output"];
  planningData.debug_info = data["debug_info"].asString();

  // vehicle
  Json::Value frontAxleCenter = carStatus["front_axle_center"];
  Json::Value rearAxleCenter = carStatus["rear_axle_center"];
  Json::Value headPoint = carStatus["head_point"];
  Json::Value rearPoint = carStatus["rear_point"];
  Json::Value hingePoint = carStatus["hinge_point"];
  planningData.front_vehicle_length = carStatus["front_vehicle_length"].asDouble();
  planningData.front_vehicle_width = carStatus["front_vehicle_width"].asDouble();
  planningData.rear_vehicle_length = carStatus["rear_vehicle_length"].asDouble();
  planningData.rear_vehicle_width = carStatus["rear_vehicle_width"].asDouble();
  planningData.steering_angle = carStatus["steering_angle"].asDouble();

  planningData.front_axle_center.id = frontAxleCenter["id"].asInt();
  planningData.front_axle_center.x = frontAxleCenter["x"].asDouble();
  planningData.front_axle_center.y = frontAxleCenter["y"].asDouble();
  planningData.front_axle_center.enu_x = frontAxleCenter["enu_x"].asDouble();
  planningData.front_axle_center.enu_y = frontAxleCenter["enu_y"].asDouble();
  planningData.front_axle_center.s = frontAxleCenter["s"].asDouble();
  planningData.front_axle_center.l = frontAxleCenter["l"].asDouble();
  planningData.front_axle_center.left_road_width = frontAxleCenter["left_road_width"].asDouble();
  planningData.front_axle_center.right_road_width = frontAxleCenter["right_road_width"].asDouble();

  planningData.rear_axle_center.id = rearAxleCenter["id"].asInt();
  planningData.rear_axle_center.x = rearAxleCenter["x"].asDouble();
  planningData.rear_axle_center.y = rearAxleCenter["y"].asDouble();
  planningData.rear_axle_center.enu_x = rearAxleCenter["enu_x"].asDouble();
  planningData.rear_axle_center.enu_y = rearAxleCenter["enu_y"].asDouble();
  planningData.rear_axle_center.s = rearAxleCenter["s"].asDouble();
  planningData.rear_axle_center.l = rearAxleCenter["l"].asDouble();
  planningData.rear_axle_center.left_road_width = rearAxleCenter["left_road_width"].asDouble();
  planningData.rear_axle_center.right_road_width = rearAxleCenter["right_road_width"].asDouble();

  planningData.head_point.id = headPoint["id"].asInt();
  planningData.head_point.x = headPoint["x"].asDouble();
  planningData.head_point.y = headPoint["y"].asDouble();
  planningData.head_point.enu_x = headPoint["enu_x"].asDouble();
  planningData.head_point.enu_y = headPoint["enu_y"].asDouble();
  planningData.head_point.s = headPoint["s"].asDouble();
  planningData.head_point.l = headPoint["l"].asDouble();
  planningData.head_point.left_road_width = headPoint["left_road_width"].asDouble();
  planningData.head_point.right_road_width = headPoint["right_road_width"].asDouble();

  planningData.rear_point.id = rearPoint["id"].asInt();
  planningData.rear_point.x = rearPoint["x"].asDouble();
  planningData.rear_point.y = rearPoint["y"].asDouble();
  planningData.rear_point.enu_x = rearPoint["enu_x"].asDouble();
  planningData.rear_point.enu_y = rearPoint["enu_y"].asDouble();
  planningData.rear_point.s = rearPoint["s"].asDouble();
  planningData.rear_point.l = rearPoint["l"].asDouble();
  planningData.rear_point.left_road_width = rearPoint["left_road_width"].asDouble();
  planningData.rear_point.right_road_width = rearPoint["right_road_width"].asDouble();

  planningData.hinge_point.id = hingePoint["id"].asInt();
  planningData.hinge_point.x = hingePoint["x"].asDouble();
  planningData.hinge_point.y = hingePoint["y"].asDouble();
  planningData.hinge_point.enu_x = hingePoint["enu_x"].asDouble();
  planningData.hinge_point.enu_y = hingePoint["enu_y"].asDouble();
  planningData.hinge_point.s = hingePoint["s"].asDouble();
  planningData.hinge_point.l = hingePoint["l"].asDouble();
  planningData.hinge_point.left_road_width = hingePoint["left_road_width"].asDouble();
  planningData.hinge_point.right_road_width = hingePoint["right_road_width"].asDouble();

  // referene
  Json::Value referencePoints = referenceLine["reference_points"];
  const int size_reference_points = referencePoints.size();
  planningData.reference_points.resize(size_reference_points);
  for (int i = 0; i < size_reference_points; ++i) {
    auto &point = planningData.reference_points[i];
    Json::Value item = referencePoints[i];

    point.id = item["id"].asInt();
    point.x = item["x"].asDouble();
    point.y = item["y"].asDouble();
    point.enu_x = item["enu_x"].asDouble();
    point.enu_y = item["enu_y"].asDouble();
    point.s = item["s"].asDouble();
    point.l = item["l"].asDouble();
    point.left_road_width = item["left_road_width"].asDouble();
    point.right_road_width = item["right_road_width"].asDouble();
  }

  Json::Value referenceSplines = referenceLine["reference_splines"];
  const int size_reference_splines = referenceSplines.size();
  planningData.reference_splines.resize(size_reference_splines);
  for (int i = 0; i < size_reference_splines; ++i) {
    auto &spline = planningData.reference_splines[i];
    Json::Value item = referenceSplines[i];

    spline.xb.x = item["XB_X"].asDouble();
    spline.xb.y = item["XB_Y"].asDouble();
    spline.xb.z = item["XB_Z"].asDouble();
    spline.xb.w = item["XB_W"].asDouble();
    spline.yb.x = item["YB_X"].asDouble();
    spline.yb.y = item["YB_Y"].asDouble();
    spline.yb.z = item["YB_Z"].asDouble();
    spline.yb.w = item["YB_W"].asDouble();
  }

  // radar
  Json::Value radarPoint = radarData["radar_point"];
  Json::Value radarResult = radarData["radar_results"];

  planningData.radar_Point.id = radarPoint["id"].asInt();
  planningData.radar_Point.x = radarPoint["x"].asDouble();
  planningData.radar_Point.y = radarPoint["y"].asDouble();
  planningData.radar_Point.enu_x = radarPoint["enu_x"].asDouble();
  planningData.radar_Point.enu_y = radarPoint["enu_y"].asDouble();
  planningData.radar_Point.s = radarPoint["s"].asDouble();
  planningData.radar_Point.l = radarPoint["l"].asDouble();
  planningData.radar_Point.left_road_width = radarPoint["left_road_width"].asDouble();
  planningData.radar_Point.right_road_width = radarPoint["right_road_width"].asDouble();

  const int size_radar_result = radarResult.size();
  planningData.radar_results.resize(size_radar_result);
  for (int i = 0; i < size_radar_result; ++i) {
    auto &target = planningData.radar_results[i];
    Json::Value item = radarResult[i];

    target.id = item["id"].asInt();
    target.range = item["range"].asDouble();
    target.range_lat = item["range_lat"].asDouble();
    target.range_lon = item["range_lon"].asDouble();
    target.angle = item["angle"].asDouble();
    target.vel = item["vel"].asDouble();
    target.v_lat = item["v_lat"].asDouble();
    target.v_lon = item["v_lon"].asDouble();
    target.status = item["status"].asInt();
    target.w = item["w"].asDouble();
    target.l = item["l"].asDouble();
    target.devid = item["devid"].asInt();
  }

  // ultrasonic
  Json::Value ultrasonicPoints = ultrasonicData["ultrasonic_points"];
  Json::Value ultrasonicResults = ultrasonicData["ultrasonic_results"];

  const int size_ultrasonic_points = qMin<int>(14, ultrasonicPoints.size());
  for (int i = 0; i < size_ultrasonic_points; ++i) {
    auto &point = planningData.ultrasonic_points[i];
    Json::Value item = ultrasonicPoints[i];

    point.id = item["id"].asInt();
    point.x = item["x"].asDouble();
    point.y = item["y"].asDouble();
    point.enu_x = item["enu_x"].asDouble();
    point.enu_y = item["enu_y"].asDouble();
    point.s = item["s"].asDouble();
    point.l = item["l"].asDouble();
    point.left_road_width = item["left_road_width"].asDouble();
    point.right_road_width = item["right_road_width"].asDouble();
  }

  const int size_ultrasonic_results = ultrasonicResults.size();
  planningData.ultrasonic_results.resize(size_ultrasonic_results);
  for (int i = 0; i < size_ultrasonic_results; ++i) {
    auto &result = planningData.ultrasonic_results[i];
    Json::Value item = ultrasonicResults[i];

    result.radar_pos_id =  static_cast<int8_t>(item["radar_pos_id"].asInt());
    result.distance = item["distance"].asDouble();
  }

  // track target
  const int size_fusion_results = fusionResults.size();
  planningData.fusion_results.resize(size_fusion_results);
  for (int i = 0; i < size_fusion_results; ++i) {
    auto &target = planningData.fusion_results[i];
    Json::Value item = fusionResults[i];

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

    Json::Value edgePoints = item["edge_points"];
    const int size_edge_points = edgePoints.size();
    target.edge_points.resize(size_edge_points);
    for (int j = 0; j < size_edge_points; ++j) {
      auto &point = target.edge_points[j];
      Json::Value childItem = edgePoints[j];

      point.x = childItem["x"].asDouble();
      point.y = childItem["y"].asDouble();
      point.z = childItem["z"].asDouble();
    }
  }

  // gargage
  const int size_garbage_detection_results = garbageDetectionResults.size();
  planningData.garbage_detection_results.resize(size_garbage_detection_results);
  for (int i = 0; i < size_garbage_detection_results; ++i) {
    auto &result = planningData.garbage_detection_results[i];
    Json::Value item = garbageDetectionResults[i];

    result.id = static_cast<uint8_t>(item["id"].asInt());
    result.size = item["size"].asFloat();
    result.angle = item["angle"].asFloat();
    result.distance = item["distance"].asFloat();
    result.length = item["length"].asFloat();
    result.width = item["width"].asFloat();
  }

  // decision
  planningData.decision = static_cast<uint8_t>(decisionState["decision"].asInt());
  planningData.radar_decision = static_cast<uint8_t>(decisionState["radar_decision"].asInt());
  planningData.ultrasonic_decision = static_cast<uint8_t>(decisionState["ultrasonic_decision"].asInt());
  planningData.track_target_decision = static_cast<uint8_t>(decisionState["track_target_decision"].asInt());

  // cost
  planningData.safety_cost_weight = costWeight["safety_cost_weight"].asDouble();
  planningData.lateral_cost_weight = costWeight["lateral_cost_weight"].asDouble();
  planningData.smoothness_cost_weight = costWeight["smoothness_cost_weight"].asDouble();
  planningData.consistency_cost_weight = costWeight["consistency_cost_weight"].asDouble();
  planningData.garbage_cost_weight = costWeight["garbage_cost_weight"].asDouble();

  // planning trajectory candidates
  const int size_planning_trajectory_candidates = planningTrajectoryCandidates.size();
  planningData.planning_trajectory_candidates.resize(size_planning_trajectory_candidates);
  for (int i = 0; i < size_planning_trajectory_candidates; ++i) {
    auto &trajectory = planningData.planning_trajectory_candidates[i];
    Json::Value item = planningTrajectoryCandidates[i];

    trajectory.id = static_cast<uint8_t>(item["id"].asInt());
    trajectory.cost = item["cost"].asDouble();
    trajectory.safety_cost = item["safety_cost"].asDouble();
    trajectory.lateral_cost = item["lateral_cost"].asDouble();
    trajectory.smoothness_cost = item["smoothness_cost"].asDouble();
    trajectory.consistency_cost = item["consistency_cost"].asDouble();
    trajectory.garbage_cost = item["garbage_cost"].asDouble();

    Json::Value itemSplines = item["splines"];
    int size_trajectory_splines = itemSplines.size();
    trajectory.splines.resize(size_trajectory_splines);
    for (int j = 0; j < size_trajectory_splines; ++j) {
      Json::Value chile_item = itemSplines[j];
      auto &spline = trajectory.splines[j];

      spline.xb.x = chile_item["xb_x"].asDouble();
      spline.xb.y = chile_item["xb_y"].asDouble();
      spline.xb.z = chile_item["xb_z"].asDouble();
      spline.xb.w = chile_item["xb_w"].asDouble();
      spline.xb.x = chile_item["yb_x"].asDouble();
      spline.xb.y = chile_item["yb_y"].asDouble();
      spline.xb.z = chile_item["yb_z"].asDouble();
      spline.xb.w = chile_item["yb_w"].asDouble();
    }
  }

  // planning trajectory
  auto &planning_trajectory = planningData.planning_trajectory;

  planning_trajectory.id = static_cast<uint8_t>(planningTrajectory["id"].asInt());
  planning_trajectory.cost = planningTrajectory["cost"].asDouble();
  planning_trajectory.safety_cost = planningTrajectory["safety_cost"].asDouble();
  planning_trajectory.lateral_cost = planningTrajectory["lateral_cost"].asDouble();
  planning_trajectory.smoothness_cost = planningTrajectory["smoothness_cost"].asDouble();
  planning_trajectory.consistency_cost = planningTrajectory["consistency_cost"].asDouble();
  planning_trajectory.garbage_cost = planningTrajectory["garbage_cost"].asDouble();

  Json::Value planningTrajectorySplines = planningTrajectory["splines"];
  int size_planning_trajectory_splines = planningTrajectorySplines.size();
  planning_trajectory.splines.resize(size_planning_trajectory_splines);
  for (int j = 0; j < size_planning_trajectory_splines; ++j) {
    Json::Value chile_item = planningTrajectorySplines[j];
    auto &spline = planning_trajectory.splines[j];

    spline.xb.x = chile_item["xb_x"].asDouble();
    spline.xb.y = chile_item["xb_y"].asDouble();
    spline.xb.z = chile_item["xb_z"].asDouble();
    spline.xb.w = chile_item["xb_w"].asDouble();
    spline.xb.x = chile_item["yb_x"].asDouble();
    spline.xb.y = chile_item["yb_y"].asDouble();
    spline.xb.z = chile_item["yb_z"].asDouble();
    spline.xb.w = chile_item["yb_w"].asDouble();
  }

  // planning output
  planningData.planning_output.decision = static_cast<uint8_t>(planningOutput["decision"].asInt());
  planningData.planning_output.velocity = planningOutput["velocity"].asFloat();
  planningData.planning_output.pose.position.x = planningOutput["pose_position_x"].asDouble();
  planningData.planning_output.pose.position.y = planningOutput["pose_position_y"].asDouble();
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
  if (m_nShowView == LocalViewVehicle) {
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
    const std::vector<::debug_tool::ads_Point> &references,
    double vehicle_width,
    double tolerance)
{
  std::vector<SplineLib::Vec2f> points;

  for (int i = 0; i < 21; i++)
  {
    double left_road_width = references[i].left_road_width;
    double right_road_width = references[i].right_road_width;
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
            planningData.head_point.x, planningData.front_vehicle_width, 0.75
            );
      if (check) break;
    }
    if (check) continue;
    check = this->RoadBoundaryCheck(
          candidates[i], planningData.reference_points,
          planningData.front_vehicle_width,
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

void QPlanningWidget::changeShowView()
{
  switch (m_nShowView) {
    case LocalViewVehicle:
      m_pWdgShow[0]->hide();
      m_pWdgShow[1]->hide();
      m_pWdgFullView->show();
      m_nShowView = FullView;
      break;
    case FullView:
      m_pWdgFullView->hide();
      m_pWdgShow[0]->show();
      m_pWdgShow[1]->show();
      m_nShowView = LocalViewVehicle;
      break;
    default:
      break;
  }
}

