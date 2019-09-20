#include <fstream>
#include <QTimerEvent>
#include <QStatusBar>
#include "QNewPlanningWidget.h"
#include "QPlanningParamWidget.h"
#include "GlobalDefine.h"
#include "QFullViewWidget.h"
#include "QReadDataManagerRos.h"
#include "QPlanningCostWidget.h"
#include "QDebugToolMainWnd.h"
#include "QNewPlanningShowWidget.h"

QNewPlanningWidget::QNewPlanningWidget(QWidget *parent)
  : QBaseWidget(parent)
{
  this->setFont(G_TEXT_FONT);
  m_pWdgParam = new QPlanningParamWidget(this);
  m_pWdgParam->setShowType(LivePlay);
  connect(m_pWdgParam, &QPlanningParamWidget::replayState,
          this, &QNewPlanningWidget::onReplayState);
  connect(m_pWdgParam, &QPlanningParamWidget::replayFrameOffset,
          this, &QNewPlanningWidget::onSetFrameIndexReplay);
  connect(m_pWdgParam, &QPlanningParamWidget::costValueChanged,
          this, &QNewPlanningWidget::onCostValueChanged);

  boost::function<void(float, float, float, float)> fun = boost::bind(&QPlanningParamWidget::showMousePosition, m_pWdgParam,
                       _1, _2, _3, _4);
  for (int i = 0; i < 2; ++i) {
    m_pWdgShow[i] = new QNewPlanningShowWidget(this);
    m_pWdgShow[i]->setFunPosition(fun);
//    connect(m_pWdgShow[i], &QBaseShowWidget::saveDataToFile,
//        this, &QPlanningWidget::onSaveDataToFile);
  }
  m_pWdgShow[1]->setCostType(QPlanningShowWidget::NEW_COST);
  m_nShowType = LivePlay;
  m_nShowView = LocalView;

  m_pWdgFullView = new QFullViewWidget(this);
  m_pWdgFullView->setFunPosition(fun);
  m_pWdgFullView->hide();

  namespace fs = boost::filesystem;
  m_fsPath = getenv("HOME");
  m_fsPath /= "NewPlanningData";

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

  connect(QReadDataManagerRos::instance(), &QReadDataManagerRos::planningDataNew,
          this, &QNewPlanningWidget::onParsePlanningData);

  QReadDataManagerRos::instance()->start_subscribe();

  m_nReplaySpeedIndex = 1;
//  memset(m_dCostValue, 0, sizeof(double) * QPlanningCostWidget::Count);
}

void QNewPlanningWidget::timerEvent(QTimerEvent *e)
{
  int id = e->timerId();
  if (id == m_nTimerId) {
    if (m_bFlagPauseReplay) {
      return;
    }
    if (m_itFile == m_listPlanningFiles.end()) {
      return;
    }
    debug_ads_msgs::ads_msgs_planning_debug_frame data;
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

void QNewPlanningWidget::onParsePlanningData(
    const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  if (!fs::exists(m_fsPath)) {
    fs::create_directories(m_fsPath);
  }

  fs::path path = m_fsPath;
  std::string strFileName = QBaseWidget::dataFileName();
  path /= strFileName;
  strFileName = path.string();

  this->saveDataToJsonFile(strFileName, data);
  if (m_nShowType == LivePlay) {
    this->setPlanningData(data, "");
  }
}

void QNewPlanningWidget::setPlanningData(
    const debug_ads_msgs::ads_msgs_planning_debug_frame &data,
    const QString &name)
{
  if (m_nShowView == LocalView) {
    m_pWdgShow[0]->setPlanningData(data);
    m_pWdgShow[1]->setPlanningData(data);
    //m_pWdgFullView->setPlanningData(data, name, false);
  }
  else {
    //m_pWdgFullView->setPlanningData(data, name, true);
  }
  //m_pWdgParam->setPlanningData(data);
}

void QNewPlanningWidget::saveDataToJsonFile(
    const std::string &name,
    const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  Json::Value json_frame;
  // obstacle
  Json::Value json_obstacles, json_obstacles_data;
  json_obstacles["NUM_OBSTACLE"] = static_cast<double>(data.num_obstacle);
  const int size_obstacle = data.obstacles.size();
  for (int i = 0; i < size_obstacle; ++i) {
    Json::Value json_item;
    for (int j = 0; j < 4; ++j) {
      Json::Value json_point;
      json_point["X"] = data.obstacles[i].points_enu[j].X;
      json_point["Y"] = data.obstacles[i].points_enu[j].Y;
      json_point["Z"] = data.obstacles[i].points_enu[j].Z;
      json_point["S"] = data.obstacles[i].points_frenet[j].s;
      json_point["L"] = data.obstacles[i].points_frenet[j].l;

      json_item.append(json_point);
    }
    json_obstacles_data.append(json_item);
  }
  json_obstacles["DATA"] = json_obstacles_data;

  // trajectories
  Json::Value json_trajectories;
  const int size_trajectories = data.trajectories.size();
  for (int i = 0; i < size_trajectories; ++i) {
    Json::Value json_item, json_current, json_last, json_current_frenet;
    const int size_current = data.trajectories[i].current_traj_points_enu.size();
    for (int j = 0; j < size_current; ++j) {
      Json::Value json_point;
      json_point["X"] = data.trajectories[i].current_traj_points_enu[j].X;
      json_point["Y"] = data.trajectories[i].current_traj_points_enu[j].Y;
      json_point["Z"] = data.trajectories[i].current_traj_points_enu[j].Z;
      json_current.append(json_point);
    }
    json_item["CURRENT"] = json_current;

    const int size_last = data.trajectories[i].last_traj_points_enu.size();
    for (int j = 0; j < size_last; ++j) {
      Json::Value json_point;
      json_point["X"] = data.trajectories[i].last_traj_points_enu[j].X;
      json_point["Y"] = data.trajectories[i].last_traj_points_enu[j].Y;
      json_point["Z"] = data.trajectories[i].last_traj_points_enu[j].Z;
      json_last.append(json_point);
    }
    json_item["LAST"] = json_last;

    const int size_current_frenet = data.trajectories[i].current_traj_points_frenet.size();
    for (int j = 0; j < size_current_frenet; ++j) {
      Json::Value json_point;
      json_point["S"] = data.trajectories[i].current_traj_points_frenet[j].s;
      json_point["L"] = data.trajectories[i].current_traj_points_frenet[j].l;
      json_current_frenet.append(json_point);
    }
    json_item["CURRENT_FRENET"] = json_current_frenet;

    json_trajectories.append(json_item);
  }

  // reference
  Json::Value json_reference, json_reference_enu, json_reference_frenet;
  const int size_reference_enu = data.reference_line_enu.size();
  for (int i = 0; i < size_reference_enu; ++i) {
    Json::Value json_point;
    json_point["X"] = data.reference_line_enu[i].X;
    json_point["Y"] = data.reference_line_enu[i].Y;
    json_point["Z"] = data.reference_line_enu[i].Z;
    json_reference_enu.append(json_point);
  }
  json_reference["REFERENCE_ENU"] = json_reference_enu;

  const int size_reference_frenet = data.reference_line_frenet.size();
  for (int i = 0; i < size_reference_frenet; ++i) {
    Json::Value json_point;
    json_point["S"] = data.reference_line_frenet[i].s;
    json_point["L"] = data.reference_line_frenet[i].l;
    json_reference_frenet.append(json_point);
  }
  json_reference["REFERENCE_FRENET"] = json_reference_frenet;

  // ego state
  Json::Value json_ego_state, json_ego_state_enu, json_ego_state_frenet;
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point;
    json_point["X"] = data.ego_state_enu[i].X;
    json_point["Y"] = data.ego_state_enu[i].Y;
    json_point["Z"] = data.ego_state_enu[i].Z;
    json_ego_state_enu.append(json_point);
  }
  json_ego_state["EGO_STATE_ENU"] = json_ego_state_enu;
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point;
    json_point["S"] = data.ego_state_frenet[i].s;
    json_point["L"] = data.ego_state_frenet[i].l;
    json_ego_state_frenet.append(json_point);
  }
  json_ego_state["EGO_STATE_FRENET"] = json_ego_state_frenet;

  // state machine
  Json::Value json_state_machine;
  json_state_machine["LAST_STATE"] = static_cast<int>(data.state_machine.last_state);
  json_state_machine["CURRENT_STATE"] = static_cast<int>(data.state_machine.current_state);
  json_state_machine["DECISION"] = static_cast<int>(data.state_machine.decision);
  json_state_machine["STOP_REASON"] = data.state_machine.stop_reason;

  // parameters
  Json::Value json_parameters;
  json_parameters["COST_1"] = data.parameters.cost_1;
  json_parameters["COST_2"] = data.parameters.cost_2;
  json_parameters["COST_3"] = data.parameters.cost_3;
  json_parameters["COST_4"] = data.parameters.cost_4;
  json_parameters["COST_5"] = data.parameters.cost_5;
  json_parameters["COST_6"] = data.parameters.cost_6;

  // json frame
  json_frame["OBSTACLES"] = json_obstacles;
  json_frame["TRAJECTORIES"] = json_trajectories;
  json_frame["REFERENCE"] = json_reference;
  json_frame["EGO_STATE"] = json_ego_state;
  json_frame["STATE_MACHINE"] = json_state_machine;
  json_frame["PARAMETERS"] = json_parameters;

  // write to file
  std::ofstream out(name.c_str());
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(json_frame, &out);
  out.close();
}

bool QNewPlanningWidget::readFromJsonFile(
    const std::string &name,
    debug_ads_msgs::ads_msgs_planning_debug_frame &data)
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
  this->parseDataFromJson(root, data);

  return true;
}

void QNewPlanningWidget::parseDataFromJson(
    const Json::Value &root,
    debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  Json::Value json_obstacles = root["OBSTACLES"];
  Json::Value json_trajectories = root["TRAJECTORIES"];
  Json::Value json_reference = root["REFERENCE"];
  Json::Value json_ego_state = root["EGO_STATE"];
  Json::Value json_state_machine = root["STATE_MACHINE"];
  Json::Value json_parameters = root["PARAMETERS"];

  // obstacle
  data.num_obstacle = static_cast<int64_t>(json_obstacles["NUM_OBSTACLE"].asDouble());
  const int size_obstacle = json_obstacles["DATA"].size();
  for (int i = 0; i < size_obstacle; ++i) {
    Json::Value json_item = json_obstacles["DATA"][i];
    for (int j = 0; j < 4; ++j) {
      Json::Value json_point = json_item[j];
      data.obstacles[i].points_enu[j].X = json_point["X"].asDouble();
      data.obstacles[i].points_enu[j].Y = json_point["Y"].asDouble();
      data.obstacles[i].points_enu[j].Z = json_point["Z"].asDouble();
      data.obstacles[i].points_frenet[j].s = json_point["S"].asDouble();
      data.obstacles[i].points_frenet[j].l = json_point["L"].asDouble();
    }
  }

  // trajectories
  const int size_trajectories = json_trajectories.size();
  for (int i = 0; i < size_trajectories; ++i) {
    Json::Value json_current = json_trajectories[i]["CURRENT"];
    const int size_current = json_current.size();
    for (int j = 0; j < size_current; ++j) {
      Json::Value json_point = json_current[j];
      data.trajectories[i].current_traj_points_enu[j].X = json_point["X"].asDouble();
      data.trajectories[i].current_traj_points_enu[j].Y = json_point["Y"].asDouble();
      data.trajectories[i].current_traj_points_enu[j].Z = json_point["Z"].asDouble();
    }

    Json::Value json_last = json_trajectories[i]["LAST"];
    const int size_last = json_last.size();
    for (int j = 0; j < size_last; ++j) {
      Json::Value json_point = json_last[j];
      data.trajectories[i].last_traj_points_enu[j].X = json_point["X"].asDouble();
      data.trajectories[i].last_traj_points_enu[j].Y = json_point["Y"].asDouble();
      data.trajectories[i].last_traj_points_enu[j].Z = json_point["Z"].asDouble();
    }

    Json::Value json_current_frenet = json_trajectories[i]["CURRENT_FRENET"];
    const int size_current_frenet = json_current_frenet.size();
    for (int j = 0; j < size_current_frenet; ++j) {
      Json::Value json_point = json_current_frenet[j];
      data.trajectories[i].current_traj_points_frenet[j].s = json_point["S"].asDouble();
      data.trajectories[i].current_traj_points_frenet[j].l = json_point["L"].asDouble();
    }
  }

  // reference
  Json::Value json_reference_enu = json_reference["REFERENCE_ENU"];
  const int size_reference_enu = json_reference_enu.size();
  for (int i = 0; i < size_reference_enu; ++i) {
    Json::Value json_point = json_reference_enu[i];
    data.reference_line_enu[i].X = json_point["X"].asDouble();
    data.reference_line_enu[i].Y = json_point["Y"].asDouble();
    data.reference_line_enu[i].Z = json_point["Z"].asDouble();
  }

  Json::Value json_reference_frenet = json_reference["REFERENCE_FRENET"];
  const int size_reference_frenet = json_reference_frenet.size();
  for (int i = 0; i < size_reference_frenet; ++i) {
    Json::Value json_point = json_reference_frenet[i];
    data.reference_line_frenet[i].s = json_point["S"].asDouble();
    data.reference_line_frenet[i].l = json_point["L"].asDouble();
  }

  // ego state
  Json::Value json_ego_state_enu = json_ego_state["EGO_STATE_ENU"];
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point = json_ego_state_enu[i];
    data.ego_state_enu[i].X = json_point["X"].asDouble();
    data.ego_state_enu[i].Y = json_point["Y"].asDouble();
    data.ego_state_enu[i].Z = json_point["Z"].asDouble();
  }
  Json::Value json_ego_state_frenet = json_ego_state["EGO_STATE_FRENET"];
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point = json_ego_state_frenet[i];
    data.ego_state_frenet[i].s = json_point["S"].asDouble();
    data.ego_state_frenet[i].l = json_point["L"].asDouble();
  }

  // state machine
  data.state_machine.last_state = static_cast<int8_t>(json_state_machine["LAST_STATE"].asInt());
  data.state_machine.current_state = static_cast<int8_t>(json_state_machine["CURRENT_STATE"].asInt());
  data.state_machine.decision = static_cast<int8_t>(json_state_machine["DECISION"].asInt());

  // parameters
  data.parameters.cost_1 = json_parameters["COST_1"].asDouble();
  data.parameters.cost_2 = json_parameters["COST_2"].asDouble();
  data.parameters.cost_3 = json_parameters["COST_3"].asDouble();
  data.parameters.cost_4 = json_parameters["COST_4"].asDouble();
  data.parameters.cost_5 = json_parameters["COST_5"].asDouble();
  data.parameters.cost_6 = json_parameters["COST_6"].asDouble();
}
