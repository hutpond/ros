#include "QNewPlanningWidget.h"

#include <fstream>
#include <QTimerEvent>
#include <QStatusBar>
#include "QPlanningParamWidget.h"
#include "GlobalDefine.h"
#include "QFullViewWidget.h"
#include "QReadDataManagerRos.h"
#include "QPlanningCostWidget.h"
#include "QDebugToolMainWnd.h"
#include "QNewPlanningShowWidget.h"
#include "QNewPlanningPlot.h"

QNewPlanningWidget::QNewPlanningWidget(QWidget *parent)
  : QBaseWidget(parent)
{
  this->setFont(G_TEXT_FONT);
  m_pWdgPlotting = new QNewPlanningPlot(this);

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
  m_pWdgShow[0] = new QNewPlanningShowWidget(this);
  m_pWdgShow[0]->setFunPosition(fun);
  m_pWdgShow[1] = Q_NULLPTR;

  m_nShowType = LivePlay;
  m_nShowView = LocalViewENU;

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

  QReadDataManagerRos::instance()->setNewPlanningWidget(this);
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
    const QString &)
{
  m_pWdgShow[0]->setPlanningData(data);
  m_pWdgPlotting->setPlanningData(data);
}

void QNewPlanningWidget::saveDataToJsonFile(
    const std::string &name,
    const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  Json::Value json_frame;
  // num obstacle
  json_frame["NUM_OBSTACLE"] = static_cast<double>(data.num_obstacle);

  // obstacle
  Json::Value json_obstacles;
  const int size_obstacle = data.obstacles.size();
  for (int i = 0; i < size_obstacle; ++i) {
    Json::Value json_points, json_points_enu, json_points_frenet;
    for (int j = 0; j < 4; ++j) {
      Json::Value json_point;
      json_point["X"] = data.obstacles[i].points_enu[j].X;
      json_point["Y"] = data.obstacles[i].points_enu[j].Y;
      json_point["Z"] = data.obstacles[i].points_enu[j].Z;
      json_point["v"] = data.obstacles[i].points_enu[j].v;
      json_point["a"] = data.obstacles[i].points_enu[j].a;
      json_point["kappa"] = data.obstacles[i].points_enu[j].kappa;
      json_point["dkappa"] = data.obstacles[i].points_enu[j].dkappa;
      json_point["theta"] = data.obstacles[i].points_enu[j].theta;
      json_point["s"] = data.obstacles[i].points_enu[j].s;
      json_points_enu.append(json_point);

      json_point.clear();
      json_point["S"] = data.obstacles[i].points_frenet[j].s;
      json_point["L"] = data.obstacles[i].points_frenet[j].l;
      json_points_frenet.append(json_point);
    }
    json_points["ENU"] = json_points_enu;
    json_points["FRENET"] = json_points_frenet;
    json_obstacles.append(json_points);
  }

  // current trajectories
  Json::Value json_current_trajectories;
  Json::Value json_current_trajectories_enu;
  const int size_current_trajectories_enu =
      data.trajectory_current.current_traj_points_enu.size();
  for (int i = 0; i < size_current_trajectories_enu; ++i) {
    Json::Value json_point;
    json_point["X"] = data.trajectory_current.current_traj_points_enu[i].X;
    json_point["Y"] = data.trajectory_current.current_traj_points_enu[i].Y;
    json_point["Z"] = data.trajectory_current.current_traj_points_enu[i].Z;
    json_point["v"] = data.trajectory_current.current_traj_points_enu[i].v;
    json_point["a"] = data.trajectory_current.current_traj_points_enu[i].a;
    json_point["kappa"] = data.trajectory_current.current_traj_points_enu[i].kappa;
    json_point["dkappa"] = data.trajectory_current.current_traj_points_enu[i].dkappa;
    json_point["theta"] = data.trajectory_current.current_traj_points_enu[i].theta;
    json_point["s"] = data.trajectory_current.current_traj_points_enu[i].s;
    json_current_trajectories_enu.append(json_point);
  }
  json_current_trajectories["ENU"] = json_current_trajectories_enu;

  Json::Value json_current_trajectories_frenet;
  const int size_current_trajectories_frenet =
      data.trajectory_current.current_traj_points_frenet.size();
  for (int i = 0; i < size_current_trajectories_frenet; ++i) {
    Json::Value json_point;
    json_point["s"] = data.trajectory_current.current_traj_points_frenet[i].s;
    json_point["l"] = data.trajectory_current.current_traj_points_frenet[i].l;
    json_current_trajectories_frenet.append(json_point);
  }
  json_current_trajectories["FRENET"] = json_current_trajectories_frenet;

  json_current_trajectories["COST"] = data.trajectory_current.cost;

  // last trajectories
  Json::Value json_last_trajectories;
  Json::Value json_last_trajectories_enu;
  const int size_last_trajectories_enu =
      data.trajectory_last.current_traj_points_enu.size();
  for (int i = 0; i < size_last_trajectories_enu; ++i) {
    Json::Value json_point;
    json_point["X"] = data.trajectory_last.current_traj_points_enu[i].X;
    json_point["Y"] = data.trajectory_last.current_traj_points_enu[i].Y;
    json_point["Z"] = data.trajectory_last.current_traj_points_enu[i].Z;
    json_point["v"] = data.trajectory_last.current_traj_points_enu[i].v;
    json_point["a"] = data.trajectory_last.current_traj_points_enu[i].a;
    json_point["kappa"] = data.trajectory_last.current_traj_points_enu[i].kappa;
    json_point["dkappa"] = data.trajectory_last.current_traj_points_enu[i].dkappa;
    json_point["theta"] = data.trajectory_last.current_traj_points_enu[i].theta;
    json_point["s"] = data.trajectory_last.current_traj_points_enu[i].s;
    json_last_trajectories_enu.append(json_point);
  }
  json_last_trajectories["ENU"] = json_last_trajectories_enu;

  Json::Value json_last_trajectories_frenet;
  const int size_last_trajectories_frenet =
      data.trajectory_last.current_traj_points_frenet.size();
  for (int i = 0; i < size_last_trajectories_frenet; ++i) {
    Json::Value json_point;
    json_point["s"] = data.trajectory_last.current_traj_points_frenet[i].s;
    json_point["l"] = data.trajectory_last.current_traj_points_frenet[i].l;
    json_current_trajectories_frenet.append(json_point);
  }
  json_last_trajectories["FRENET"] = json_last_trajectories_frenet;

  json_last_trajectories["COST"] = data.trajectory_last.cost;

  // reference enu
  Json::Value json_reference_enu;
  const int size_reference_enu = data.reference_line_enu.size();
  for (int i = 0; i < size_reference_enu; ++i) {
    Json::Value json_point;
    json_point["X"] = data.reference_line_enu[i].X;
    json_point["Y"] = data.reference_line_enu[i].Y;
    json_point["Z"] = data.reference_line_enu[i].Z;
    json_point["v"] = data.reference_line_enu[i].v;
    json_point["a"] = data.reference_line_enu[i].a;
    json_point["kappa"] = data.reference_line_enu[i].kappa;
    json_point["dkappa"] = data.reference_line_enu[i].dkappa;
    json_point["theta"] = data.reference_line_enu[i].theta;
    json_point["s"] = data.reference_line_enu[i].s;
    json_reference_enu.append(json_point);
  }

  // reference frenet
  Json::Value json_reference_frenet;
  const int size_reference_frenet = data.reference_line_frenet.size();
  for (int i = 0; i < size_reference_frenet; ++i) {
    Json::Value json_point;
    json_point["S"] = data.reference_line_frenet[i].s;
    json_point["L"] = data.reference_line_frenet[i].l;
    json_reference_frenet.append(json_point);
  }

  // ego state enu
  Json::Value json_ego_state_enu;
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point;
    json_point["X"] = data.ego_state_enu[i].X;
    json_point["Y"] = data.ego_state_enu[i].Y;
    json_point["Z"] = data.ego_state_enu[i].Z;
    json_point["v"] = data.ego_state_enu[i].v;
    json_point["a"] = data.ego_state_enu[i].a;
    json_point["kappa"] = data.ego_state_enu[i].kappa;
    json_point["dkappa"] = data.ego_state_enu[i].dkappa;
    json_point["theta"] = data.ego_state_enu[i].theta;
    json_point["s"] = data.ego_state_enu[i].s;
    json_ego_state_enu.append(json_point);
  }

  // ego state frenet
  Json::Value json_ego_state_frenet;
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point;
    json_point["S"] = data.ego_state_frenet[i].s;
    json_point["L"] = data.ego_state_frenet[i].l;
    json_ego_state_frenet.append(json_point);
  }

  // state machine
  Json::Value json_state_machine;
  json_state_machine["LAST_STATE"] = data.state_machine.last_state;
  json_state_machine["CURRENT_STATE"] = data.state_machine.current_state;
  json_state_machine["DECISION"] = data.state_machine.decision;
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
  json_frame["TRAJECTORIES_CURRENT"] = json_current_trajectories;
  json_frame["TRAJECTORIES_LAST"] = json_last_trajectories;
  json_frame["REFERENCE_ENU"] = json_reference_enu;
  json_frame["REFERENCE_FRENET"] = json_reference_frenet;
  json_frame["EGO_STATE_ENU"] = json_ego_state_enu;
  json_frame["EGO_STATE_FRENET"] = json_ego_state_frenet;
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
  Json::Value json_current_trajectories = root["TRAJECTORIES_CURRENT"];
  Json::Value json_last_trajectories = root["TRAJECTORIES_LAST"];
  Json::Value json_reference_enu = root["REFERENCE_ENU"];
  Json::Value json_reference_frenet = root["REFERENCE_FRENET"];
  Json::Value json_ego_state_enu = root["EGO_STATE_ENU"];
  Json::Value json_ego_state_frenet = root["EGO_STATE_FRENET"];
  Json::Value json_state_machine = root["STATE_MACHINE"];
  Json::Value json_parameters = root["PARAMETERS"];

  // num obstacle
  data.num_obstacle = static_cast<int64_t>(root["NUM_OBSTACLE"].asDouble());

  // obstacle
  const int size_obstacle = json_obstacles.size();
  for (int i = 0; i < size_obstacle; ++i) {
    Json::Value json_points = json_obstacles[i];
    Json::Value json_points_enu = json_points["ENU"];
    Json::Value json_points_frenet = json_points["FRENET"];

    debug_ads_msgs::ads_msgs_planning_debug_obstacle obstacles;
    for (int j = 0; j < 4; ++j) {
      Json::Value json_point = json_points_enu[j];
      obstacles.points_enu[j].X = json_point["X"].asDouble();
      obstacles.points_enu[j].Y = json_point["Y"].asDouble();
      obstacles.points_enu[j].Z = json_point["Z"].asDouble();
      obstacles.points_enu[j].v = json_point["v"].asDouble();
      obstacles.points_enu[j].a = json_point["a"].asDouble();
      obstacles.points_enu[j].kappa = json_point["kappa"].asDouble();
      obstacles.points_enu[j].dkappa = json_point["dkappa"].asDouble();
      obstacles.points_enu[j].theta = json_point["theta"].asDouble();
      obstacles.points_enu[j].s = json_point["s"].asDouble();

      Json::Value json_point_f = json_points_frenet[j];
      obstacles.points_frenet[j].s = json_point_f["S"].asDouble();
      obstacles.points_frenet[j].l = json_point_f["L"].asDouble();
    }
    data.obstacles.push_back(obstacles);
  }

  // current trajectories
  Json::Value json_current_trajectories_enu = json_current_trajectories["ENU"];
  const int size_current_trajectories_enu = json_current_trajectories_enu.size();
  for (int i = 0; i < size_current_trajectories_enu; ++i) {
    Json::Value json_point = json_current_trajectories_enu[i];

    debug_ads_msgs::ads_msgs_planning_debug_pointENU point;
    point.X = json_point["X"].asDouble();
    point.Y = json_point["Y"].asDouble();
    point.Z = json_point["Z"].asDouble();
    point.v = json_point["v"].asDouble();
    point.a = json_point["a"].asDouble();
    point.kappa = json_point["kappa"].asDouble();
    point.dkappa = json_point["dkappa"].asDouble();
    point.theta = json_point["theta"].asDouble();
    point.s = json_point["s"].asDouble();

    data.trajectory_current.current_traj_points_enu.push_back(point);
  }
  Json::Value json_current_trajectories_frenet = json_current_trajectories["FRENET"];
  const int size_current_trajectories_frenet = json_current_trajectories_frenet.size();
  for (int i = 0; i < size_current_trajectories_frenet; ++i) {
    Json::Value json_point = json_current_trajectories_frenet[i];

    debug_ads_msgs::ads_msgs_planning_debug_pointFRENET point;
    point.s = json_point["s"].asDouble();
    point.l = json_point["l"].asDouble();
    data.trajectory_current.current_traj_points_frenet.push_back(point);
  }
  data.trajectory_current.cost = json_current_trajectories["COST"].asDouble();

  // last trajectories
  Json::Value json_last_trajectories_enu = json_last_trajectories["ENU"];
  const int size_last_trajectories_enu = json_last_trajectories_enu.size();
  for (int i = 0; i < size_last_trajectories_enu; ++i) {
    Json::Value json_point = json_last_trajectories_enu[i];

    debug_ads_msgs::ads_msgs_planning_debug_pointENU point;
    point.X = json_point["X"].asDouble();
    point.Y = json_point["Y"].asDouble();
    point.Z = json_point["Z"].asDouble();
    point.v = json_point["v"].asDouble();
    point.a = json_point["a"].asDouble();
    point.kappa = json_point["kappa"].asDouble();
    point.dkappa = json_point["dkappa"].asDouble();
    point.theta = json_point["theta"].asDouble();
    point.s = json_point["s"].asDouble();

    data.trajectory_last.current_traj_points_enu.push_back(point);
  }
  Json::Value json_last_trajectories_frenet = json_last_trajectories["FRENET"];
  const int size_last_trajectories_frenet = json_last_trajectories_frenet.size();
  for (int i = 0; i < size_last_trajectories_frenet; ++i) {
    Json::Value json_point = json_last_trajectories_frenet[i];

    debug_ads_msgs::ads_msgs_planning_debug_pointFRENET point;
    point.s = json_point["s"].asDouble();
    point.l = json_point["l"].asDouble();

    data.trajectory_last.current_traj_points_frenet.push_back(point);
  }
  data.trajectory_last.cost = json_last_trajectories["COST"].asDouble();

  // reference enu
  const int size_reference_enu = json_reference_enu.size();
  for (int i = 0; i < size_reference_enu; ++i) {
    Json::Value json_point = json_reference_enu[i];

    debug_ads_msgs::ads_msgs_planning_debug_pointENU point;
    point.X = json_point["X"].asDouble();
    point.Y = json_point["Y"].asDouble();
    point.Z = json_point["Z"].asDouble();
    point.v = json_point["v"].asDouble();
    point.a = json_point["a"].asDouble();
    point.kappa = json_point["kappa"].asDouble();
    point.dkappa = json_point["dkappa"].asDouble();
    point.theta = json_point["theta"].asDouble();
    point.s = json_point["s"].asDouble();

    data.reference_line_enu.push_back(point);
  }

  // reference frenet
  const int size_reference_frenet = json_reference_frenet.size();
  for (int i = 0; i < size_reference_frenet; ++i) {
    Json::Value json_point = json_reference_frenet[i];

    debug_ads_msgs::ads_msgs_planning_debug_pointFRENET point;
    point.s = json_point["S"].asDouble();
    point.l = json_point["L"].asDouble();

    data.reference_line_frenet.push_back(point);
  }

  // ego state enu
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point = json_ego_state_enu[i];
    data.ego_state_enu[i].X = json_point["X"].asDouble();
    data.ego_state_enu[i].Y = json_point["Y"].asDouble();
    data.ego_state_enu[i].Z = json_point["Z"].asDouble();
    data.ego_state_enu[i].v = json_point["v"].asDouble();
    data.ego_state_enu[i].a = json_point["a"].asDouble();
    data.ego_state_enu[i].kappa = json_point["kappa"].asDouble();
    data.ego_state_enu[i].dkappa = json_point["dkappa"].asDouble();
    data.ego_state_enu[i].theta = json_point["theta"].asDouble();
    data.ego_state_enu[i].s = json_point["s"].asDouble();
  }

  // ego state frenet
  for (int i = 0; i < 4; ++i) {
    Json::Value json_point = json_ego_state_frenet[i];
    data.ego_state_frenet[i].s = json_point["S"].asDouble();
    data.ego_state_frenet[i].l = json_point["L"].asDouble();
  }

  // state machine
  data.state_machine.last_state = json_state_machine["LAST_STATE"].asInt();
  data.state_machine.current_state = json_state_machine["CURRENT_STATE"].asInt();
  data.state_machine.decision = json_state_machine["DECISION"].asInt();
  data.state_machine.stop_reason = json_state_machine["DECISION"].asString();

  // parameters
  data.parameters.cost_1 = json_parameters["COST_1"].asDouble();
  data.parameters.cost_2 = json_parameters["COST_2"].asDouble();
  data.parameters.cost_3 = json_parameters["COST_3"].asDouble();
  data.parameters.cost_4 = json_parameters["COST_4"].asDouble();
  data.parameters.cost_5 = json_parameters["COST_5"].asDouble();
  data.parameters.cost_6 = json_parameters["COST_6"].asDouble();
}

void QNewPlanningWidget::changeShowView()
{
  switch (m_nShowView) {
    case LocalViewENU:
      m_pWdgShow[0]->setShowCoord(QBaseShowWidget::FrenetCoord);
      m_nShowView = LocalViewFrenet;
      break;
    case LocalViewFrenet:
      m_pWdgShow[0]->hide();
      m_pWdgPlotting->show();
      m_nShowView = PlottingView;
      break;
    case PlottingView:
      m_pWdgPlotting->hide();
      m_pWdgShow[0]->show();
      m_pWdgShow[0]->setShowCoord(QBaseShowWidget::EnuCoord);
      m_nShowView = LocalViewENU;
      break;
    default:
      break;
  }
}
