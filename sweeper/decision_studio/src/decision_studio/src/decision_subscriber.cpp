#include "decision_subscriber.h"

#include <fstream>
#include <QTimer>
#include <jsoncpp/json/json.h>

#include "qcentralwidget.h"
#include "qreplaywidget.h"

constexpr int REPLAY_INTERVAL[] = {500, 250, 50, 10};

DecisionSubscriber::DecisionSubscriber(QCentralWidget &widget, QObject *parent)
  : m_rWdgCentral(widget)
  , QObject(parent)
  , m_bFlagLiveing(true)
  , m_bFlagSaveFile(false)
  , m_nReplayIntervalIndex(sizeof(REPLAY_INTERVAL) - 1)
  , m_bFlagReplayPause(false)
{
  m_subscriber = m_nodeHandle.subscribe(
        "decision_debug_data", 10,
        &DecisionSubscriber::onSubscribeData, this);

  m_pTimerSubscribe = new QTimer(this);
  connect(m_pTimerSubscribe, &QTimer::timeout, this, &DecisionSubscriber::onSpin);

  m_pTimerReplay = new QTimer(this);
  connect(m_pTimerReplay, &QTimer::timeout, this, &DecisionSubscriber::onReplay);

  this->setLiving(true);
}

DecisionSubscriber::~DecisionSubscriber()
{
  m_pTimerSubscribe->stop();
  m_pTimerReplay->stop();
}

void DecisionSubscriber::setSaveFileFlag(bool flag)
{
  m_bFlagSaveFile = flag;
}

void DecisionSubscriber::openReplayDir(const QString &path)
{
  m_bFlagReplayPause = false;
  this->fileList(path.toStdString(), m_listPlanningFiles);
  m_itFile = m_listPlanningFiles.begin();

  this->setLiving(false);
}

void DecisionSubscriber::onSpin()
{
  ros::spinOnce();
}

void DecisionSubscriber::onReplay()
{
  if (!m_bFlagLiveing && !m_bFlagReplayPause && m_itFile != m_listPlanningFiles.end()) {
    this->readDataFromFile(*m_itFile);
    ++ m_itFile;

    emit replayNextIndex();
  }
}

void DecisionSubscriber::onSubscribeData(const decision_studio::ads_DecisionData4Debug &data)
{
  if (!boost::filesystem::exists(m_fsPath)) {
    boost::filesystem::create_directories(m_fsPath);
  }
  if (m_bFlagLiveing) {
    m_rWdgCentral.setData(data);
  }
  if (m_bFlagSaveFile) {
    this->saveDataToFile(data);
  }
}

void DecisionSubscriber::saveDataToFile(const decision_studio::ads_DecisionData4Debug &data)
{
  Json::Value root;

  // header
  Json::Value header;
  header["second"] = data.header.stamp.sec;
  header["nano_second"] = data.header.stamp.nsec;

  // vehicle
  Json::Value carStatus, frontAxleCenter, rearAxleCenter, headPoint, rearPoint, hingePoint;

  frontAxleCenter["id"] = data.front_axle_center.id;
  frontAxleCenter["x"] = data.front_axle_center.x;
  frontAxleCenter["y"] = data.front_axle_center.y;
  frontAxleCenter["enu_x"] = data.front_axle_center.enu_x;
  frontAxleCenter["enu_y"] = data.front_axle_center.enu_y;
  frontAxleCenter["s"] = data.front_axle_center.s;
  frontAxleCenter["l"] = data.front_axle_center.l;
  frontAxleCenter["left_road_width"] = data.front_axle_center.left_road_width;
  frontAxleCenter["right_road_width"] = data.front_axle_center.right_road_width;

  rearAxleCenter["id"] = data.rear_axle_center.id;
  rearAxleCenter["x"] = data.rear_axle_center.x;
  rearAxleCenter["y"] = data.rear_axle_center.y;
  rearAxleCenter["enu_x"] = data.rear_axle_center.enu_x;
  rearAxleCenter["enu_y"] = data.rear_axle_center.enu_y;
  rearAxleCenter["s"] = data.rear_axle_center.s;
  rearAxleCenter["l"] = data.rear_axle_center.l;
  rearAxleCenter["left_road_width"] = data.rear_axle_center.left_road_width;
  rearAxleCenter["right_road_width"] = data.rear_axle_center.right_road_width;

  headPoint["id"] = data.head_point.id;
  headPoint["x"] = data.head_point.x;
  headPoint["y"] = data.head_point.y;
  headPoint["enu_x"] = data.head_point.enu_x;
  headPoint["enu_y"] = data.head_point.enu_y;
  headPoint["s"] = data.head_point.s;
  headPoint["l"] = data.head_point.l;
  headPoint["left_road_width"] = data.head_point.left_road_width;
  headPoint["right_road_width"] = data.head_point.right_road_width;

  rearPoint["id"] = data.rear_point.id;
  rearPoint["x"] = data.rear_point.x;
  rearPoint["y"] = data.rear_point.y;
  rearPoint["enu_x"] = data.rear_point.enu_x;
  rearPoint["enu_y"] = data.rear_point.enu_y;
  rearPoint["s"] = data.rear_point.s;
  rearPoint["l"] = data.rear_point.l;
  rearPoint["left_road_width"] = data.rear_point.left_road_width;
  rearPoint["right_road_width"] = data.rear_point.right_road_width;

  hingePoint["id"] = data.hinge_point.id;
  hingePoint["x"] = data.hinge_point.x;
  hingePoint["y"] = data.hinge_point.y;
  hingePoint["enu_x"] = data.hinge_point.enu_x;
  hingePoint["enu_y"] = data.hinge_point.enu_y;
  hingePoint["s"] = data.hinge_point.s;
  hingePoint["l"] = data.hinge_point.l;
  hingePoint["left_road_width"] = data.hinge_point.left_road_width;
  hingePoint["right_road_width"] = data.hinge_point.right_road_width;

  carStatus["front_axle_center"] = frontAxleCenter;
  carStatus["rear_axle_center"] = rearAxleCenter;
  carStatus["head_point"] = headPoint;
  carStatus["rear_point"] = rearPoint;
  carStatus["hinge_point"] = hingePoint;
  carStatus["front_vehicle_length"] = data.front_vehicle_length;
  carStatus["front_vehicle_width"] = data.front_vehicle_width;
  carStatus["rear_vehicle_length"] = data.rear_vehicle_length;
  carStatus["rear_vehicle_width"] = data.rear_vehicle_width;
  carStatus["steering_angle"] = data.steering_angle;
  carStatus["vehicle_enu_x"] = data.vehicle_enu_x;
  carStatus["vehicle_enu_y"] = data.vehicle_enu_y;
  carStatus["vehicle_enu_z"] = data.vehicle_enu_z;
  carStatus["vehicle_pitch"] = data.vehicle_pitch;
  carStatus["vehicle_roll"] = data.vehicle_roll;
  carStatus["vehicle_yaw"] = data.vehicle_yaw;

  // tarck targets
  Json::Value fusionResults;
  for (const auto &target : data.fusion_results) {
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
  root["fusion_results"] = fusionResults;


  // float64 safe_stop_dis
  root["safe_stop_dis"] = data.safe_stop_dis;

  //  float64 wait_dis
  root["wait_dis"] = data.wait_dis;

  // referene points
  Json::Value referencePoints;
  for (const auto &point : data.reference_points) {
    Json::Value item;
    item["id"] = point.id;
    item["x"] = point.x;
    item["y"] = point.y;
//    item["enu_x"] = point.enu_x;
//    item["enu_y"] = point.enu_y;
    item["s"] = point.s;
    item["l"] = point.l;
    item["left_road_width"] = point.left_road_width;
    item["right_road_width"] = point.right_road_width;
    referencePoints.append(item);
  }
  root["reference_points"] = referencePoints;

  // reference splines
  Json::Value referenceSplines;
  for (const auto &spline : data.reference_splines) {
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
  root["reference_splines"] = referenceSplines;

  // uint8 decision
  root["decision"] = static_cast<int32_t>(data.decision);

  // string debug_info
  root["debug_info"] = data.debug_info;

  // save file
  std::string fileName = m_fsPath.string();
  if (fileName[fileName.size() - 1] != '/') {
    fileName.push_back('/');
  }
  fileName += this->timeToString();
  fileName += ".txt";

  std::ofstream out(fileName.c_str());
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(root, &out);
  out.close();
}

void DecisionSubscriber::readDataFromFile(const std::string &name)
{
  // read file
  const std::string fileName = name.substr(1, name.length() - 2);
  //m_strJsonFile = fileName;

  FILE *pf = fopen(fileName.c_str(), "r");
  if (pf == NULL) {
    return;
  }
  fseek(pf , 0 , SEEK_END);
  long size = ftell(pf);
  rewind(pf);
  char *buffer = (char*)malloc(size + 1);
  memset(buffer, 0, size + 1);
  if (buffer == NULL) {
    return;
  }
  fread(buffer,1, size, pf);
  fclose(pf);

  // convert to json
  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(buffer, root)) {
    delete buffer;
    return;
  }
  delete buffer;
  emit replayFileName(QString::fromStdString(name));

  // parse json
  decision_studio::ads_DecisionData4Debug data;

  // header
  Json::Value header = root["header"];
  data.header.stamp.sec = header["second"].asUInt();
  data.header.stamp.nsec = header["nano_second"].asUInt();

  // vehicle
  Json::Value carStatus = root["car_status"];

  Json::Value frontAxleCenter = carStatus["front_axle_center"];
  Json::Value rearAxleCenter = carStatus["rear_axle_center"];
  Json::Value headPoint = carStatus["head_point"];
  Json::Value rearPoint = carStatus["rear_point"];
  Json::Value hingePoint = carStatus["hinge_point"];
  data.front_vehicle_length = carStatus["front_vehicle_length"].asDouble();
  data.front_vehicle_width = carStatus["front_vehicle_width"].asDouble();
  data.rear_vehicle_length = carStatus["rear_vehicle_length"].asDouble();
  data.rear_vehicle_width = carStatus["rear_vehicle_width"].asDouble();
  data.steering_angle = carStatus["steering_angle"].asDouble();

  data.vehicle_enu_x = carStatus["vehicle_enu_x"].asDouble();
  data.vehicle_enu_y = carStatus["vehicle_enu_y"].asDouble();
  data.vehicle_enu_z = carStatus["vehicle_enu_z"].asDouble();
  data.vehicle_pitch = carStatus["vehicle_pitch"].asDouble();
  data.vehicle_roll = carStatus["vehicle_roll"].asDouble();
  data.vehicle_yaw = carStatus["vehicle_yaw"].asDouble();

  data.front_axle_center.id = frontAxleCenter["id"].asInt();
  data.front_axle_center.x = frontAxleCenter["x"].asDouble();
  data.front_axle_center.y = frontAxleCenter["y"].asDouble();
  data.front_axle_center.enu_x = frontAxleCenter["enu_x"].asDouble();
  data.front_axle_center.enu_y = frontAxleCenter["enu_y"].asDouble();
  data.front_axle_center.s = frontAxleCenter["s"].asDouble();
  data.front_axle_center.l = frontAxleCenter["l"].asDouble();
  data.front_axle_center.left_road_width = frontAxleCenter["left_road_width"].asDouble();
  data.front_axle_center.right_road_width = frontAxleCenter["right_road_width"].asDouble();

  data.rear_axle_center.id = rearAxleCenter["id"].asInt();
  data.rear_axle_center.x = rearAxleCenter["x"].asDouble();
  data.rear_axle_center.y = rearAxleCenter["y"].asDouble();
  data.rear_axle_center.enu_x = rearAxleCenter["enu_x"].asDouble();
  data.rear_axle_center.enu_y = rearAxleCenter["enu_y"].asDouble();
  data.rear_axle_center.s = rearAxleCenter["s"].asDouble();
  data.rear_axle_center.l = rearAxleCenter["l"].asDouble();
  data.rear_axle_center.left_road_width = rearAxleCenter["left_road_width"].asDouble();
  data.rear_axle_center.right_road_width = rearAxleCenter["right_road_width"].asDouble();

  data.head_point.id = headPoint["id"].asInt();
  data.head_point.x = headPoint["x"].asDouble();
  data.head_point.y = headPoint["y"].asDouble();
  data.head_point.enu_x = headPoint["enu_x"].asDouble();
  data.head_point.enu_y = headPoint["enu_y"].asDouble();
  data.head_point.s = headPoint["s"].asDouble();
  data.head_point.l = headPoint["l"].asDouble();
  data.head_point.left_road_width = headPoint["left_road_width"].asDouble();
  data.head_point.right_road_width = headPoint["right_road_width"].asDouble();

  data.rear_point.id = rearPoint["id"].asInt();
  data.rear_point.x = rearPoint["x"].asDouble();
  data.rear_point.y = rearPoint["y"].asDouble();
  data.rear_point.enu_x = rearPoint["enu_x"].asDouble();
  data.rear_point.enu_y = rearPoint["enu_y"].asDouble();
  data.rear_point.s = rearPoint["s"].asDouble();
  data.rear_point.l = rearPoint["l"].asDouble();
  data.rear_point.left_road_width = rearPoint["left_road_width"].asDouble();
  data.rear_point.right_road_width = rearPoint["right_road_width"].asDouble();

  data.hinge_point.id = hingePoint["id"].asInt();
  data.hinge_point.x = hingePoint["x"].asDouble();
  data.hinge_point.y = hingePoint["y"].asDouble();
  data.hinge_point.enu_x = hingePoint["enu_x"].asDouble();
  data.hinge_point.enu_y = hingePoint["enu_y"].asDouble();
  data.hinge_point.s = hingePoint["s"].asDouble();
  data.hinge_point.l = hingePoint["l"].asDouble();
  data.hinge_point.left_road_width = hingePoint["left_road_width"].asDouble();
  data.hinge_point.right_road_width = hingePoint["right_road_width"].asDouble();

  // tarck targets
  Json::Value fusionResults = root["fusion_results"];
  const int size_fusion_results = fusionResults.size();
  data.fusion_results.resize(size_fusion_results);
  for (int i = 0; i < size_fusion_results; ++i) {
    auto &target = data.fusion_results[i];
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

  // float64 safe_stop_dis
  data.safe_stop_dis = root["safe_stop_dis"].asDouble();

  //  float64 wait_dis
  data.wait_dis = root["wait_dis"].asDouble();

  // referene points
  Json::Value referencePoints = root["reference_points"];
  const int size_reference_points = referencePoints.size();
  data.reference_points.resize(size_reference_points);
  for (int i = 0; i < size_reference_points; ++i) {
    auto &point = data.reference_points[i];
    Json::Value item = referencePoints[i];

    point.id = item["id"].asInt();
    point.x = item["x"].asDouble();
    point.y = item["y"].asDouble();
//    point.enu_x = item["enu_x"].asDouble();
//    point.enu_y = item["enu_y"].asDouble();
    point.s = item["s"].asDouble();
    point.l = item["l"].asDouble();
    point.left_road_width = item["left_road_width"].asDouble();
    point.right_road_width = item["right_road_width"].asDouble();
  }

  // reference splines
  Json::Value referenceSplines = root["reference_splines"];
  const int size_reference_splines = referenceSplines.size();
  data.reference_splines.resize(size_reference_splines);
  for (int i = 0; i < size_reference_splines; ++i) {
    auto &spline = data.reference_splines[i];
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

  // uint8 decision
  data.decision = static_cast<uint8_t>(root["decision"].asInt());

  // string debug_info
  data.debug_info = root["debug_info"].asString();

  m_rWdgCentral.setData(data);
}

void DecisionSubscriber::createSavePath(const boost::filesystem::path &path)
{
  m_fsPath = path;
  m_fsPath /= this->timeToString(1);
}

void DecisionSubscriber::setLiving(bool flag)
{
  m_bFlagLiveing = flag;
  if (flag) {
    m_pTimerSubscribe->start(20);
    m_pTimerReplay->stop();
  }
  else {
    m_pTimerSubscribe->stop();
    m_pTimerReplay->start(REPLAY_INTERVAL[m_nReplayIntervalIndex]);
  }
}

void DecisionSubscriber::onPlayClicked(int index, int value)
{
  switch (index) {
  case QReplayWidget::BtnPlayPause:
    m_bFlagReplayPause = (value == 0);
    break;
  case QReplayWidget::BtnPrevious:
  case QReplayWidget::BtnNext:
    this->replayIndex(value);
    break;
  case QReplayWidget::BtnPlayVelocity:
    this->setReplayVelocity(value);
    break;
  case QReplayWidget::BtnCount:
    this->replayIndex(value);
    break;
  default:
    break;
  }
}

void DecisionSubscriber::replayIndex(int index)
{
  if (!m_bFlagLiveing && m_bFlagReplayPause && index >= 0 &&
      index < m_listPlanningFiles.size()) {
    m_itFile = m_listPlanningFiles.begin();
    std::advance(m_itFile, index);
    this->readDataFromFile(*m_itFile);
  }
}

void DecisionSubscriber::setReplayVelocity(int value)
{
  if (value < 0 || value >= sizeof(REPLAY_INTERVAL) - 1 && m_nReplayIntervalIndex == value) {
    return;
  }
  m_nReplayIntervalIndex = value;
  m_pTimerReplay->setInterval(REPLAY_INTERVAL[m_nReplayIntervalIndex]);
}

int DecisionSubscriber::sizeOfReplayFiles()
{
  return m_listPlanningFiles.size();
}

std::string DecisionSubscriber::timeToString(int type)
{
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);

  struct timeval tv;
  gettimeofday(&tv, NULL);

  char time_sz[64] = {0};
  std::string time_str;
  if (type == 0) {
    sprintf(time_sz, "%04d%02d%02d_%02d%02d%02d_%3d",
            utcTime->tm_year + 1900,
            utcTime->tm_mon + 1,
            utcTime->tm_mday,
            utcTime->tm_hour,
            utcTime->tm_min,
            utcTime->tm_sec,
            static_cast<int>((tv.tv_usec / 1000) % 1000)
            );
  }
  else {
    sprintf(time_sz, "%04d%02d%02d_%02d%02d%02d",
            utcTime->tm_year + 1900,
            utcTime->tm_mon + 1,
            utcTime->tm_mday,
            utcTime->tm_hour,
            utcTime->tm_min,
            utcTime->tm_sec
            );
  }
  time_str = time_sz;

  return time_str;
}

/**
 * @brief 获取某路径下所有数据文件名
 * @param path: 路径名

 * @return: 数据文件名字符串链表
 */
void DecisionSubscriber::fileList(const std::string &path, std::vector<std::string> &files)
{
  files.clear();
  namespace fs = boost::filesystem;
  fs::directory_iterator end_iter;
  for (fs::directory_iterator it(path); it != end_iter; ++it) {
    std::stringstream ss;
    ss << *it;
    std::string name = ss.str();
    std::string substr = name.substr(name.size() - 4, 3);
    if (substr != "txt") {
      continue;
    }
    files.push_back(ss.str());
  }
  std::sort(files.begin(), files.end());
}

