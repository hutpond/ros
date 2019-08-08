#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include "read_files.h"

ReadFiles::ReadFiles()
{

}

void ReadFiles::start_read()
{
  std::cout << "start read" << std::endl;
  run_read_flag_ = true;
  boost::function<void()> fun = boost::bind(&ReadFiles::on_read_files, this);
  boost::thread *read_thread = new boost::thread(fun);
  read_files_thread_.reset(read_thread);
}

void ReadFiles::stop_read()
{
  run_read_flag_ = false;
  read_files_thread_->join();
}

void ReadFiles::setPubTrackFun(
    boost::function<void(const perception_msg_publish::TrackTargetColl &)> fun)
{
  m_funPubTrack = fun;
}

void ReadFiles::setPubUltrasonicFun(
    boost::function<void(const perception_msg_publish::UltrasonicTargetColl &)> fun)
{
  m_funPubUltrasonic = fun;
}

void ReadFiles::setPubMsflOutputFun(
    boost::function<void(const perception_msg_publish::msflOutput &)> fun)
{
  m_funPubMsflOutput = fun;
}

void ReadFiles::on_read_files()
{
  std::cout << "begin read" << std::endl;
  namespace fs = boost::filesystem;
  fs::path path = fs::current_path();
  path /= "PerceptionJsonData";
  std::cout << path.string() << std::endl;
  if (!fs::exists(path)) {
    run_read_flag_ = false;
    return;
  }
  int number = 0;
  fs::directory_iterator end_iter;
  for (fs::directory_iterator iter(path); iter!=end_iter; ++iter) {
    ++ number;
  }
  number /= 2;
  while (run_read_flag_) {
    for (int i = 0; i < number; ++i) {
      if (!run_read_flag_) break;
      char index[8] = {0};
      sprintf(index, "%d", i);

      std::string sense_file_name = path.string() + "/" + index + "_sense.txt";
      this->parse_json_data(i, sense_file_name);

      std::string gps_file_name = path.string() + "/" + index + "_gps.txt";
      this->parse_gps_data(i, gps_file_name);

      usleep(2000 * 1000);
    }
    usleep(3000 * 1000);
  }
  std::cout << "end read" << std::endl;
}

void ReadFiles::parse_json_data(int id, const std::string &sense_name)
{
  std::ifstream in_sense(sense_name.c_str());
  std::string sense_str((std::istreambuf_iterator<char>(in_sense)),
                      std::istreambuf_iterator<char>());
  in_sense.close();
  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(sense_str, root)) {
    return;
  }

  perception_msg_publish::TrackTargetColl msg_track;
  memset(&msg_track, 0, sizeof(msg_track));
  int num_lidar = root["NUM_LIDAR_OBJECTS"].asInt();
  if (num_lidar > 0) {
    Json::Value lidars = root["LIDAR_OBJECTS"];
    for (int i = 0; i < num_lidar; ++i) {
      Json::Value lidar = lidars[i];

      msg_track.track_objects[msg_track.object_count].TRACK_ID = lidar["ID"].asInt();
      msg_track.track_objects[msg_track.object_count].ANGLE = lidar["A"].asFloat();
      msg_track.track_objects[msg_track.object_count].L = lidar["L"].asFloat();
      msg_track.track_objects[msg_track.object_count].W = lidar["W"].asFloat();
      msg_track.track_objects[msg_track.object_count].X = lidar["X"].asFloat();
      msg_track.track_objects[msg_track.object_count].Y = lidar["Y"].asFloat();
      msg_track.track_objects[msg_track.object_count].SX = lidar["SX"].asFloat();
      msg_track.track_objects[msg_track.object_count].P1_X = lidar["P1_X"].asFloat();
      msg_track.track_objects[msg_track.object_count].P1_Y = lidar["P1_Y"].asFloat();
      msg_track.track_objects[msg_track.object_count].P2_X = lidar["P2_X"].asFloat();
      msg_track.track_objects[msg_track.object_count].P2_Y = lidar["P2_Y"].asFloat();
      msg_track.track_objects[msg_track.object_count].P3_X = lidar["P3_X"].asFloat();
      msg_track.track_objects[msg_track.object_count].P3_Y = lidar["P3_Y"].asFloat();
      msg_track.track_objects[msg_track.object_count].P4_X = lidar["P4_X"].asFloat();
      msg_track.track_objects[msg_track.object_count].P4_Y = lidar["P4_Y"].asFloat();

      ++ msg_track.object_count;
    }
  }
  int num_radar = root["NUM_RADAR_OBJECTS"].asInt();
  if (num_radar > 0) {
    Json::Value radars = root["RADAR_OBJECTS"];
    for (int i = 0; i < num_radar; ++i) {
      Json::Value radar = radars[i];

      msg_track.track_objects[msg_track.object_count].TRACK_ID = radar["ID"].asInt();
      msg_track.track_objects[msg_track.object_count].ANGLE = radar["A"].asFloat();
      msg_track.track_objects[msg_track.object_count].L = radar["L"].asFloat();
      msg_track.track_objects[msg_track.object_count].W = radar["W"].asFloat();
      msg_track.track_objects[msg_track.object_count].X = radar["X"].asFloat();
      msg_track.track_objects[msg_track.object_count].Y = radar["Y"].asFloat();

      ++ msg_track.object_count;
    }
  }
  msg_track.id_index = id;
  m_funPubTrack(msg_track);

  perception_msg_publish::UltrasonicTargetColl msg_ultrasonic;
  memset(&msg_ultrasonic, 0, sizeof(msg_ultrasonic));

  Json::Value ultrasonics = root["ULTRASOUND_OBJECTS"];
  const int SIZE = std::min<int>(16, ultrasonics.size());
  msg_ultrasonic.object_count = 0;
  for (int i = 0; i < SIZE; ++i) {
    Json::Value ultrasonic = ultrasonics[i];
    Json::Value available = ultrasonic["AVAILABLE"];
    Json::Value distance = ultrasonic["DISTANCE"];
    Json::Value position = ultrasonic["POSITION"];
    bool avail_flag = available.asBool();
    float dis = distance.asFloat();
    if (!avail_flag || dis <= 0) continue;
    msg_ultrasonic.us_objects[msg_ultrasonic.object_count].distance = dis;
    msg_ultrasonic.us_objects[msg_ultrasonic.object_count].pos_id = position.asInt();

    ++ msg_ultrasonic.object_count;
  }
  msg_ultrasonic.id_index = id;
  m_funPubUltrasonic(msg_ultrasonic);
}

void ReadFiles::parse_gps_data(int id, const std::string &gps_name)
{
  std::ifstream in_sense(gps_name.c_str());
  std::string sense_str((std::istreambuf_iterator<char>(in_sense)),
                      std::istreambuf_iterator<char>());
  in_sense.close();
  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(sense_str, root)) {
    return;
  }

  perception_msg_publish::msflOutput msfl_output;
  memset(&msfl_output, 0, sizeof(msfl_output));
  msfl_output.lat = root["LATITUDE"].asDouble();
  msfl_output.lon = root["LONGTITUDE"].asDouble();
  msfl_output.height = root["ALTITUDE"].asDouble();
  msfl_output.accX = root["ACC_X"].asDouble();
  msfl_output.accY = root["ACC_Y"].asDouble();
  msfl_output.accZ = root["ACC_Z"].asDouble();
  msfl_output.gyroX = root["GYRO_X"].asDouble();
  msfl_output.gyroY = root["GYRO_Y"].asDouble();
  msfl_output.gyroZ = root["GYRO_Z"].asDouble();
  msfl_output.pitch = root["PITCH"].asDouble();
  msfl_output.roll = root["ROLL"].asDouble();
  msfl_output.yaw = root["YAW"].asDouble();
  msfl_output.velEast = root["VEL_EAST"].asDouble();
  msfl_output.velNorth = root["VEL_NORTH"].asDouble();
  msfl_output.velUp = root["VEL_UP"].asDouble();
  msfl_output.gps_ms = root["GPS_MS"].asDouble();
  msfl_output.navState = root["NAV_STATE"].asDouble();
  msfl_output.gpsState = root["GPS_STATE"].asDouble();

  msfl_output.id_index = id;
  m_funPubMsflOutput(msfl_output);
}

