#include <iostream>
#include <fstream>
#include <thread>
#include "ros/ros.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

void publish_msg();
void readMsgFile(int, debug_ads_msgs::ads_msgs_planning_debug_frame &);
bool flag_run = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sweeper_debug_planning_publisher_node");

  flag_run = true;
  std::thread thread(publish_msg);
  while (true)
  {
      usleep(100 * 1000);
      char c;
      std::cin.get(c);
      if (c == 'q') break;
  }
  flag_run = false;
  thread.join();

  return 0;
}

void publish_msg()
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<debug_ads_msgs::ads_msgs_planning_debug_frame>(
        "planning_debug_data_new", 5);
  ros::Rate loop_rate(100);
  debug_ads_msgs::ads_msgs_planning_debug_frame frame;
  int index = 0;
  while (ros::ok() && flag_run)
  {
    readMsgFile(index, frame);
    ++ index %= 2;
    pub.publish(frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void readMsgFile(int index, debug_ads_msgs::ads_msgs_planning_debug_frame &frame)
{
  std::string path = getenv("HOME");
  if (path.back() != '/') {
    path.push_back('/');
  }
  path += "NewPlanningData/MsgData/csv";
  if (index == 1) {
    path.push_back('2');
  }
  path.push_back('/');
  char comma;

  std::ifstream in;
  in.open(path + "decision.csv", std::ios::out);
  in >> frame.state_machine.decision >> comma
           >> frame.state_machine.last_state >> comma
           >> frame.state_machine.current_state >> comma
           >> frame.state_machine.stop_reason;
  in.close();

  in.open(path + "vehicle_state_enu.csv", std::ios::out);
  for(int i = 0; i < 4; i++){
    in >> frame.ego_state_enu[i].X >> comma
             >> frame.ego_state_enu[i].Y >> comma
             >> frame.ego_state_enu[i].Z;
  }
  in.close();

  in.open(path + "vehicle_state_frenet.csv", std::ios::out);
  for(int i = 0; i < 4; i++){
    in >> frame.ego_state_frenet[i].s >> comma
             >> frame.ego_state_frenet[i].l;
  }
  in.close();

  in.open(path + "obstacles_enu.csv", std::ios::out);
  frame.num_obstacle = 0;
  frame.obstacles.clear();
  while (!in.eof()) {
    debug_ads_msgs::ads_msgs_planning_debug_obstacle obstacles;
    for(int j = 0; j < 4; j++){
      if (in.eof()) break;
      in >> obstacles.points_enu[j].X >> comma
               >> obstacles.points_enu[j].Y >> comma
               >> obstacles.points_enu[j].Z >> comma;
    }
    if (in.eof()) break;
    //in >> comma;

    frame.num_obstacle += 1;
    frame.obstacles.push_back(obstacles);
  }
  in.close();

  in.open(path + "obstacles_frenet.csv", std::ios::out);
  for(int i = 0;i < frame.num_obstacle; i++){
    for(int j = 0; j < 4; j++){
      in >> frame.obstacles[i].points_frenet[j].s >> comma
               >> frame.obstacles[i].points_frenet[j].l >> comma;
    }
    //in >> comma;
  }
  in.close();


  in.open(path + "trajectory_current.csv", std::ios::out);
  frame.trajectory_current.current_traj_points_enu.clear();
  while (!in.eof()) {
    debug_ads_msgs::ads_msgs_planning_debug_pointENU point;
    in >> point.X >> comma
        >> point.Y >> comma
        >> point.Z >> comma
        >> point.v >> comma
        >> point.a >> comma
        >> point.kappa >> comma
        >> point.theta >> comma
        >> point.s;
    if (in.eof()) break;
    frame.trajectory_current.current_traj_points_enu.push_back(point);
  }
  in.close();

  in.open(path + "trajectory_current_frenet.csv", std::ios::out);
  frame.trajectory_current.current_traj_points_frenet.clear();
  while (!in.eof()) {
    debug_ads_msgs::ads_msgs_planning_debug_pointFRENET point;
    in >> point.s >> comma
        >> point.l;
    if (in.eof()) break;
    frame.trajectory_current.current_traj_points_frenet.push_back(point);
  }
  in.close();

  in.open(path + "trajectory_last.csv", std::ios::out);
  frame.trajectory_last.current_traj_points_enu.clear();
  while (!in.eof()) {
    debug_ads_msgs::ads_msgs_planning_debug_pointENU point;
    in >> point.X >> comma
        >> point.Y >> comma
        >> point.Z;
    if (in.eof()) break;
    frame.trajectory_last.current_traj_points_enu.push_back(point);
  }
  in.close();

  in.open(path + "reference_line.csv", std::ios::out);
  frame.reference_line_enu.clear();
  while (!in.eof()) {
    debug_ads_msgs::ads_msgs_planning_debug_pointENU point;
    in >> point.X >> comma
        >> point.Y >> comma
        >> point.Z;
    if (in.eof()) break;
    frame.reference_line_enu.push_back(point);
  }
  in.close();
}
