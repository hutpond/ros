/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDataDisplayDialog.cpp
 * Author: liuzheng
 * Date: 2019/7/12
 * Description: 显示详细数据
********************************************************/
#include <QTreeWidget>
#include <QHeaderView>
#include "QDataDisplayDialog.h"

QDataDisplayDialog::QDataDisplayDialog(QWidget *parent)
  : QWidget(parent)
{
  m_pTreeWidget = new QTreeWidget(this);
  m_pTreeWidget->header()->hide();
}

void QDataDisplayDialog::resizeEvent(QResizeEvent *)
{
  m_pTreeWidget->setGeometry(this->rect());
}

/*******************************************************
 * @brief 设置显示数据
 * @param text: 显示内容

 * @return
********************************************************/
void QDataDisplayDialog::setPlanningData(
    const debug_tool::ads_PlanningData4Debug &planningData)
{
  m_pTreeWidget->clear();
  // car status
  QTreeWidgetItem *itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "CAR_STATUS");

  QTreeWidgetItem *item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("LONGITUDE: %1").arg(planningData.vehicle_longitude));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("LATITUDE: %1").arg(planningData.vehicle_latitude));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ALTITUDE: %1").arg(planningData.vehicle_altitude));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ENU_X: %1").arg(planningData.vehicle_x));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ENU_Y: %1").arg(planningData.vehicle_y));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ENU_Z: %1").arg(planningData.vehicle_z));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("L: %1").arg(planningData.vehicle_l));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("S: %1").arg(planningData.vehicle_s));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("VEHICLE_LENGTH: %1").arg(planningData.vehicle_length));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("VEHICLE_WIDTH: %1").arg(planningData.vehicle_width));

  // decision
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "DECISION_STATE");

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("DECISION: %1").arg(planningData.decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RADAR28_DECISION: %1").arg(planningData.radar28f_decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RADAR73_DECISION: %1").arg(planningData.radar73f_decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ULTRASONIC_DECISION: %1").arg(planningData.ultrasonic_decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("TRACK_TARGET_DECISION: %1").arg(planningData.track_target_decision));

  // target points
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "TARGET_POINT");
  for (int i = 0; i < 4; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("sensor_type: %1").
                       arg(static_cast<int>(planningData.decision_targets[i].sensor_type)));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("x: %1").arg(planningData.decision_targets[i].x));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("y: %1").arg(planningData.decision_targets[i].y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("angle: %1").arg(planningData.decision_targets[i].angle));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("width: %1").arg(planningData.decision_targets[i].width));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("length: %1").arg(planningData.decision_targets[i].length));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("s: %1").arg(planningData.decision_targets[i].s));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("l: %1").arg(planningData.decision_targets[i].l));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("sl_width: %1").arg(planningData.decision_targets[i].sl_width));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("sl_length: %1").arg(planningData.decision_targets[i].sl_length));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p1_x: %1").arg(planningData.decision_targets[i].p1_x));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p1_y: %1").arg(planningData.decision_targets[i].p1_y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p2_x: %1").arg(planningData.decision_targets[i].p2_x));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p2_y: %1").arg(planningData.decision_targets[i].p2_y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p3_x: %1").arg(planningData.decision_targets[i].p3_x));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p3_y: %1").arg(planningData.decision_targets[i].p3_y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p4_x: %1").arg(planningData.decision_targets[i].p4_x));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("p4_y: %1").arg(planningData.decision_targets[i].p4_y));
  }

  // road
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "ROAD_INFO");

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("LEFT_HALF_ROAD_WIDTH: %1").arg(planningData.left_half_road_width));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RIGHT_HALF_ROAD_WIDTH: %1").arg(planningData.right_half_road_width));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("MIN_SAFE_DISTANCE: %1").arg(planningData.safe_dis1));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("SAFE_DISTANCE: %1").arg(planningData.safe_dis2));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("MAX_PLANNING_DISTANCE: %1").arg(planningData.max_planning_distance));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("LEFT_ROAD_BOUNDARY_AVAILABLE: %1").
                arg(planningData.left_road_boundary_available ? "true" : "false"));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RIGHT_ROAD_BOUNDARY_AVAILABLE: %1").
                arg(planningData.right_road_boundary_available ? "true" : "false"));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, "LEFT_ROAD_BOUNDARY");
  for (int i = 0; i < 5; ++i) {
    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("BOUNDARY%1: %2").
                       arg(i).
                       arg(planningData.left_road_boundary[i]));
  }
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, "RIGHT_ROAD_BOUNDARY");
  for (int i = 0; i < 5; ++i) {
    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("BOUNDARY%1: %2").
                       arg(i).
                       arg(planningData.right_road_boundary[i]));
  }
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("LEFT_ROAD_BOUNDARY_S: %1, %2").
                arg(planningData.left_road_boundary_start_s).
                arg(planningData.left_road_boundary_end_s));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RIGHT_ROAD_BOUNDARY_S: %1, %2").
                arg(planningData.right_road_boundary_start_s).
                arg(planningData.right_road_boundary_end_s));

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ADS_PUBCURB"));
  QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("CURB_L_FOUND: %1").
                     arg(planningData.curb.curb_L_FOUND ? "true" : "false"));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_L1_X: %1").arg(planningData.curb.Point_L1.x));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_L1_Y: %1").arg(planningData.curb.Point_L1.y));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_L2_X: %1").arg(planningData.curb.Point_L2.x));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_L2_Y: %1").arg(planningData.curb.Point_L2.y));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("CURB_R_FOUND: %1").
                     arg(planningData.curb.curb_R_FOUND ? "true" : "false"));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_R1_X: %1").arg(planningData.curb.Point_R1.x));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_R1_Y: %1").arg(planningData.curb.Point_R1.y));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_R2_X: %1").arg(planningData.curb.Point_R2.x));
  itemChild = new QTreeWidgetItem(item);
  itemChild->setText(0, QString("Point_R2_Y: %1").arg(planningData.curb.Point_R2.y));

  const int SIZE_LEFT_ROAD_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_left_road_boundary_splines), 100);
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("LEFT_ROAD_BOUNDARY_SPLINES [%1]").arg(SIZE_LEFT_ROAD_SPLINES));
  for (int i = 0; i < SIZE_LEFT_ROAD_SPLINES; ++i) {
    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    QString text = QString("Index:%1 xb(x: %2, y: %3, z: %4, w: %5) yb(x: %6, y: %7, z: %8, w: %9)").
        arg(i).
        arg(planningData.left_road_boundary_splines[i].xb.x).
        arg(planningData.left_road_boundary_splines[i].xb.y).
        arg(planningData.left_road_boundary_splines[i].xb.z).
        arg(planningData.left_road_boundary_splines[i].xb.w).
        arg(planningData.left_road_boundary_splines[i].yb.x).
        arg(planningData.left_road_boundary_splines[i].yb.y).
        arg(planningData.left_road_boundary_splines[i].yb.z).
        arg(planningData.left_road_boundary_splines[i].yb.w);
    itemChild->setText(0, text);
  }
  const int SIZE_RIGHT_ROAD_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_right_road_boundary_splines), 100);
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RIGHT_ROAD_BOUNDARY_SPLINES [%1]").arg(SIZE_RIGHT_ROAD_SPLINES));
  for (int i = 0; i < SIZE_RIGHT_ROAD_SPLINES; ++i) {
    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    QString text = QString("Index:%1 xb(x: %2, y: %3, z: %4, w: %5) yb(x: %6, y: %7, z: %8, w: %9)").
        arg(i).
        arg(planningData.right_road_boundary_splines[i].xb.x).
        arg(planningData.right_road_boundary_splines[i].xb.y).
        arg(planningData.right_road_boundary_splines[i].xb.z).
        arg(planningData.right_road_boundary_splines[i].xb.w).
        arg(planningData.right_road_boundary_splines[i].yb.x).
        arg(planningData.right_road_boundary_splines[i].yb.y).
        arg(planningData.right_road_boundary_splines[i].yb.z).
        arg(planningData.right_road_boundary_splines[i].yb.w);
    itemChild->setText(0, text);
  }

  // planning path
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "TRAJECTORY");
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("decision: %1").arg(static_cast<int>(planningData.planning_output.decision)));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("velocity: %1").arg(planningData.planning_output.velocity));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("position.x: %1").arg(planningData.planning_output.pose.position.x));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("position.y: %1").arg(planningData.planning_output.pose.position.y));

  auto &val_candidates = planningData.planning_trajectory_candidates;
  const int size_candidates = val_candidates.size();
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, QString("PLANNING_TRAJECTORY_CANDIDATES (%1)").arg(size_candidates));
  for (int i = 0; i < size_candidates; ++ i) {
    const auto &val_candidates_spines = val_candidates[i].splines;
    int size_candidates_splines = static_cast<int>(val_candidates_spines.size());
    item = new QTreeWidgetItem(itemRoot);
    QString text = QString(
          "id: %1, cost: %2, safety: %3, lateral: %4, smoothness: %5, consistency: %6, garbage: %7, splines: %8").
        arg(val_candidates[i].id).
        arg(val_candidates[i].cost).
        arg(val_candidates[i].safety_cost).
        arg(val_candidates[i].lateral_cost).
        arg(val_candidates[i].smoothness_cost).
        arg(val_candidates[i].consistency_cost).
        arg(val_candidates[i].garbage_cost).
        arg(size_candidates_splines);
    item->setText(0, text);

    for (int j = 0; j < size_candidates_splines; ++ j) {
      QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
      text = QString("Index:%1 xb(x: %2, y: %3, z: %4, w: %5) yb(x: %6, y: %7, z: %8, w: %9)").
          arg(j).
          arg(val_candidates_spines[j].xb.x).
          arg(val_candidates_spines[j].xb.y).
          arg(val_candidates_spines[j].xb.z).
          arg(val_candidates_spines[j].xb.w).
          arg(val_candidates_spines[j].yb.x).
          arg(val_candidates_spines[j].yb.y).
          arg(val_candidates_spines[j].yb.z).
          arg(val_candidates_spines[j].yb.w);
      itemChild->setText(0, text);
    }
  }

  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "PLANNING_TRAJECTORY");
  const auto &val_planning_trajectory = planningData.planning_trajectory;
  const auto &val_planning_trajectory_spines = val_planning_trajectory.splines;
  int size_trajectory_splines = static_cast<int>(val_planning_trajectory_spines.size());
  item = new QTreeWidgetItem(itemRoot);
  QString text = QString(
        "id: %1, cost: %2, safety: %3, lateral: %4, smoothness: %5, consistency: %6, garbage: %7, splines: %8").
      arg(val_planning_trajectory.id).
      arg(val_planning_trajectory.cost).
      arg(val_planning_trajectory.safety_cost).
      arg(val_planning_trajectory.lateral_cost).
      arg(val_planning_trajectory.smoothness_cost).
      arg(val_planning_trajectory.consistency_cost).
      arg(val_planning_trajectory.garbage_cost).
      arg(size_trajectory_splines);
  item->setText(0, text);

  for (int j = 0; j < size_trajectory_splines; ++ j) {
    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    QString text = QString("Index:%1 xb(x: %2, y: %3, z: %4, w: %5) yb(x: %6, y: %7, z: %8, w: %9)").
        arg(j).
        arg(val_planning_trajectory_spines[j].xb.x).
        arg(val_planning_trajectory_spines[j].xb.y).
        arg(val_planning_trajectory_spines[j].xb.z).
        arg(val_planning_trajectory_spines[j].xb.w).
        arg(val_planning_trajectory_spines[j].yb.x).
        arg(val_planning_trajectory_spines[j].yb.y).
        arg(val_planning_trajectory_spines[j].yb.z).
        arg(val_planning_trajectory_spines[j].yb.w);
    itemChild->setText(0, text);
  }

  // reference
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  int SIZE = qBound<int>(0, static_cast<int>(planningData.num_reference_points), 100);
  itemRoot->setText(0, QString("REFERENCE_LINE [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ID: %1").arg(planningData.reference_points[i].id));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.reference_points[i].l));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("S: %1").arg(planningData.reference_points[i].s));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("X: %1").arg(planningData.reference_points[i].x));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("Y: %1").arg(planningData.reference_points[i].y));
  }

  // reference splines
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  int SIZE_REFERENCE_SPLINES = qBound<int>(
        0, static_cast<int>(planningData.num_reference_splines), 100);
  itemRoot->setText(0, QString("REFERENCE_SPLINES [%1]").arg(SIZE_REFERENCE_SPLINES));
  for (int i = 0; i < SIZE_REFERENCE_SPLINES; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    QString text = QString("Index:%1 xb(x: %2, y: %3, z: %4, w: %5) yb(x: %6, y: %7, z: %8, w: %9)").
        arg(i).
        arg(planningData.reference_splines[i].xb.x).
        arg(planningData.reference_splines[i].xb.y).
        arg(planningData.reference_splines[i].xb.z).
        arg(planningData.reference_splines[i].xb.w).
        arg(planningData.reference_splines[i].yb.x).
        arg(planningData.reference_splines[i].yb.y).
        arg(planningData.reference_splines[i].yb.z).
        arg(planningData.reference_splines[i].yb.w);
    item->setText(0, text);
  }

  // radar 28 targets
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.radar28f_results.object_count);
  itemRoot->setText(0, QString("RADAR28_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ID: %1").arg(planningData.radar28f_results.radar_objects[i].id));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ANGLE: %1").arg(planningData.radar28f_results.radar_objects[i].angle));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("DEVID: %1").arg(planningData.radar28f_results.radar_objects[i].devid));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.radar28f_results.radar_objects[i].l));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("W: %1").arg(planningData.radar28f_results.radar_objects[i].w));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE: %1").arg(planningData.radar28f_results.radar_objects[i].range));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LAT: %1").arg(planningData.radar28f_results.radar_objects[i].range_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LON: %1").arg(planningData.radar28f_results.radar_objects[i].range_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LAT: %1").arg(planningData.radar28f_results.radar_objects[i].v_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LON: %1").arg(planningData.radar28f_results.radar_objects[i].v_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("STATUS: %1").arg(planningData.radar28f_results.radar_objects[i].status));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("VEL: %1").arg(planningData.radar28f_results.radar_objects[i].vel));
  }

  // radar 73 targets
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.radar73f_results.object_count);
  itemRoot->setText(0, QString("RADAR73_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ID: %1").arg(planningData.radar73f_results.radar_objects[i].id));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ANGLE: %1").arg(planningData.radar73f_results.radar_objects[i].angle));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("DEVID: %1").arg(planningData.radar73f_results.radar_objects[i].devid));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.radar73f_results.radar_objects[i].l));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("W: %1").arg(planningData.radar73f_results.radar_objects[i].w));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE: %1").arg(planningData.radar73f_results.radar_objects[i].range));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LAT: %1").arg(planningData.radar73f_results.radar_objects[i].range_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LON: %1").arg(planningData.radar73f_results.radar_objects[i].range_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LAT: %1").arg(planningData.radar73f_results.radar_objects[i].v_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LON: %1").arg(planningData.radar73f_results.radar_objects[i].v_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("STATUS: %1").arg(planningData.radar73f_results.radar_objects[i].status));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("VEL: %1").arg(planningData.radar73f_results.radar_objects[i].vel));
  }

  // ultrasonic
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.ultrasonic_results.object_count);
  itemRoot->setText(0, QString("ULTRASONIC_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("POS_ID: %1  DISTANCE: %2").
                  arg(planningData.ultrasonic_results.us_objects[i].radar_pos_id).
                  arg(planningData.ultrasonic_results.us_objects[i].distance));
  }

  // track targets
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.fusion_results.object_count);
  itemRoot->setText(0, QString("TRACK_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("TRACK_ID: %1").arg(planningData.fusion_results.track_objects[i].TRACK_ID));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("X: %1").arg(planningData.fusion_results.track_objects[i].X));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("Y: %1").arg(planningData.fusion_results.track_objects[i].Y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ANGLE: %1").arg(planningData.fusion_results.track_objects[i].ANGLE));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("SX: %1").arg(planningData.fusion_results.track_objects[i].SX));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("W: %1").arg(planningData.fusion_results.track_objects[i].W));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.fusion_results.track_objects[i].L));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("H: %1").arg(planningData.fusion_results.track_objects[i].H));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P1_X: %1").arg(planningData.fusion_results.track_objects[i].P1_X));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P1_Y: %1").arg(planningData.fusion_results.track_objects[i].P1_Y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P2_X: %1").arg(planningData.fusion_results.track_objects[i].P2_X));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P2_Y: %1").arg(planningData.fusion_results.track_objects[i].P2_Y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P3_X: %1").arg(planningData.fusion_results.track_objects[i].P3_X));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P3_Y: %1").arg(planningData.fusion_results.track_objects[i].P3_Y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P4_X: %1").arg(planningData.fusion_results.track_objects[i].P4_X));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("P4_Y: %1").arg(planningData.fusion_results.track_objects[i].P4_Y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("STATUS: %1").arg(planningData.fusion_results.track_objects[i].STATUS));
  }

  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.garbage_detection_results.result.size());
  itemRoot->setText(0, QString("GARBAGE_RESULTS [%1]").arg(SIZE));
  const auto &garbage_result = planningData.garbage_detection_results.result;
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("ID: %1, SIZE: %2, ANGLE: %3, DISTANCE: %4").
                  arg(garbage_result[i].id).
                  arg(garbage_result[i].size).
                  arg(garbage_result[i].angle).
                  arg(garbage_result[i].distance));
  }
}
