/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDataDisplayQWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/12
 * Description: 显示详细数据
********************************************************/
#include <QTreeWidget>
#include <QHeaderView>
#include "QDataDisplayWidget.h"

QDataDisplayWidget::QDataDisplayWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pTreeWidget = new QTreeWidget(this);
  m_pTreeWidget->header()->hide();
  m_pTreeWidget->setFont(QFont("Mono", 12));
}

void QDataDisplayWidget::resizeEvent(QResizeEvent *)
{
  m_pTreeWidget->setGeometry(this->rect());
}

/*******************************************************
 * @brief 设置显示数据
 * @param text: 显示内容

 * @return
********************************************************/
void QDataDisplayWidget::setPlanningData(
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
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("HEAD_DISTANCE: %1").arg(planningData.head_distance));

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
  item->setText(0, QString("LEFT_HALF_ROAD_WIDTH: %1").arg(planningData.left_road_width));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RIGHT_HALF_ROAD_WIDTH: %1").arg(planningData.right_road_width));
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

  const int SIZE_LEFT_ROAD_SPLINES = planningData.left_road_boundary_splines.size();
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
  const int SIZE_RIGHT_ROAD_SPLINES = planningData.right_road_boundary_splines.size();
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

  // planning cost weight
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "COST WEIGHT");
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("safety: %1").arg(planningData.safety_cost_weight));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("lateral: %1").arg(planningData.lateral_cost_weight));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("smoothness: %1").arg(planningData.smoothness_cost_weight));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("consistency: %1").arg(planningData.consistency_cost_weight));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("garbage: %1").arg(planningData.garbage_cost_weight));

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
  int SIZE = planningData.reference_points.size();
  itemRoot->setText(0, QString("REFERENCE_LINE [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("ID: %1, L: %2, S: %3, X: %4, Y: %5").
                  arg(planningData.reference_points[i].id, 2, 10, QLatin1Char(' ')).
                  arg(planningData.reference_points[i].l, 1).
                  arg(planningData.reference_points[i].s, 6, 'f', 2).
                  arg(planningData.reference_points[i].x, 6, 'f', 2).
                  arg(planningData.reference_points[i].y, 6, 'f', 2)
                  );
  }

  // reference splines
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  int SIZE_REFERENCE_SPLINES = planningData.reference_splines.size();
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
  SIZE = planningData.radar28f_results.size();
  itemRoot->setText(0, QString("RADAR28_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ID: %1").arg(planningData.radar28f_results[i].id));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ANGLE: %1").arg(planningData.radar28f_results[i].angle));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("DEVID: %1").arg(planningData.radar28f_results[i].devid));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.radar28f_results[i].l));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("W: %1").arg(planningData.radar28f_results[i].w));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE: %1").arg(planningData.radar28f_results[i].range));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LAT: %1").arg(planningData.radar28f_results[i].range_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LON: %1").arg(planningData.radar28f_results[i].range_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LAT: %1").arg(planningData.radar28f_results[i].v_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LON: %1").arg(planningData.radar28f_results[i].v_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("STATUS: %1").arg(planningData.radar28f_results[i].status));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("VEL: %1").arg(planningData.radar28f_results[i].vel));
  }

  // radar 73 targets
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.radar73f_results.size());
  itemRoot->setText(0, QString("RADAR73_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ID: %1").arg(planningData.radar73f_results[i].id));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ANGLE: %1").arg(planningData.radar73f_results[i].angle));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("DEVID: %1").arg(planningData.radar73f_results[i].devid));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.radar73f_results[i].l));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("W: %1").arg(planningData.radar73f_results[i].w));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE: %1").arg(planningData.radar73f_results[i].range));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LAT: %1").arg(planningData.radar73f_results[i].range_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("RANGE_LON: %1").arg(planningData.radar73f_results[i].range_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LAT: %1").arg(planningData.radar73f_results[i].v_lat));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("V_LON: %1").arg(planningData.radar73f_results[i].v_lon));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("STATUS: %1").arg(planningData.radar73f_results[i].status));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("VEL: %1").arg(planningData.radar73f_results[i].vel));
  }

  // ultrasonic
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.ultrasonic_results.size());
  itemRoot->setText(0, QString("ULTRASONIC_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("POS_ID: %1  DISTANCE: %2").
                  arg(planningData.ultrasonic_results[i].radar_pos_id).
                  arg(planningData.ultrasonic_results[i].distance));
  }

  // track targets
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.fusion_results.size());
  itemRoot->setText(0, QString("TRACK_TRARGETS [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("Index: %1").arg(i));

    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("TRACK_ID: %1").arg(planningData.fusion_results[i].TRACK_ID));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("X: %1").arg(planningData.fusion_results[i].X));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("Y: %1").arg(planningData.fusion_results[i].Y));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("ANGLE: %1").arg(planningData.fusion_results[i].ANGLE));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("SX: %1").arg(planningData.fusion_results[i].SX));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("W: %1").arg(planningData.fusion_results[i].W));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("L: %1").arg(planningData.fusion_results[i].L));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("H: %1").arg(planningData.fusion_results[i].H));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("DEVICE_SOURCE: %1").
                       arg(static_cast<int>(planningData.fusion_results[i].DEVICE_SOURCE)));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("MOTION_STATUS: %1").
                       arg(static_cast<int>(planningData.fusion_results[i].MOTION_STATUS)));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("TARGET_TYPE: %1").
                       arg(static_cast<int>(planningData.fusion_results[i].TARGET_TYPE)));
    int index = 0;
    for (const auto &point : planningData.fusion_results[i].edge_points) {
      itemChild = new QTreeWidgetItem(item);
      itemChild->setText(0, QString("index: %1, x: %2, y: %3, z: %4").
                         arg(index ++).
                         arg(point.x).
                         arg(point.y).
                         arg(point.z));
    }
  }

  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.garbage_detection_results.size());
  itemRoot->setText(0, QString("GARBAGE_RESULTS [%1]").arg(SIZE));
  const auto &garbage_result = planningData.garbage_detection_results;
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("ID: %1, SIZE: %2, ANGLE: %3, DISTANCE: %4, LENGTH: %5, WIDTH: %6").
                  arg(garbage_result[i].id).
                  arg(garbage_result[i].size).
                  arg(garbage_result[i].angle).
                  arg(garbage_result[i].distance).
                  arg(garbage_result[i].length).
                  arg(garbage_result[i].width));
  }
}

void QDataDisplayWidget::setPlanningData(
    const debug_ads_msgs::ads_msgs_planning_debug_frame &frame)
{
  m_pTreeWidget->clear();
  QTreeWidgetItem *item;

  // obstacle
  QTreeWidgetItem *itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "obstacle");
  const int size_obstacles = frame.obstacles.size();
  for (int i = 0; i < size_obstacles; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("obstacle %1 {[x,y,z,v,a,kappa,dkappa,theta,s],[s,l]}").arg(i));
    for (int j = 0; j < 4; ++j) {
      const auto point_enu = frame.obstacles[i].points_enu[j];
      const auto point_frenet = frame.obstacles[i].points_frenet[j];
      QString text = QString("{[%1,%2,%3,%4,%5,%6,%7,%8,%9],[%10,%11]}").
          arg(point_enu.X).arg(point_enu.Y).arg(point_enu.Z).
          arg(point_enu.v).arg(point_enu.a).arg(point_enu.kappa).
          arg(point_enu.dkappa).arg(point_enu.theta).arg(point_enu.s).
          arg(point_frenet.s).arg(point_frenet.l);

      QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
      itemChild->setText(0, text);
    }
  }

  // current trajectories
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "current trajectories {[x,y,z,v,a,kappa,dkappa,theta,s],[s,l]}");
  const int size_current_trajectories_enu =
      frame.trajectory_current.current_traj_points_enu.size();
  const int size_current_trajectories_frenet =
      frame.trajectory_current.current_traj_points_frenet.size();
  const int size_current_trajectories = qMax<int>(size_current_trajectories_enu, size_current_trajectories_frenet);
  for (int i = 0; i < size_current_trajectories; ++i) {
    QString text;
    if (i < size_current_trajectories_enu) {
      const auto &point_enu = frame.trajectory_current.current_traj_points_enu[i];
      text = QString("{%1:[%2,%3,%4,%5,%6,%7,%8,%9,%10],").
          arg(i).arg(point_enu.X).arg(point_enu.Y).arg(point_enu.Z).
          arg(point_enu.v).arg(point_enu.a).arg(point_enu.kappa).
          arg(point_enu.dkappa).arg(point_enu.theta).arg(point_enu.s);
    }
    else {
      text = QString("{%1:[],").arg(i);
    }
    if (i < size_current_trajectories_frenet) {
      const auto &point_frenet = frame.trajectory_current.current_traj_points_frenet[i];
      text += QString("[%1,%2]}").
              arg(point_frenet.s).arg(point_frenet.l);
    }
    else {
      text += QString("[]}");
    }

    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, text);
  }

  // last trajectories
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "last trajectories {[x,y,z,v,a,kappa,dkappa,theta,s],[s,l]}");
  const int size_last_trajectories_enu =
      frame.trajectory_last.current_traj_points_enu.size();
  const int size_last_trajectories_frenet =
      frame.trajectory_last.current_traj_points_frenet.size();
  const int size_last_trajectories = qMax<int>(size_last_trajectories_enu, size_last_trajectories_frenet);
  for (int i = 0; i < size_last_trajectories; ++i) {
    QString text;
    if (i < size_last_trajectories_enu) {
      const auto &point_enu = frame.trajectory_last.current_traj_points_enu[i];
      text = QString("{%1:[%2,%3,%4,%5,%6,%7,%8,%9,%10],").
          arg(i).arg(point_enu.X).arg(point_enu.Y).arg(point_enu.Z).
          arg(point_enu.v).arg(point_enu.a).arg(point_enu.kappa).
          arg(point_enu.dkappa).arg(point_enu.theta).arg(point_enu.s);
    }
    else {
      text = QString("{%1:[],").arg(i);
    }
    if (i < size_last_trajectories_frenet) {
      const auto &point_frenet = frame.trajectory_last.current_traj_points_frenet[i];
      text += QString("[%1,%2]}").
              arg(point_frenet.s).arg(point_frenet.l);
    }
    else {
      text += QString("[]}");
    }

    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, text);
  }

  // reference enu
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "reference {[x,y,z,v,a,kappa,dkappa,theta,s],[s,l]}");
  const int size_reference_line_enu =
      frame.reference_line_enu.size();
  const int size_reference_line_frenet =
      frame.reference_line_frenet.size();
  const int size_reference_line = qMax<int>(
        size_reference_line_enu, size_reference_line_frenet);
  for (int i = 0; i < size_reference_line; ++i) {
    QString text;
    if (i < size_reference_line_enu) {
      const auto &point_enu = frame.reference_line_enu[i];
      text = QString("{%1:[%2,%3,%4,%5,%6,%7,%8,%9,%10],").
          arg(i).arg(point_enu.X).arg(point_enu.Y).arg(point_enu.Z).
          arg(point_enu.v).arg(point_enu.a).arg(point_enu.kappa).
          arg(point_enu.dkappa).arg(point_enu.theta).arg(point_enu.s);
    }
    else {
      text = QString("{%1:[],").arg(i);
    }

    if (i < size_reference_line_frenet) {
      const auto &point_frenet = frame.reference_line_frenet[i];
      text += QString("[%1,%2]}").
          arg(point_frenet.s).arg(point_frenet.l);
    }
    else {
      text += QString("[]}");
    }

    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, text);
  }

  // ego state enu
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "ego state {[%1,%2,%3,%4,%5,%6,%7,%8,%9],[%10,%11]}");
  for (int i = 0; i < 4; ++i) {
    const auto &point_enu = frame.ego_state_enu[i];
    const auto &point_frenet = frame.ego_state_frenet[i];

    QString text = QString("%1:{[%2,%3,%4,%5,%6,%7,%8,%9,%10],[%11,%12]}").
        arg(i).arg(point_enu.X).arg(point_enu.Y).arg(point_enu.Z).
        arg(point_enu.v).arg(point_enu.a).arg(point_enu.kappa).
        arg(point_enu.dkappa).arg(point_enu.theta).arg(point_enu.s).
        arg(point_frenet.s).arg(point_frenet.l);

    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, text);
  }

  // state machine
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, QString("state machine, LAST_STATE:%1, CURRENT_STATE:%2, DECISION:%3, REASON:%4").
      arg(frame.state_machine.last_state).
      arg(frame.state_machine.current_state).
      arg(frame.state_machine.decision).
      arg(frame.state_machine.stop_reason.c_str()));

  // parameters
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, QString("cost [%1, %2, %3, %4, %5, %6]").
      arg(frame.parameters.cost_1).
      arg(frame.parameters.cost_2).
      arg(frame.parameters.cost_3).
      arg(frame.parameters.cost_4).
      arg(frame.parameters.cost_5).
      arg(frame.parameters.cost_6));
}
