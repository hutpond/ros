/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDataDisplayQWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/12
 * Description: 显示详细数据
********************************************************/
#include "QDataDisplayWidget.h"

#include <QTreeWidget>
#include <QHeaderView>

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
  itemRoot->setText(0, "vehicle(id,x,y,enu_x,enu_y,s,l,left,right)");

  QTreeWidgetItem *item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("front len width, rear len width, angle: %1, %2, %3, %4, %5").
                arg(planningData.front_vehicle_length).
                arg(planningData.front_vehicle_width).
                arg(planningData.rear_vehicle_length).
                arg(planningData.rear_vehicle_width).
                arg(planningData.steering_angle)
                );

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("vehicle enu_x, enu_y, enu_z, pitch, roll, yaw: %1, %2, %3, %4, %5, %6").
                arg(planningData.vehicle_enu_x).
                arg(planningData.vehicle_enu_y).
                arg(planningData.vehicle_enu_z).
                arg(planningData.vehicle_pitch).
                arg(planningData.vehicle_roll).
                arg(planningData.vehicle_yaw)
                );

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("front_axle_center: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                arg(planningData.front_axle_center.id).
                arg(planningData.front_axle_center.x).
                arg(planningData.front_axle_center.y).
                arg(planningData.front_axle_center.enu_x).
                arg(planningData.front_axle_center.enu_y).
                arg(planningData.front_axle_center.s).
                arg(planningData.front_axle_center.l).
                arg(planningData.front_axle_center.left_road_width).
                arg(planningData.front_axle_center.right_road_width)
                );

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("rear_axle_center: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                arg(planningData.rear_axle_center.id).
                arg(planningData.rear_axle_center.x).
                arg(planningData.rear_axle_center.y).
                arg(planningData.rear_axle_center.enu_x).
                arg(planningData.rear_axle_center.enu_y).
                arg(planningData.rear_axle_center.s).
                arg(planningData.rear_axle_center.l).
                arg(planningData.rear_axle_center.left_road_width).
                arg(planningData.rear_axle_center.right_road_width)
                );

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("head_point: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                arg(planningData.head_point.id).
                arg(planningData.head_point.x).
                arg(planningData.head_point.y).
                arg(planningData.head_point.enu_x).
                arg(planningData.head_point.enu_y).
                arg(planningData.head_point.s).
                arg(planningData.head_point.l).
                arg(planningData.head_point.left_road_width).
                arg(planningData.head_point.right_road_width)
                );

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("rear_point: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                arg(planningData.rear_point.id).
                arg(planningData.rear_point.x).
                arg(planningData.rear_point.y).
                arg(planningData.rear_point.enu_x).
                arg(planningData.rear_point.enu_y).
                arg(planningData.rear_point.s).
                arg(planningData.rear_point.l).
                arg(planningData.rear_point.left_road_width).
                arg(planningData.rear_point.right_road_width)
                );

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("hinge_point: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                arg(planningData.hinge_point.id).
                arg(planningData.hinge_point.x).
                arg(planningData.hinge_point.y).
                arg(planningData.hinge_point.enu_x).
                arg(planningData.hinge_point.enu_y).
                arg(planningData.hinge_point.s).
                arg(planningData.hinge_point.l).
                arg(planningData.hinge_point.left_road_width).
                arg(planningData.hinge_point.right_road_width)
                );

  // reference
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  int SIZE = planningData.reference_points.size();
  itemRoot->setText(0, QString("REFERENCE_LINE [%1]").arg(SIZE));
  for (int i = 0; i < SIZE; ++i) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("ID: %1, L: %2, S: %3, X: %4, Y: %5, LEFT: %6, RIGHT: %7").
                  arg(planningData.reference_points[i].id, 2, 10, QLatin1Char(' ')).
                  arg(planningData.reference_points[i].l, 1).
                  arg(planningData.reference_points[i].s, 6, 'f', 2).
                  arg(planningData.reference_points[i].x, 6, 'f', 2).
                  arg(planningData.reference_points[i].y, 6, 'f', 2).
                  arg(planningData.reference_points[i].left_road_width, 6, 'f', 2).
                  arg(planningData.reference_points[i].right_road_width, 6, 'f', 2)
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

  // radar and ultrasonic point
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, QString("radar and ultrasonic point(id,x,y,enu_x,enu_y,s,l,left,right)"));

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("radar_Point: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                arg(planningData.radar_Point.id).
                arg(planningData.radar_Point.x).
                arg(planningData.radar_Point.y).
                arg(planningData.radar_Point.enu_x).
                arg(planningData.radar_Point.enu_y).
                arg(planningData.radar_Point.s).
                arg(planningData.radar_Point.l).
                arg(planningData.radar_Point.left_road_width).
                arg(planningData.radar_Point.right_road_width)
                );

  const int size_ultrasonic_points = planningData.ultrasonic_points.size();
  for (int i = 0; i < size_ultrasonic_points; ++i) {
    const auto &point = planningData.ultrasonic_points[i];
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("ultrasonic_points %1: %2, %3, %4, %5, %6, %7, %8, %9, %10").
                  arg(i).arg(point.id).arg(point.x).arg(point.y).arg(point.enu_x).
                  arg(point.enu_y).arg(point.s).arg(point.l).arg(point.left_road_width).
                  arg(point.right_road_width)
                  );
  }

  // radar result
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, QString("radar_results(id,range,range_lat,range_lon,angle,vel,v_lat,v_lon,status,w,l,devid)"));
  const int size_radar_results = planningData.radar_results.size();
  for (int i = 0; i < size_radar_results; ++i) {
    const auto &result = planningData.radar_results[i];
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("%1: %2, %3, %4, %5, %6, %7, %8, %9, %10, %11, %12, %13").
                  arg(i).arg(result.id).arg(result.range).arg(result.range_lat).
                  arg(result.range_lon).arg(result.angle).arg(result.vel).arg(result.v_lat).
                  arg(result.v_lon).arg(result.status).arg(result.w).arg(result.l).arg(result.devid)
                  );
  }

  // ultrasonic result
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  const int size_ultrasonic_results = planningData.ultrasonic_results.size();
  itemRoot->setText(0, QString("ultrasonic_results %1(id,distance)").arg(size_ultrasonic_results));
  for (const auto &result : planningData.ultrasonic_results) {
    item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString("%1, %2").
                  arg(result.radar_pos_id).arg(result.distance)
                  );
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
    itemChild->setText(0, QString("DIRX: %1").arg(planningData.fusion_results[i].DIRX));
    itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("DIRY: %1").arg(planningData.fusion_results[i].DIRY));
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

  // garbage_detection_results
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  SIZE = static_cast<int>(planningData.garbage_detection_results.size());
  itemRoot->setText(0, QString("garbage_detection_results [%1]").arg(SIZE));
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

  // decision
  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "DECISION_STATE");

  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("DECISION: %1").arg(planningData.decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("RADAR_DECISION: %1").arg(planningData.radar_decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("ULTRASONIC_DECISION: %1").arg(planningData.ultrasonic_decision));
  item = new QTreeWidgetItem(itemRoot);
  item->setText(0, QString("TRACK_TARGET_DECISION: %1").arg(planningData.track_target_decision));

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
    const auto &val_candidates_points = val_candidates[i].points;
    int size_candidates_points = static_cast<int>(val_candidates_points.size());

    item = new QTreeWidgetItem(itemRoot);
    QString text = QString(
          "id: %1, cost: %2, safety: %3, lateral: %4, smoothness: %5, consistency: %6, garbage: %7, splines: %8, points: %9").
        arg(val_candidates[i].id).
        arg(val_candidates[i].cost).
        arg(val_candidates[i].safety_cost).
        arg(val_candidates[i].lateral_cost).
        arg(val_candidates[i].smoothness_cost).
        arg(val_candidates[i].consistency_cost).
        arg(val_candidates[i].garbage_cost).
        arg(size_candidates_splines).
        arg(size_candidates_points);
    item->setText(0, text);

    for (int j = 0; j < size_candidates_splines; ++ j) {
      QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
      text = QString("spline index:%1 xb(x: %2, y: %3, z: %4, w: %5) yb(x: %6, y: %7, z: %8, w: %9)").
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

    for (int j = 0; j < size_candidates_points; ++ j) {
      QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
      itemChild->setText(0, QString("point: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                         arg(val_candidates_points[j].id).
                         arg(val_candidates_points[j].x).
                         arg(val_candidates_points[j].y).
                         arg(val_candidates_points[j].enu_x).
                         arg(val_candidates_points[j].enu_y).
                         arg(val_candidates_points[j].s).
                         arg(val_candidates_points[j].l).
                         arg(val_candidates_points[j].left_road_width).
                         arg(val_candidates_points[j].right_road_width)
                         );
    }
  }

  itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "PLANNING_TRAJECTORY");
  const auto &val_planning_trajectory = planningData.planning_trajectory;
  const auto &val_planning_trajectory_spines = val_planning_trajectory.splines;
  int size_trajectory_splines = static_cast<int>(val_planning_trajectory_spines.size());
  const auto &val_planning_trajectory_points = val_planning_trajectory.points;
  int size_trajectory_points = static_cast<int>(val_planning_trajectory_points.size());
  item = new QTreeWidgetItem(itemRoot);
  QString text = QString(
        "id: %1, cost: %2, safety: %3, lateral: %4, smoothness: %5, consistency: %6, garbage: %7, splines: %8, points: %9").
      arg(val_planning_trajectory.id).
      arg(val_planning_trajectory.cost).
      arg(val_planning_trajectory.safety_cost).
      arg(val_planning_trajectory.lateral_cost).
      arg(val_planning_trajectory.smoothness_cost).
      arg(val_planning_trajectory.consistency_cost).
      arg(val_planning_trajectory.garbage_cost).
      arg(size_trajectory_splines).
      arg(size_trajectory_points);
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

  for (int j = 0; j < size_trajectory_points; ++ j) {
    QTreeWidgetItem *itemChild = new QTreeWidgetItem(item);
    itemChild->setText(0, QString("point: %1, %2, %3, %4, %5, %6, %7, %8, %9").
                       arg(val_planning_trajectory_points[j].id).
                       arg(val_planning_trajectory_points[j].x).
                       arg(val_planning_trajectory_points[j].y).
                       arg(val_planning_trajectory_points[j].enu_x).
                       arg(val_planning_trajectory_points[j].enu_y).
                       arg(val_planning_trajectory_points[j].s).
                       arg(val_planning_trajectory_points[j].l).
                       arg(val_planning_trajectory_points[j].left_road_width).
                       arg(val_planning_trajectory_points[j].right_road_width)
                       );
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
