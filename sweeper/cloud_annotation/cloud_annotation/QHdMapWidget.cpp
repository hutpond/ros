#include "QHdMapWidget.h"

#include <fstream>
#include <QTreeWidget>
#include <QMenu>
#include <QMessageBox>
#include <jsoncpp/json/json.h>

#include "QAddSegmentDialog.h"
#include "QPointValue.h"
#include "qcloudpoints.h"

QHdMapWidget::QHdMapWidget(QWidget *parent)
  : QWidget(parent)
{
  tree_hdmap_ = new QTreeWidget(this);
  tree_hdmap_->setHeaderHidden(true);
  tree_hdmap_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(tree_hdmap_, &QTreeWidget::customContextMenuRequested,
          this, &QHdMapWidget::onCustomContextMenuRequested);
  root_item_ = new QTreeWidgetItem(tree_hdmap_, ItemTypeMap);
  root_item_->setText(0, "map");

  this->createTreeMenu();

  point_val_widget_ = new QPointValue(this);
  this->setMinimumWidth(300);

  connect(tree_hdmap_, SIGNAL(itemSelectionChanged()),
          this, SLOT(onItemSelectionChanged()));
  connect(point_val_widget_, SIGNAL(saveData()),
                     this, SLOT(onSavePointValue()));
}

void QHdMapWidget::clear()
{
  const int size_childs = root_item_->childCount();
  for (int i = size_childs - 1; i >= 0; --i) {
    root_item_->removeChild(root_item_->child(i));
  }
}

void QHdMapWidget::saveHdMapData(const QString &fileName)
{
  Json::Value root;
  Json::Value json_segments, json_traffic_lights, json_stop_lines,
      json_crossings, json_markings, json_signs;

  const int size_item = root_item_->childCount();
  for (int i = 0; i < size_item; ++i) {
    // item
    QTreeWidgetItem *item = root_item_->child(i);
    int type = item->type();

    if (type == ItemTypeSegment) {
      // segment
      Json::Value json_segment;
      QTreeMapItem *item_segment = dynamic_cast<QTreeMapItem *>(item);
      json_segment["type"] = item_segment->mapType();

      // road
      Json::Value json_roads;
      const int size_road = item_segment->childCount();
      for (int j = 0; j < size_road; ++j) {
        Json::Value json_road;
        QTreeMapItem *item_road = dynamic_cast<QTreeMapItem *>(item_segment->child(j));

        // road side
        Json::Value json_left_side, json_right_side, json_reference;
        const int size_road_side = item_road->childCount();
        for (int k = 0; k < size_road_side; ++k) {
          QTreeMapItem *item_road_side = dynamic_cast<QTreeMapItem *>(
                item_road->child(k));

          // point
          const int size_point = item_road_side->childCount();
          for (int l = 0; l < size_point; ++l) {
            QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(
                  item_road_side->child(l));
            Json::Value json_point;
            Point point = item_point->point();
            json_point["x"] = point.x;
            json_point["y"] = point.y;
            json_point["z"] = point.z;
            if (item_road_side->mapType() == Road::LEFT) {
              json_left_side.append(json_point);
            }
            else if (item_road_side->mapType() == Road::RIGHT) {
              json_right_side.append(json_point);
            }
            else if (item_road_side->mapType() == Road::CENTRAL) {
              json_reference.append(json_point);
            }
          }

          json_road["left"] = json_left_side;
          json_road["right"] = json_right_side;
          json_road["reference"] = json_reference;
        }

        json_roads.append(json_road);
      }
      json_segment["road"] = json_roads;
      json_segments.append(json_segment);
    }
    else if (type == ItemTypeTrafficLight) {
      QTreeMapItem *item_light = dynamic_cast<QTreeMapItem *>(item);
      if (item_light != nullptr && item_light->childCount() == 1) {
        QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(item_light->child(0));
        if (item_point != nullptr) {
          Json::Value json_light;
          Point point = item_point->point();
          json_light["x"] = point.x;
          json_light["y"] = point.y;
          json_light["z"] = point.z;

          json_traffic_lights.append(json_light);
        }
      }
    }
    else if (type == ItemTypeStopLines) {
      QTreeMapItem *item_stop_line = dynamic_cast<QTreeMapItem *>(item);
      if (item_stop_line != nullptr && item_stop_line->childCount() == 1) {
        QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(item_stop_line->child(0));
        if (item_point != nullptr) {
          Json::Value json_stop_line;
          Point point = item_point->point();
          json_stop_line["x"] = point.x;
          json_stop_line["y"] = point.y;
          json_stop_line["z"] = point.z;

          json_stop_lines.append(json_stop_line);
        }
      }
    }
    else if (type == ItemTypeCrossings) {
      QTreeMapItem *item_crossing = dynamic_cast<QTreeMapItem *>(item);
      if (item_crossing != nullptr && item_crossing->childCount() > 0) {
        Json::Value json_crossing;
        const int size_points = item_crossing->childCount();
        for (int i = 0; i < size_points; ++i) {
          QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(
                item_crossing->child(i));
          if (item_point != nullptr) {
            Json::Value json_point;
            Point point = item_point->point();
            json_point["x"] = point.x;
            json_point["y"] = point.y;
            json_point["z"] = point.z;

            json_crossing.append(json_point);
          }
        }
        json_crossings.append(json_crossing);
      }
    }
    else if (type == ItemTypeMarkings) {
      QTreeMapItem *item_marking = dynamic_cast<QTreeMapItem *>(item);
      if (item_marking != nullptr && item_marking->childCount() == 1) {
        QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(item_marking->child(0));
        if (item_point != nullptr) {
          Json::Value json_marking;
          Point point = item_point->point();
          json_marking["x"] = point.x;
          json_marking["y"] = point.y;
          json_marking["z"] = point.z;

          json_markings.append(json_marking);
        }
      }
    }
    else if (type == ItemTypeSigns) {
      QTreeMapItem *item_sign = dynamic_cast<QTreeMapItem *>(item);
      if (item_sign != nullptr && item_sign->childCount() == 1) {
        QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(item_sign->child(0));
        if (item_point != nullptr) {
          Json::Value json_sign;
          Point point = item_point->point();
          json_sign["x"] = point.x;
          json_sign["y"] = point.y;
          json_sign["z"] = point.z;

          json_signs.append(json_sign);
        }
      }
    }
  }

  root["segment"] = json_segments;
  root["light"] = json_traffic_lights;
  root["stop_line"] = json_stop_lines;
  root["crossing"] = json_crossings;
  root["marking"] = json_markings;
  root["sign"] = json_signs;

  // save file
  std::ofstream out(fileName.toLocal8Bit().data());
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(root, &out);
  out.close();
}

bool QHdMapWidget::parseHdMapData(const QString &fileName)
{
  // read file
  FILE *pf = fopen(fileName.toLocal8Bit().data(), "r");
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

  // segment
  Json::Value &json_segments = root["segment"];
  const int size_segments = json_segments.size();
  for (int i = 0; i < size_segments; ++i) {
    Json::Value &json_roads = json_segments[i]["road"];
    int map_type = json_segments[i]["type"].asInt();
    QTreeMapItem *item_segment = new QTreeMapItem(
          root_item_, ItemTypeSegment, map_type, "Road Segment");
    item_segment->setIndex(i);

    // roads
    const int size_roads = json_roads.size();
    for (int j = 0; j < size_roads; ++j) {
//      item_segment->addRoadSize();
      QTreeMapItem *item_road = new QTreeMapItem(
            item_segment, ItemTypeRoad, 0, "Road");
      item_road->setIndex(j);
      if (item_segment->mapType() == RoadSegment::ROAD) {
        // left
        QTreeMapItem *item_left_side = new QTreeMapItem(
              item_road, ItemTypeRoadSide, Road::LEFT, "Left Side");
        item_left_side->setIndex(0);

        Json::Value &json_left_side = json_roads[j]["left"];
        const int size_left_points = json_left_side.size();
        for (int k = 0; k < size_left_points; ++k) {
          Json::Value &json_point = json_left_side[k];
          QTreeMapItem *item_point = new QTreeMapItem(
                item_left_side, ItemTypePoint, 0, "Point");
          item_point->setIndex(k);
          Point point;
          point.x = json_point["x"].asDouble();
          point.y = json_point["y"].asDouble();
          point.z = json_point["z"].asDouble();
          item_point->setPoint(point);
        }

        // right
        QTreeMapItem *item_right_side = new QTreeMapItem(
              item_road, ItemTypeRoadSide, Road::RIGHT, "Right Side");
        item_right_side->setIndex(0);

        Json::Value json_right_side = json_roads[j]["right"];
        const int size_right_points = json_right_side.size();
        for (int k = 0; k < size_right_points; ++k) {
          Json::Value &json_point = json_right_side[k];
          QTreeMapItem *item_point = new QTreeMapItem(
                item_right_side, ItemTypePoint, 0, "Point");
          item_point->setIndex(k);
          Point point;
          point.x = json_point["x"].asDouble();
          point.y = json_point["y"].asDouble();
          point.z = json_point["z"].asDouble();
          item_point->setPoint(point);
        }
      }
      else if (item_segment->mapType() == RoadSegment::SQUARE) {
        QTreeMapItem *item_reference = new QTreeMapItem(
              item_road, ItemTypeRoadSide, Road::OUTLINE, "Outline");
        item_reference->setIndex(0);

        Json::Value json_reference = json_roads[j]["reference"];
        const int size_points = json_reference.size();
        for (int k = 0; k < size_points; ++k) {
          Json::Value &json_point = json_reference[k];
          QTreeMapItem *item_point = new QTreeMapItem(
                item_reference, ItemTypePoint, 0, "Point");
          item_point->setIndex(k);
          Point point;
          point.x = json_point["x"].asDouble();
          point.y = json_point["y"].asDouble();
          point.z = json_point["z"].asDouble();
          item_point->setPoint(point);
        }
      }
    }
  }

  // traffic light
  Json::Value &json_traffic_lights = root["light"];
  const int size_traffic_lights = json_traffic_lights.size();
  for (int i = 0; i < size_traffic_lights; ++i) {
    QTreeMapItem *item_light = new QTreeMapItem(
          root_item_, ItemTypeTrafficLight, 0, "Traffic Light");
    item_light->setIndex(i);

    QTreeMapItem *item_point = new QTreeMapItem(
          item_light, ItemTypePoint, 0, "Point");
    item_point->setIndex(0);
    Point point;
    point.x = json_traffic_lights[i]["x"].asDouble();
    point.y = json_traffic_lights[i]["y"].asDouble();
    point.z = json_traffic_lights[i]["z"].asDouble();
    item_point->setPoint(point);
  }

  // stop line
  Json::Value &json_stop_lines = root["stop_line"];
  const int size_stop_lines = json_stop_lines.size();
  for (int i = 0; i < size_stop_lines; ++i) {
    QTreeMapItem *item_stop_lines = new QTreeMapItem(
          root_item_, ItemTypeStopLines, 0, "Stop Line");
    item_stop_lines->setText(0,  QString("Stop Line %1").arg(i + 1));

    QTreeMapItem *item_point = new QTreeMapItem(
          item_stop_lines, ItemTypePoint, 0, "Point");
    item_point->setIndex(0);
    Point point;
    point.x = json_stop_lines[i]["x"].asDouble();
    point.y = json_stop_lines[i]["y"].asDouble();
    point.z = json_stop_lines[i]["z"].asDouble();
    item_point->setPoint(point);
  }

  // cross
  Json::Value &json_crossings = root["crossing"];
  const int size_crossings = json_crossings.size();
  for (int i = 0; i < size_crossings; ++i) {
     QTreeMapItem *item_crossing = new QTreeMapItem(
          root_item_, ItemTypeCrossings, 0, "Crossing");
    item_crossing->setIndex(i);

    const int size_points = json_crossings[i].size();
    for (int j = 0; j < size_points; ++j) {
      Json::Value &json_point = json_crossings[i][j];
      QTreeMapItem *item_point = new QTreeMapItem(
            item_crossing, ItemTypePoint, 0, "Point");
      item_point->setIndex(j);
      Point point;
      point.x = json_point["x"].asDouble();
      point.y = json_point["y"].asDouble();
      point.z = json_point["z"].asDouble();
      item_point->setPoint(point);
    }
  }

  // markings
  Json::Value &json_markings = root["marking"];
  const int size_marking = json_markings.size();
  for (int i = 0; i < size_marking; ++i) {
    QTreeMapItem *item_marking = new QTreeMapItem(
          root_item_, ItemTypeMarkings, 0, "Marking");
    item_marking->setIndex(i);

    QTreeMapItem *item_point = new QTreeMapItem(
          item_marking, ItemTypePoint, 0, "Point");
    item_point->setIndex(0);
    Point point;
    point.x = json_stop_lines[i]["x"].asDouble();
    point.y = json_stop_lines[i]["y"].asDouble();
    point.z = json_stop_lines[i]["z"].asDouble();
    item_point->setPoint(point);
  }

  // signs
  Json::Value &json_signs = root["sign"];
  const int size_signs = json_signs.size();
  for (int i = 0; i < size_signs; ++i) {
    QTreeMapItem *item_sign = new QTreeMapItem(
          root_item_, ItemTypeSigns, 0, "Sign");
    item_sign->setIndex(i);

    QTreeMapItem *item_point = new QTreeMapItem(
          item_sign, ItemTypePoint, 0, "Point");
    item_point->setIndex(0);
    Point point;
    point.x = json_stop_lines[i]["x"].asDouble();
    point.y = json_stop_lines[i]["y"].asDouble();
    point.z = json_stop_lines[i]["z"].asDouble();
    item_point->setPoint(point);
  }
  tree_hdmap_->expandAll();
  this->updateHdMap();

  return true;
}

void QHdMapWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const float TREE_H_PF = 0.8;
  tree_hdmap_->setGeometry(0, 0, WIDTH, HEIGHT * TREE_H_PF);
  point_val_widget_->setGeometry(
        0, HEIGHT * TREE_H_PF, WIDTH, HEIGHT * (1.0f - TREE_H_PF));
}

int QHdMapWidget::childCount(int type)
{
  int size_child = 0;
  const int size_item = root_item_->childCount();
  for (int i = 0; i < size_item; ++i) {
    if (root_item_->child(i)->type() == type) {
      ++ size_child;
    }
  }
  return size_child;
}

void QHdMapWidget::createTreeMenu()
{
  // menu map
  menu_map_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("添加路段"), menu_map_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onAddRoadSegment);
  menu_map_->addAction(action);

  action = new QAction(tr("添加红绿灯"), menu_map_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onAddTrafficLight);
  menu_map_->addAction(action);

  action = new QAction(tr("人行横道"), menu_map_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onAddCrossing);
  menu_map_->addAction(action);

  // menu item
  menu_item_ = new QMenu(tree_hdmap_);

  action_add_load_ = new QAction(tr("添加路径"), menu_item_);
  connect(action_add_load_, &QAction::triggered, this, &QHdMapWidget::onAddRoad);
  menu_item_->addAction(action_add_load_);

  action_delete_ = new QAction(tr("删除"), menu_item_);
  connect(action_delete_, &QAction::triggered, this, &QHdMapWidget::onDeleteCurrentItem);
  menu_item_->addAction(action_delete_);

  action_delete_all_ = new QAction(tr("清空所有点"), menu_item_);
  connect(action_delete_all_, &QAction::triggered, this, &QHdMapWidget::onDeleteAllItems);
  menu_item_->addAction(action_delete_all_);

  // menu point
  menu_points_ = new QMenu(tree_hdmap_);

  action = new QAction(tr("删除"), menu_points_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteCurrentItem);
  menu_points_->addAction(action);

  action = new QAction(tr("上升"), menu_points_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onUpPoint);
  menu_points_->addAction(action);

  action = new QAction(tr("下降"), menu_points_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDownPoint);
  menu_points_->addAction(action);
}

void QHdMapWidget::onCustomContextMenuRequested(const QPoint &pos)
{
  QTreeWidgetItem *item = tree_hdmap_->currentItem();
  switch (item->type()) {
    case ItemTypeSegment:
      action_add_load_->setEnabled(true);
      action_delete_->setEnabled(true);
      action_delete_all_->setEnabled(false);
      menu_item_->exec(mapToGlobal(pos));
      break;
    case ItemTypeRoad:
      action_add_load_->setEnabled(false);
      action_delete_->setEnabled(true);
      action_delete_all_->setEnabled(false);
      menu_item_->exec(mapToGlobal(pos));
      break;
      break;
    case ItemTypeRoadSide:
      action_add_load_->setEnabled(false);
      action_delete_->setEnabled(false);
      action_delete_all_->setEnabled(true);
      menu_item_->exec(mapToGlobal(pos));
      break;
    case ItemTypeTrafficLight:
    case ItemTypeCrossings:
      action_add_load_->setEnabled(false);
      action_delete_->setEnabled(true);
      action_delete_all_->setEnabled(true);
      menu_item_->exec(mapToGlobal(pos));
      break;
    case ItemTypePoint:
      menu_points_->exec(mapToGlobal(pos));
      break;
    case ItemTypeMap:
      menu_map_->exec(mapToGlobal(pos));
      break;
    default:
      break;
  }
}

void QHdMapWidget::onAddRoadSegment()
{
  QAddSegmentDialog dlg;
  dlg.setWindowTitle("Road Segment");
  if (dlg.exec() == QDialog::Accepted) {
    int type = dlg.type();
    QTreeWidgetItem *parent = tree_hdmap_->currentItem();
    if (RoadSegment::SQUARE == type) {
      if (this->childCount(ItemTypeSegment) == 0) {
        QTreeMapItem *item = new QTreeMapItem(
              parent, ItemTypeSegment, RoadSegment::SQUARE, "Square");
        item->setIndex(0);

        QTreeMapItem *item_child = new QTreeMapItem(
              item, ItemTypeRoad, 0, "Outline");
        item_child->setIndex(0);

        tree_hdmap_->expandItem(parent);
        tree_hdmap_->expandItem(item);
      }
    }
    else {
      int index = this->childCount(ItemTypeSegment);
      QTreeMapItem *item = new QTreeMapItem(
            parent, ItemTypeSegment, RoadSegment::ROAD, "Road Segment");
      item->setIndex(index);
      tree_hdmap_->expandItem(parent);
    }
  }
}

/**
 * @brief 添加路径
 */
void QHdMapWidget::onAddRoad()
{
  // return if type of segment is square
  QTreeMapItem *parent = dynamic_cast<QTreeMapItem *>(tree_hdmap_->currentItem());
  if (parent == nullptr || parent->mapType() == RoadSegment::SQUARE) {
    return;
  }

  // add road
  QTreeMapItem *item = new QTreeMapItem(
        parent, ItemTypeRoad, 0, "Road");
  item->setIndex(parent->childCount() - 1);

  // add left side and right side
  QTreeMapItem *childLeft = new QTreeMapItem(
        item, ItemTypeRoadSide, Road::LEFT, "Left Side");
  childLeft->setIndex(0);

  QTreeMapItem *childRight = new QTreeMapItem(
        item, ItemTypeRoadSide, Road::RIGHT, "Right Side");
  childRight->setIndex(0);

  // set current and expand
  tree_hdmap_->setCurrentItem(item);
  item->setExpanded(true);
}

void QHdMapWidget::onAddPoint(const Point &point)
{
  QTreeWidgetItem *item = tree_hdmap_->currentItem();
  if (item == nullptr) {
    return;
  }

  int type = item->type();
  if (type == ItemTypePoint) { // replace
    QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(item);
    item_point->setPoint(point);
  }
  else if (type == ItemTypeRoadSide || type ==  ItemTypeCrossings) {  // add
    int size_point = item->childCount();
    QTreeMapItem *item_point = new QTreeMapItem(
          item, ItemTypePoint, 0, "Point");
    item_point->setIndex(size_point);
    item_point->setPoint(point);
  }
  else {  // replace or add
    QTreeMapItem *item_point = nullptr;
    if (item->childCount() == 0) {
      QTreeMapItem *item_point = new QTreeMapItem(
            item, ItemTypePoint, 0, "Point");
      item_point->setIndex(0);
    }
    else {
      item_point = dynamic_cast<QTreeMapItem *>(item->child(0));
    }
    item_point->setPoint(point);
  }

  this->updateHdMap();
  item->setExpanded(true);
}

void QHdMapWidget::onAddTrafficLight()
{
  int index = this->childCount(ItemTypeTrafficLight);
  QTreeWidgetItem *parent = tree_hdmap_->currentItem();
  QTreeMapItem *item = new QTreeMapItem(
        parent, ItemTypeTrafficLight, 0, "Traffic Light");
  item->setIndex(index);
  tree_hdmap_->expandItem(parent);
}

void QHdMapWidget::onAddCrossing()
{
  int index = this->childCount(ItemTypeCrossings);
  QTreeWidgetItem *parent = tree_hdmap_->currentItem();
  QTreeMapItem *item = new QTreeMapItem(
        parent, ItemTypeCrossings, 0, "Crossing");
  item->setIndex(index);
  tree_hdmap_->expandItem(parent);
}

/**
 * @brief 删除当前节点
 */
void QHdMapWidget::onDeleteCurrentItem()
{
  // current item and parent
  QTreeWidgetItem *item = tree_hdmap_->currentItem();
  if (item == nullptr) {
    return;
  }
  QTreeWidgetItem *parent = item->parent();
  if (parent == nullptr) {
    return;
  }

  // check child
  if (this->checkHasChildren(item)) {
    return;
  }

  // reset inde and size
  QTreeMapItem *item_map = dynamic_cast<QTreeMapItem *>(item);
  if (item_map == nullptr) {
    return;
  }
  const int size_item = parent->childCount();
  for (int i = 0; i < size_item; ++i) {
    QTreeMapItem *item_child = dynamic_cast<QTreeMapItem *>(parent->child(i));
    if (item_child == nullptr) {
      continue;
    }
    if (item_child->type() == item->type() && item_child->index() > item_map->index()) {
      item_child->setIndex(item_child->index() - 1);
    }
  }

  // remove select item and set current item
  parent->removeChild(item);
  tree_hdmap_->setCurrentItem(parent);

  this->updateHdMap();
}

bool QHdMapWidget::checkHasChildren(QTreeWidgetItem *item)
{
  int size_child = 0;
  if (item->type() == ItemTypeRoad) {
    const int size_road = item->childCount();
    for (int i = 0; i < size_road; ++i) {
      size_child += item->child(i)->childCount();
    }
  }
  else {
    size_child = item->childCount();
  }
  if (size_child > 0) {
    QMessageBox::warning(
          this, QStringLiteral("删除"),
          QStringLiteral("请先清空所有子节点"),
          QMessageBox::Cancel);
    return true;
  }
  return false;
}

/**
 * @brief 删除所有子节点
 */
void QHdMapWidget::onDeleteAllItems()
{
  QTreeWidgetItem *item = tree_hdmap_->currentItem();
  if (item == nullptr) {
    return;
  }
  int ret = QMessageBox::warning(
        this, QStringLiteral("清空"),
        QStringLiteral("是否清空所有点？"),
        QMessageBox::Ok | QMessageBox::Cancel);
  if (ret == QMessageBox::Cancel) {
    return;
  }

  const int size_child = item->childCount();
  for (int i = size_child - 1; i >= 0; --i) {
    item->removeChild(item->child(i));
  }

  this->updateHdMap();
}

void QHdMapWidget::onUpPoint()
{
  this->updateHdMap();
}

void QHdMapWidget::onDownPoint()
{
  this->updateHdMap();
}

void QHdMapWidget::onItemSelectionChanged()
{
  QTreeWidgetItem *item = tree_hdmap_->currentItem();
  if (item == nullptr || item->type() != ItemTypePoint) {
    return;
  }

  QTreeMapItem *itemPoint = dynamic_cast<QTreeMapItem *>(item);
  if (itemPoint != nullptr) {
    const auto &point = itemPoint->point();
    point_val_widget_->setPoint(point);
  }
}

void QHdMapWidget::onSavePointValue()
{
  QTreeWidgetItem *item = tree_hdmap_->currentItem();
  if (item == nullptr || item->type() != ItemTypePoint) {
    return;
  }
  QTreeMapItem *itemPoint = dynamic_cast<QTreeMapItem *>(item);
  if (itemPoint != nullptr) {
    bool ok;
    const auto &point = point_val_widget_->getPoint(ok);
    if (ok) {
      itemPoint->setPoint(point);

      this->updateHdMap();
    }
  }
}

void QHdMapWidget::updateHdMap()
{
  HdMap hdmap;
  const int size_item = root_item_->childCount();
  for (int i = 0; i < size_item; ++i) {
    // item
    QTreeWidgetItem *item = root_item_->child(i);
    int type = item->type();

    if (type == ItemTypeSegment) {
      // segment
      RoadSegment segment;
      QTreeMapItem *item_segment = dynamic_cast<QTreeMapItem *>(item);
      segment.type = item_segment->type();

      // road
      const int size_road = item_segment->childCount();
      int index_road = 0;
      for (int j = 0; j < size_road; ++j) {
        Road road;
        QTreeMapItem *item_road = dynamic_cast<QTreeMapItem *>(item_segment->child(j));

        // road side
        const int size_road_side = item_road->childCount();
        for (int k = 0; k < size_road_side; ++k) {
          QTreeMapItem *item_road_side = dynamic_cast<QTreeMapItem *>(
                item_road->child(k));

          // point
          const int size_point = item_road_side->childCount();
          for (int l = 0; l < size_point; ++l) {
            QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(
                  item_road_side->child(l));
            Point point = item_point->point();
            if (item_road_side->mapType() == Road::LEFT) {
              road.left_side.push_back(point);
            }
            else if (item_road_side->mapType() == Road::RIGHT) {
              road.right_side.push_back(point);
            }
            else if (item_road_side->mapType() == Road::CENTRAL) {
              road.reference.push_back(point);
            }
          }
        }

        if ((road.left_side.size() + road.right_side.size() + road.reference.size()) > 0) {
          segment.roads[index_road ++] = road;
        }
      }

      // push segment
      if (segment.roads.size() > 0) {
        hdmap.road_segments.push_back(segment);
      }
    }
    else if (type == ItemTypeTrafficLight) {
      TrafficLight light;
      QTreeMapItem *item_light = dynamic_cast<QTreeMapItem *>(item);
      if (item_light != nullptr && item_light->childCount() == 1) {
        QTreeMapItem *item_point = dynamic_cast<QTreeMapItem *>(item_light->child(0));
        if (item_point != nullptr) {
          light.type = item_light->type();
          light.point = item_point->point();

          hdmap.traffic_lights.push_back(light);
        }
      }
    }
  }

  QCloudPoints::instance().setHdMap(hdmap);
}
