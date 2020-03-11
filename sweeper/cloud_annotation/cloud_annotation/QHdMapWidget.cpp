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
  , road_segment_size_(0)
  , traffic_light_size_(0)
{
  tree_hdmap_ = new QTreeWidget(this);
  tree_hdmap_->setHeaderHidden(true);
  tree_hdmap_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(tree_hdmap_, &QTreeWidget::customContextMenuRequested,
          this, &QHdMapWidget::onCustomContextMenuRequested);
  root_item_ = new QTreeWidgetItem(tree_hdmap_, ItemTypeMap);
  root_item_->setText(0, "map");

  this->createMenuMap();
  this->createMenuSegment();
  this->createMenuDelete();
  this->createMenuRoadSide();
  this->createMenuPoints();

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
      QTreeSegmentItem *item_segment = dynamic_cast<QTreeSegmentItem *>(item);
      json_segment["type"] = item_segment->segmentType();

      // road
      Json::Value json_roads;
      const int size_road = item_segment->childCount();
      for (int j = 0; j < size_road; ++j) {
        Json::Value json_road;
        QTreeRoadItem *item_road = dynamic_cast<QTreeRoadItem *>(item_segment->child(j));

        // road side
        Json::Value json_left_side, json_right_side, json_reference;
        const int size_road_side = item_road->childCount();
        for (int k = 0; k < size_road_side; ++k) {
          QTreeRoadSideItem *item_road_side = dynamic_cast<QTreeRoadSideItem *>(
                item_road->child(k));

          // point
          const int size_point = item_road_side->childCount();
          for (int l = 0; l < size_point; ++l) {
            QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(
                  item_road_side->child(l));
            Json::Value json_point;
            Point point = item_point->point();
            json_point["x"] = point.x;
            json_point["y"] = point.y;
            json_point["z"] = point.z;
            if (item_road_side->roadtype() == Road::LEFT) {
              json_left_side.append(json_point);
            }
            else if (item_road_side->roadtype() == Road::RIGHT) {
              json_right_side.append(json_point);
            }
            else if (item_road_side->roadtype() == Road::CENTRAL) {
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
      QTreeTrafficLightItem *item_light = dynamic_cast<QTreeTrafficLightItem *>(item);
      if (item_light != nullptr && item_light->childCount() == 1) {
        QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item_light->child(0));
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
        QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item_stop_line->child(0));
        if (item_point != nullptr) {
          Json::Value json_stop_line;
          Point point = item_point->point();
          json_stop_line["x"] = point.x;
          json_stop_line["y"] = point.y;
          json_stop_line["z"] = point.z;
          json_stop_line["type"] = item_stop_line->mapType();

          json_stop_lines.append(json_stop_line);
        }
      }
    }
    else if (type == ItemTypeCrossings) {
      QTreeMapItem *item_crossing = dynamic_cast<QTreeMapItem *>(item);
      if (item_crossing != nullptr && item_crossing->childCount() == 1) {
        QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item_crossing->child(0));
        if (item_point != nullptr) {
          Json::Value json_crossing;
          Point point = item_point->point();
          json_crossing["x"] = point.x;
          json_crossing["y"] = point.y;
          json_crossing["z"] = point.z;
          json_crossing["type"] = item_crossing->mapType();

          json_crossings.append(json_crossing);
        }
      }
    }
    else if (type == ItemTypeMarkings) {
      QTreeMapItem *item_marking = dynamic_cast<QTreeMapItem *>(item);
      if (item_marking != nullptr && item_marking->childCount() == 1) {
        QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item_marking->child(0));
        if (item_point != nullptr) {
          Json::Value json_marking;
          Point point = item_point->point();
          json_marking["x"] = point.x;
          json_marking["y"] = point.y;
          json_marking["z"] = point.z;
          json_marking["type"] = item_marking->mapType();

          json_markings.append(json_marking);
        }
      }
    }
    else if (type == ItemTypeSigns) {
      QTreeMapItem *item_sign = dynamic_cast<QTreeMapItem *>(item);
      if (item_sign != nullptr && item_sign->childCount() == 1) {
        QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item_sign->child(0));
        if (item_point != nullptr) {
          Json::Value json_sign;
          Point point = item_point->point();
          json_sign["x"] = point.x;
          json_sign["y"] = point.y;
          json_sign["z"] = point.z;
          json_sign["type"] = item_sign->mapType();

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
  road_segment_size_ = size_segments;
  for (int i = 0; i < size_segments; ++i) {
    Json::Value &json_roads = json_segments[i]["road"];
    QTreeSegmentItem *item_segment = new QTreeSegmentItem(root_item_);
    item_segment->setSegmentType(json_segments[i]["type"].asInt());
    item_segment->setText(0, QString("Road Segment %1").arg(i + 1));

    // roads
    const int size_roads = json_roads.size();
    for (int j = 0; j < size_roads; ++j) {
      item_segment->addRoadSize();
      QTreeRoadItem *item_road = new QTreeRoadItem(item_segment);
      item_road->setText(0, QString("Road %1").arg(j + 1));
      item_road->setRoadIndex(j);
      if (item_segment->segmentType() == RoadSegment::ROAD) {
        // left
        QTreeRoadSideItem *item_left_side = new QTreeRoadSideItem(item_road);
        item_left_side->setRoadType(Road::LEFT);
        item_left_side->setText(0, QString("Left Side"));

        Json::Value &json_left_side = json_roads[j]["left"];
        const int size_left_points = json_left_side.size();
        for (int k = 0; k < size_left_points; ++k) {
          Json::Value &json_point = json_left_side[k];
          QTreePointItem *item_point = new QTreePointItem(item_left_side);
          item_point->setText(0, QString("Point %1").arg(k + 1));
          item_point->setIndex(k);
          Point point;
          point.x = json_point["x"].asDouble();
          point.y = json_point["y"].asDouble();
          point.z = json_point["z"].asDouble();
          item_point->setPoint(point);
        }

        // right
        QTreeRoadSideItem *item_right_side = new QTreeRoadSideItem(item_road);
        item_right_side->setRoadType(Road::RIGHT);
        item_right_side->setText(0, QString("Right Side"));

        Json::Value json_right_side = json_roads[j]["right"];
        const int size_right_points = json_right_side.size();
        for (int k = 0; k < size_right_points; ++k) {
          Json::Value &json_point = json_right_side[k];
          QTreePointItem *item_point = new QTreePointItem(item_right_side);
          item_point->setText(0, QString("Point %1").arg(k + 1));
          item_point->setIndex(k);
          Point point;
          point.x = json_point["x"].asDouble();
          point.y = json_point["y"].asDouble();
          point.z = json_point["z"].asDouble();
          item_point->setPoint(point);
        }
      }
      else if (item_segment->segmentType() == RoadSegment::SQUARE) {
        QTreeRoadSideItem *item_reference = new QTreeRoadSideItem(item_road);
        item_reference->setRoadType(Road::OUTLINE);
        item_reference->setText(0, QString("Outline"));

        Json::Value json_reference = json_roads[j]["reference"];
        const int size_points = json_reference.size();
        for (int k = 0; k < size_points; ++k) {
          Json::Value &json_point = json_reference[k];
          QTreePointItem *item_point = new QTreePointItem(item_reference);
          item_point->setText(0, QString("Point %1").arg(k + 1));
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
  traffic_light_size_ = size_traffic_lights;
  for (int i = 0; i < size_traffic_lights; ++i) {
    QTreeTrafficLightItem *item_light = new QTreeTrafficLightItem(root_item_);
    item_light->setText(0,  QString("Traffic Light %1").arg(i + 1));

    QTreePointItem *item_point = new QTreePointItem(item_light);
    item_point->setText(0, QString("Point"));
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
  stop_lines_size_ = size_stop_lines;
  for (int i = 0; i < size_stop_lines; ++i) {
    QTreeMapItem *item_stop_lines = new QTreeMapItem(root_item_, ItemTypeStopLines);
    item_stop_lines->setText(0,  QString("Stop Line %1").arg(i + 1));

    QTreePointItem *item_point = new QTreePointItem(item_stop_lines);
    item_point->setText(0, QString("Point"));
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
  crossings_size_ = size_crossings;
  for (int i = 0; i < size_crossings; ++i) {
    QTreeMapItem *item_crossing = new QTreeMapItem(root_item_, ItemTypeCrossings);
    item_crossing->setText(0,  QString("Crossing %1").arg(i + 1));

    QTreePointItem *item_point = new QTreePointItem(item_crossing);
    item_point->setText(0, QString("Point"));
    item_point->setIndex(0);
    Point point;
    point.x = json_stop_lines[i]["x"].asDouble();
    point.y = json_stop_lines[i]["y"].asDouble();
    point.z = json_stop_lines[i]["z"].asDouble();
    item_point->setPoint(point);
  }

  // markings
  Json::Value &json_markings = root["marking"];
  const int size_marking = json_markings.size();
  markings_size_ = size_marking;
  for (int i = 0; i < size_marking; ++i) {
    QTreeMapItem *item_marking = new QTreeMapItem(root_item_, ItemTypeMarkings);
    item_marking->setText(0,  QString("Marking %1").arg(i + 1));

    QTreePointItem *item_point = new QTreePointItem(item_marking);
    item_point->setText(0, QString("Point"));
    item_point->setIndex(0);
    Point point;
    point.x = json_stop_lines[i]["x"].asDouble();
    point.y = json_stop_lines[i]["y"].asDouble();
    point.z = json_stop_lines[i]["z"].asDouble();
    item_point->setPoint(point);
  }

  // markings
  Json::Value &json_signs = root["sign"];
  const int size_signs = json_signs.size();
  signs_size_ = size_signs;
  for (int i = 0; i < size_signs; ++i) {
    QTreeMapItem *item_sign = new QTreeMapItem(root_item_, ItemTypeSigns);
    item_sign->setText(0,  QString("Sign %1").arg(i + 1));

    QTreePointItem *item_point = new QTreePointItem(item_sign);
    item_point->setText(0, QString("Point"));
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

void QHdMapWidget::createMenuMap()
{
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
}

void QHdMapWidget::createMenuSegment()
{
  menu_segment_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("添加路径"), menu_segment_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onAddRoad);
  menu_segment_->addAction(action);

  action = new QAction(tr("删除"), menu_segment_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteCurrentItem);
  menu_segment_->addAction(action);
}

/**
 * @brief 右键菜单，通用，只有“删除”操作
 */
void QHdMapWidget::createMenuDelete()
{
  menu_delete_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("删除"), menu_delete_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteCurrentItem);
  menu_delete_->addAction(action);
}

/**
 * @brief 右键菜单，清空点
 */
void QHdMapWidget::createMenuRoadSide()
{
  // road side
  menu_road_side_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("清空所有点"), menu_road_side_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteAllItems);
  menu_road_side_->addAction(action);

  // crossing
  menu_item_with_points_ = new QMenu(tree_hdmap_);

  action = new QAction(tr("删除"), menu_item_with_points_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteCurrentItem);
  menu_item_with_points_->addAction(action);

  action = new QAction(tr("清空所有点"), menu_item_with_points_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteAllItems);
  menu_item_with_points_->addAction(action);

}

void QHdMapWidget::createMenuPoints()
{
  menu_points_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("删除"), menu_points_);
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
    {
      QTreeSegmentItem *item_current = dynamic_cast<QTreeSegmentItem *>(item);
      if (item_current != nullptr && item_current->segmentType() == RoadSegment::ROAD) {
        menu_segment_->exec(mapToGlobal(pos));
      }
      break;
    }
    case ItemTypeTrafficLight:
      menu_delete_->exec(mapToGlobal(pos));
      break;
    case ItemTypeCrossings:
      menu_item_with_points_->exec(mapToGlobal(pos));
    case ItemTypeRoad:
    {
      QTreeRoadItem *item_current = dynamic_cast<QTreeRoadItem *>(item);
      if (item_current != nullptr) {
        QTreeSegmentItem *item_parent = dynamic_cast<QTreeSegmentItem *>(
              item_current->parent());
        if (item_parent != nullptr && item_parent->segmentType() == RoadSegment::ROAD) {
          menu_delete_->exec(mapToGlobal(pos));
        }
      }
      break;
    }
    case ItemTypeRoadSide:
      menu_road_side_->exec(mapToGlobal(pos));
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
      if (road_segment_size_ == 0) {
        QTreeSegmentItem *item = new QTreeSegmentItem(parent);
        item->setSegmentType(type);
        item->setIndex(road_segment_size_ ++);
        item->setText(0, QString("Square"));

        QTreeRoadItem *item_child = new QTreeRoadItem(item);
        item_child->setRoadIndex(0);
        item_child->setText(0, QStringLiteral("Outline"));

        tree_hdmap_->expandItem(parent);
        tree_hdmap_->expandItem(item);
      }
    }
    else {
      QTreeSegmentItem *item = new QTreeSegmentItem(parent);
      item->setSegmentType(type);
      item->setIndex(road_segment_size_ ++);
      item->setText(0, QString("Road Segment %1").arg(road_segment_size_));
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
  QTreeSegmentItem *parent = dynamic_cast<QTreeSegmentItem *>(tree_hdmap_->currentItem());
  if (parent == nullptr || parent->segmentType() == RoadSegment::SQUARE) {
    return;
  }

  // add road
  QTreeRoadItem *item = new QTreeRoadItem(parent);
  item->setRoadIndex(parent->roadSize());
  parent->addRoadSize();
  item->setText(0, QString("Road %1").arg(parent->roadSize()));

  // add left side and right side
  QTreeRoadSideItem *childLeft = new QTreeRoadSideItem(item);
  childLeft->setRoadType(Road::LEFT);
  childLeft->setText(0, QString("Left Side"));
  QTreeRoadSideItem *childRight = new QTreeRoadSideItem(item);
  childRight->setRoadType(Road::RIGHT);
  childRight->setText(0, QString("Right Side"));

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
  if (type == ItemTypeRoadSide || type ==  ItemTypeCrossings) {
    int size_point = item->childCount();
    QTreePointItem *itemChild = new QTreePointItem(item);
    itemChild->setIndex(size_point);
    itemChild->setPoint(point);
    itemChild->setText(0, QString("Point %1").arg(size_point + 1));

    this->updateHdMap();
  }
  else if (type == ItemTypeTrafficLight) {
    QTreePointItem *itemChild = nullptr;
    if (item->childCount() == 0) {
      itemChild = new QTreePointItem(item);
      itemChild->setText(0, QString("Point"));
    }
    else {
      itemChild = dynamic_cast<QTreePointItem *>(item->child(0));
    }
    if (itemChild != nullptr) {
      itemChild->setPoint(point);

      this->updateHdMap();
    }
  }
  else if (type == ItemTypePoint) {
    QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item);
    if (item_point != nullptr) {
      item_point->setPoint(point);

      this->updateHdMap();
    }
  }
  item->setExpanded(true);
}

void QHdMapWidget::onAddTrafficLight()
{
  QTreeWidgetItem *parent = tree_hdmap_->currentItem();
  QTreeTrafficLightItem *item = new QTreeTrafficLightItem(parent);
  item->setLightIndex(traffic_light_size_ ++);
  item->setText(0, QString("Traffic Light %1").arg(traffic_light_size_));
  tree_hdmap_->expandItem(parent);
}

void QHdMapWidget::onAddCrossing()
{
  QTreeWidgetItem *parent = tree_hdmap_->currentItem();
  QTreeCrossingItem *item = new QTreeCrossingItem(parent);
  item->setIndex(crossings_size_ ++);
  item->setText(0, QString("Crossing %1").arg(crossings_size_));
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

  // reset inde and size
  bool ret = false;
  int type = item->type();
  if (type == ItemTypeSegment) {
    ret = this->deleteRoadSegment(item, parent);
  }
  else if (type == ItemTypeRoad) {
    ret = this->deleteRoad(item, parent);
  }
  else if (type == ItemTypeTrafficLight) {
    traffic_light_size_ = 0;
    ret = this->deleteSingleItem(item, parent);
  }
  else if (type == ItemTypeCrossings) {
    ret = this->deleteCrossing(item, parent);
  }
  else if (item->type() == ItemTypePoint) {
    ret = this->deletePoint(item, parent);
  }

  // remove select item and set current item
  if (ret) {
    parent->removeChild(item);
    tree_hdmap_->setCurrentItem(parent);

    this->updateHdMap();
  }
}

bool QHdMapWidget::deleteRoadSegment(QTreeWidgetItem *item, QTreeWidgetItem *parent)
{
  // return if size of childs is not zero
  if (item->childCount() > 0) {
    QMessageBox::warning(
          this, QStringLiteral("删除"),
          QStringLiteral("请先清空所有子节点"),
          QMessageBox::Cancel);
    return false;
  }

  // reset index and size
  const int size_item = parent->childCount();
  road_segment_size_ = 0;
  for (int i = 0; i < size_item; ++i) {
    QTreeWidgetItem *child = parent->child(i);
    if (child == item || child->type() != item->type()) {
      continue;
    }

    QTreeSegmentItem *childSegment = dynamic_cast<QTreeSegmentItem *>(child);
    if (childSegment == nullptr) {
      continue;
    }
    childSegment->setIndex(road_segment_size_ ++);
    childSegment->setText(0, QString("Road Segment %1").arg(road_segment_size_));
  }

  return true;
}

bool QHdMapWidget::deleteRoad(QTreeWidgetItem *item, QTreeWidgetItem *parent)
{
  // return if size of childs is not zero
  const int size_child = item->childCount();
  bool hasChild = false;
  int size_points = 0;
  for (int i = 0; i < size_child; ++i) {
    QTreeWidgetItem *child = item->child(i);
    size_points += child->childCount();
  }
  hasChild = (size_points > 0);
  if (hasChild) {
    QMessageBox::warning(
          this, QStringLiteral("删除"),
          QStringLiteral("请先清空所有子节点"),
          QMessageBox::Cancel);
    return false;
  }

  // reset index and size
  QTreeSegmentItem *parentSegment = dynamic_cast<QTreeSegmentItem *>(parent);
  parentSegment->reduceRoadSize();
  int road_size = 0;
  const int size_item = parent->childCount();
  for (int i = 0; i < size_item; ++i) {
    QTreeWidgetItem *child = parent->child(i);
    if (child == item || child->type() != item->type()) {
      continue;
    }

    QTreeRoadItem *childRoad = dynamic_cast<QTreeRoadItem *>(child);
    if (childRoad == nullptr) {
      continue;
    }
    childRoad->setRoadIndex(road_size ++);
    childRoad->setText(0, QString("Road %1").arg(road_size));
  }

  return true;
}

/**
 * @brief 删除单节点
 * @param item
 * @param parent
 * @return
 */
bool QHdMapWidget::deleteSingleItem(QTreeWidgetItem *item, QTreeWidgetItem *parent)
{
  // reset index and size
  const int size_item = parent->childCount();
  for (int i = 0; i < size_item; ++i) {
    QTreeWidgetItem *child = parent->child(i);
    if (child == item || child->type() != item->type()) {
      continue;
    }

    if (child->type() == ItemTypeTrafficLight) {
      QTreeTrafficLightItem *childLight = dynamic_cast<QTreeTrafficLightItem *>(child);
      if (childLight == nullptr) {
        continue;
      }
      childLight->setLightIndex(traffic_light_size_ ++);
      childLight->setText(0, QString("Traffic Light %1").arg(traffic_light_size_));
    }
  }

  return true;
}

bool QHdMapWidget::deletePoint(QTreeWidgetItem *item, QTreeWidgetItem *parent)
{
  // reset index and size
  int index = 0;
  const int size_item = parent->childCount();
  for (int i = 0; i < size_item; ++i) {
    QTreeWidgetItem *child = parent->child(i);
    if (child == item || child->type() != item->type()) {
      continue;
    }

    QTreePointItem *childPoint = dynamic_cast<QTreePointItem *>(child);
    if (childPoint == nullptr) {
      continue;
    }
    childPoint->setIndex(index ++);
    childPoint->setText(0, QString("Point %1").arg(index));
  }

  return true;
}

bool QHdMapWidget::deleteCrossing(QTreeWidgetItem *item, QTreeWidgetItem *parent)
{
  // return if size of childs is not zero
  if (this->checkHasChildren(item)) {
    return false;
  }

  // reset index and size
  const int size_item = parent->childCount();
  crossings_size_ = 0;
  for (int i = 0; i < size_item; ++i) {
    QTreeWidgetItem *child = parent->child(i);
    if (child == item || child->type() != item->type()) {
      continue;
    }

    QTreeCrossingItem *childCrossing = dynamic_cast<QTreeCrossingItem *>(child);
    if (childCrossing == nullptr) {
      continue;
    }
    childCrossing->setIndex(crossings_size_ ++);
    childCrossing->setText(0, QString("Crossing %1").arg(crossings_size_));
  }

  return true;

}

bool QHdMapWidget::checkHasChildren(QTreeWidgetItem *item)
{
  // return if size of childs is not zero
  if (item->childCount() > 0) {
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
  if (ret == QDialog::Rejected) {
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

  QTreePointItem *itemPoint = dynamic_cast<QTreePointItem *>(item);
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
  QTreePointItem *itemPoint = dynamic_cast<QTreePointItem *>(item);
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
      QTreeSegmentItem *item_segment = dynamic_cast<QTreeSegmentItem *>(item);
      segment.type = item_segment->type();

      // road
      const int size_road = item_segment->childCount();
      int index_road = 0;
      for (int j = 0; j < size_road; ++j) {
        Road road;
        QTreeRoadItem *item_road = dynamic_cast<QTreeRoadItem *>(item_segment->child(j));

        // road side
        const int size_road_side = item_road->childCount();
        for (int k = 0; k < size_road_side; ++k) {
          QTreeRoadSideItem *item_road_side = dynamic_cast<QTreeRoadSideItem *>(
                item_road->child(k));

          // point
          const int size_point = item_road_side->childCount();
          for (int l = 0; l < size_point; ++l) {
            QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(
                  item_road_side->child(l));
            Point point = item_point->point();
            if (item_road_side->roadtype() == Road::LEFT) {
              road.left_side.push_back(point);
            }
            else if (item_road_side->roadtype() == Road::RIGHT) {
              road.right_side.push_back(point);
            }
            else if (item_road_side->roadtype() == Road::CENTRAL) {
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
      QTreeTrafficLightItem *item_light = dynamic_cast<QTreeTrafficLightItem *>(item);
      if (item_light != nullptr && item_light->childCount() == 1) {
        QTreePointItem *item_point = dynamic_cast<QTreePointItem *>(item_light->child(0));
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
