#include "QHdMapWidget.h"

#include <QTreeWidget>
#include <QMenu>
#include <QMessageBox>

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
  menu_road_side_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("清空所有点"), menu_points_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onDeleteAllItems);
  menu_road_side_->addAction(action);
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
  item->setRoadIndex(parent->roadSize(Road::ALL));
  parent->addRoadSize(Road::ALL);
  item->setText(0, QString("Road %1").arg(parent->roadSize(Road::ALL)));

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

  if (item->type() == ItemTypeRoadSide) {
    int size_point = item->childCount();
    QTreePointItem *itemChild = new QTreePointItem(item);
    itemChild->setIndex(size_point);
    itemChild->setPoint(point);
    itemChild->setText(0, QString("Point %1").arg(size_point + 1));

    this->updateHdMap();
  }
  else if (item->type() == ItemTypeTrafficLight) {
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
  if (item->type() == ItemTypeSegment) {
    ret = this->deleteRoadSegment(item, parent);
  }
  else if (item->type() == ItemTypeRoad) {
    ret = this->deleteRoad(item, parent);
  }
  else if (item->type() == ItemTypeTrafficLight) {
    traffic_light_size_ = 0;
    ret = this->deleteSingleItem(item, parent);
  }

  // remove select item and set current item
  if (ret) {
    parent->removeChild(item);
    tree_hdmap_->setCurrentItem(parent);
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
  parentSegment->reduceRoadSize(Road::ALL);
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
}

void QHdMapWidget::onUpPoint()
{

}

void QHdMapWidget::onDownPoint()
{

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
