#ifndef QHDMAPWIDGET_H
#define QHDMAPWIDGET_H

#include <QWidget>
#include <QTreeWidgetItem>

#include "GlobalDefine.h"

class QTreeWidget;
class QMenu;
class QPointValue;

enum TreeItemType{
  ItemTypeSegment = 1,
  ItemTypeTrafficLight,
  ItemTypeRoad,
  ItemTypeRoadSide,
  ItemTypePoint,
  ItemTypeMap
};

class QTreeSegmentItem : public QTreeWidgetItem
{
public:
  QTreeSegmentItem(QTreeWidgetItem *parent)
    : QTreeWidgetItem(parent, ItemTypeSegment) {
    left_size_ = 0;
    right_size_ = 0;
    central_size_ = 0;
    road_size_ = 0;
  }

  void setIndex(int index) {
    segment_index_ = index;
  }
  int index() {
    return segment_index_;
  }
  void setSegmentType(int type) {
    segment_type_ = type;
  }
  int segmentType() {
    return segment_type_;
  }
  void addRoadSize(int type) {
    if (type == Road::LEFT) {
      ++ left_size_;
    }
    else if (type == Road::RIGHT) {
      ++ right_size_;
    }
    else if (type == Road::CENTRAL) {
      central_size_ = 1;
    }
    else if (type == Road::ALL) {
      ++ road_size_;
    }
  }
  void reduceRoadSize(int type) {
    if (type == Road::LEFT) {
      -- left_size_;
    }
    else if (type == Road::RIGHT) {
      -- right_size_;
    }
    else if (type == Road::CENTRAL) {
      central_size_ = 0;
    }
    else if (type == Road::ALL) {
      -- road_size_;
    }
  }
  int roadSize(int type) {
    if (type == Road::LEFT) {
      return left_size_;
    }
    else if (type == Road::RIGHT) {
      return right_size_;
    }
    else if (type == Road::CENTRAL) {
      return central_size_;
    }
    else if (type == Road::ALL) {
      return road_size_;
    }
    return -1;
  }

private:
  int segment_type_;
  int segment_index_;
  int central_size_;
  int left_size_;
  int right_size_;
  int road_size_;
};

class QTreeTrafficLightItem : public QTreeWidgetItem
{
public:
  QTreeTrafficLightItem(QTreeWidgetItem *parent)
    : QTreeWidgetItem(parent, ItemTypeTrafficLight)
  {
  }

  void setLightIndex(int index) {
    index_ = index;
  }
  int lightIndex() {
    return index_;
  }

private:
  int index_;
};

class QTreeRoadItem : public QTreeWidgetItem
{
public:
  QTreeRoadItem(QTreeWidgetItem *parent)
    : QTreeWidgetItem(parent, ItemTypeRoad)
  {
  }

  void setRoadIndex(int index) {
    index_ = index;
  }
  int roadIndex() {
    return index_;
  }

private:
  int index_;
};

class QTreeRoadSideItem : public QTreeWidgetItem
{
public:
  QTreeRoadSideItem(QTreeWidgetItem *parent)
    : QTreeWidgetItem(parent, ItemTypeRoadSide)
  {
  }

  void setRoadType(int type) {
    road_type_ = type;
  }
  int roadtype() {
    return road_type_;
  }

private:
  int road_type_;
};

class QTreePointItem : public QTreeWidgetItem
{
public:
  QTreePointItem(QTreeWidgetItem *parent)
    : QTreeWidgetItem(parent, ItemTypePoint)
  {
  }

  void setIndex(int index) {
    index_ = index;
  }
  int index() {
    return index_;
  }
  void setPoint(const Point &point) {
    point_.x = point.x;
    point_.y = point.y;
    point_.z = point.z;
  }
  Point point() {
    return point_;
  }

private:
  int index_;
  Point point_;
};

class QHdMapWidget : public QWidget
{
  Q_OBJECT

public:
  explicit QHdMapWidget(QWidget *parent = nullptr);

public slots:
  void onAddPoint(const Point &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  void createMenuMap();
  void createMenuSegment();
  void createMenuDelete();
  void createMenuPoints();

  void updateHdMap();

protected:
  void onCustomContextMenuRequested(const QPoint &);
  void onAddRoadSegment();
  void onAddRoad();

  void onAddTrafficLight();

  void onDeleteCurrentItem();
  void onDeleteAllItems();

  bool deleteRoadSegment(QTreeWidgetItem *, QTreeWidgetItem *);
  bool deleteRoad(QTreeWidgetItem *, QTreeWidgetItem *);
  bool deleteSingleItem(QTreeWidgetItem *, QTreeWidgetItem *);

protected slots:
  void onItemSelectionChanged();
  void onSavePointValue();

private:
  QTreeWidget *tree_hdmap_;
  QTreeWidgetItem *root_item_;
  int road_segment_size_;
  int traffic_light_size_;
  QMenu *menu_map_;
  QMenu *menu_segment_;
  QMenu *menu_delete_;
  QMenu *menu_points_;

  QPointValue *point_val_widget_;
};

#endif // QHDMAPWIDGET_H
