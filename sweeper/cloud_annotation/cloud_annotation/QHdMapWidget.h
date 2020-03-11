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
  ItemTypeStopLines,
  ItemTypeCrossings,
  ItemTypeMarkings,
  ItemTypeSigns,
  ItemTypeRoad,
  ItemTypeRoadSide,
  ItemTypePoint,
  ItemTypeMap
};

class QTreeMapItem : public QTreeWidgetItem
{
public:
  QTreeMapItem(QTreeWidgetItem *parent, int type)
    : QTreeWidgetItem(parent, type) {
    index_ = 0;
  }

  void setIndex(int index) {
    index_ = index;
  }
  int index() {
    return index_;
  }
  void setMapType(int map_type) {
    map_type_ = map_type;
  }
  int mapType() {
    return map_type_;
  }

private:
  int index_;
  int map_type_;
};

class QTreeSegmentItem : public QTreeWidgetItem
{
public:
  QTreeSegmentItem(QTreeWidgetItem *parent)
    : QTreeWidgetItem(parent, ItemTypeSegment) {
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
  void addRoadSize() {
    ++ road_size_;
  }
  void reduceRoadSize() {
    -- road_size_;
  }
  int roadSize() {
    return road_size_;
  }

private:
  int segment_type_;
  int segment_index_;
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
  void clear();
  void saveHdMapData(const QString &);
  bool parseHdMapData(const QString &);

public slots:
  void onAddPoint(const Point &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  void createMenuMap();
  void createMenuSegment();
  void createMenuDelete();
  void createMenuRoadSide();
  void createMenuPoints();

  void updateHdMap();

protected:
  void onCustomContextMenuRequested(const QPoint &);
  void onAddRoadSegment();
  void onAddRoad();

  void onAddTrafficLight();

  void onDeleteCurrentItem();
  void onDeleteAllItems();

  void onUpPoint();
  void onDownPoint();

  bool deleteRoadSegment(QTreeWidgetItem *, QTreeWidgetItem *);
  bool deleteRoad(QTreeWidgetItem *, QTreeWidgetItem *);
  bool deleteSingleItem(QTreeWidgetItem *, QTreeWidgetItem *);
  bool deletePoint(QTreeWidgetItem *, QTreeWidgetItem *);

protected slots:
  void onItemSelectionChanged();
  void onSavePointValue();

private:
  QTreeWidget *tree_hdmap_;
  QTreeWidgetItem *root_item_;
  int road_segment_size_;
  int traffic_light_size_;
  int stop_lines_size_;
  int crossings_size_;
  int markings_size_;
  int signs_size_;

  QMenu *menu_map_;
  QMenu *menu_segment_;
  QMenu *menu_delete_;
  QMenu *menu_road_side_;
  QMenu *menu_points_;

  QPointValue *point_val_widget_;
};

#endif // QHDMAPWIDGET_H
