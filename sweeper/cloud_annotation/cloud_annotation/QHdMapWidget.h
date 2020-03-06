#ifndef QHDMAPWIDGET_H
#define QHDMAPWIDGET_H

#include <QWidget>
#include <QTreeWidgetItem>

#include "GlobalDefine.h"

class QTreeWidget;
class QMenu;

enum TreeItemType{
  ItemTypeSegment = 1,
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
    return -1;
  }

private:
  int segment_type_;
  int segment_index_;
  int central_size_;
  int left_size_;
  int right_size_;
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

  void setPoint(const Point &point) {
    point_ = point;
  }
  Point point() {
    return point_;
  }

private:
  Point point_;
};

class QHdMapWidget : public QWidget
{
  Q_OBJECT

public:
  explicit QHdMapWidget(QWidget *parent = nullptr);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  void createMenuMap();
  void createMenuSegment();

protected:
  void onCustomContextMenuRequested(const QPoint &);
  void onAddRoadSegment();
  void onAddRoad();

private:
  QTreeWidget *tree_hdmap_;
  int road_segment_size_;
  QMenu *menu_map_;
  QMenu *menu_segment_;
};

#endif // QHDMAPWIDGET_H
