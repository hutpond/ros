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
  QTreeMapItem(QTreeWidgetItem *parent, int type, int map_type, const QString &text)
    : QTreeWidgetItem(parent, type)
    , index_(0)
    , map_type_(map_type)
    , text_(text) {
  }

  void setIndex(int index) {
    index_ = index;
    this->setText(0, QString("%1 %2").arg(text_).arg(index_ + 1));
  }
  int index() {
    return index_;
  }
  int mapType() {
    return map_type_;
  }

  void setPoint(const Point &point) {
    point_ =  point;
  }
  Point point() {
    return point_;
  }

private:
  int index_;
  int map_type_;
  QString text_;
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
  void updateHdMap();

public slots:
  void onAddPoint(const Point &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  int childCount(int);
  void createTreeMenu();

protected:
  void onCustomContextMenuRequested(const QPoint &);
  void onAddRoadSegment();
  void onAddRoad();

  void onAddTrafficLight();
  void onAddCrossing();

  void onDeleteCurrentItem();
  void onDeleteAllItems();

  void onUpPoint();
  void onDownPoint();

  bool checkHasChildren(QTreeWidgetItem *);

protected slots:
  void onItemSelectionChanged();
  void onSavePointValue();

private:
  QTreeWidget *tree_hdmap_;
  QTreeWidgetItem *root_item_;

  QMenu *menu_map_;
  QMenu *menu_item_;
  QMenu *menu_points_;

  QAction *action_add_load_;
  QAction *action_delete_;
  QAction *action_delete_all_;

  QPointValue *point_val_widget_;
};

#endif // QHDMAPWIDGET_H
