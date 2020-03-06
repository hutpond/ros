#include "QHdMapWidget.h"

#include <QTreeWidget>
#include <QMenu>

#include <QAddSegmentDialog.h>

QHdMapWidget::QHdMapWidget(QWidget *parent)
  : QWidget(parent)
  , road_segment_size_(0)
{
  tree_hdmap_ = new QTreeWidget(this);
  tree_hdmap_->setHeaderHidden(true);
  tree_hdmap_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(tree_hdmap_, &QTreeWidget::customContextMenuRequested,
          this, &QHdMapWidget::onCustomContextMenuRequested);
  QTreeWidgetItem *root = new QTreeWidgetItem(tree_hdmap_, ItemTypeMap);
  root->setText(0, "map");

  this->createMenuMap();
  this->createMenuSegment();

  this->setMinimumWidth(300);
}

void QHdMapWidget::resizeEvent(QResizeEvent *)
{
  tree_hdmap_->setGeometry(this->rect());
}

void QHdMapWidget::createMenuMap()
{
  menu_map_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("添加路段"), menu_map_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onAddRoadSegment);
  menu_map_->addAction(action);

  action = new QAction(tr("添加红绿灯"), menu_map_);
  menu_map_->addAction(action);
}

void QHdMapWidget::createMenuSegment()
{
  menu_segment_ = new QMenu(tree_hdmap_);

  QAction *action = new QAction(tr("添加路径"), menu_segment_);
  connect(action, &QAction::triggered, this, &QHdMapWidget::onAddRoad);
  menu_segment_->addAction(action);

  action = new QAction(tr("删除"), menu_segment_);
  menu_segment_->addAction(action);
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
    case ItemTypeRoad:

      break;
    case ItemTypeRoadSide:

      break;
    case ItemTypePoint:

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

void QHdMapWidget::onAddRoad()
{
  QTreeSegmentItem *parent = dynamic_cast<QTreeSegmentItem *>(tree_hdmap_->currentItem());
  if (parent == nullptr) {
    return;
  }
}
