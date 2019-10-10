#include <iostream>
#include <QTreeWidget>
#include <QHeaderView>
#include <QMenu>
#include <QContextMenuEvent>
#include "QProjectManagerWidget.h"
#include "QProjectObject.h"
#include "QAddElementDialog.h"

#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"

QProjectManagerWidget::QProjectManagerWidget(QProjectObject *obj, QWidget *parent)
  : m_pObjProject(obj)
  , QWidget(parent)
{
  this->setContextMenuPolicy(Qt::DefaultContextMenu);
  m_pTreeWidget = new QTreeWidget(this);
  m_pTreeWidget->header()->hide();
  m_pTreeWidget->setItemsExpandable(true);
}

void QProjectManagerWidget::resizeEvent(QResizeEvent *)
{
  m_pTreeWidget->setGeometry(this->rect());
}


void QProjectManagerWidget::contextMenuEvent(QContextMenuEvent *e)
{
  QMenu *menu = new QMenu();

  // lane
  QMenu *menu_lane = menu->addMenu(tr("Lane"));
  QAction *action = new QAction(tr("Add Lane"), menu_lane);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddLane);
  menu_lane->addAction(action);

  action = new QAction(tr("Add Boundary"), menu_lane);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddBoundary);
  menu_lane->addAction(action);

  menu->exec(e->globalPos());
  delete menu;
}

void QProjectManagerWidget::doUpdate()
{
  m_pTreeWidget->clear();
  QTreeWidgetItem *itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "map");

  apollo::hdmap::Map &map = m_pObjProject->mapData();

  // lane
  auto size_lane = map.lane_size();
  QTreeWidgetItem *itemLane = new QTreeWidgetItem(itemRoot);
  itemLane->setText(0, "lane");
  for (int i = 0; i < size_lane; ++i)
  {
    const auto &lane = map.lane(i);

    // lane
    QTreeWidgetItem *item = new QTreeWidgetItem(itemLane);
    item->setText(0, QString::fromStdString(lane.id().id()));

    // central curve
    const int size_segment = lane.central_curve().segment_size();
    QTreeWidgetItem *item_child;
    if (size_segment > 0) {
      item_child = new QTreeWidgetItem(item);
      item_child->setText(0, "central_curve");
    }
    for (int j = 0; j < size_segment; ++j) {
      QTreeWidgetItem *item_segment = new QTreeWidgetItem(item_child);
      item_segment->setText(0, QString("segment %1").arg(j + 1));
    }

    // left boundary
    const int size_left_boundary = lane.left_boundary().curve().segment_size();
    if (size_left_boundary > 0) {
      item_child = new QTreeWidgetItem(item);
      item_child->setText(0, "left_boundary");
    }
    for (int j = 0; j < size_left_boundary; ++j) {
      QTreeWidgetItem *item_segment = new QTreeWidgetItem(item_child);
      item_segment->setText(0, QString("segment %1").arg(j + 1));
    }

    // right boundary
    const int size_right_boundary = lane.right_boundary().curve().segment_size();
    if (size_right_boundary > 0) {
      item_child = new QTreeWidgetItem(item);
      item_child->setText(0, "right_boundary");
    }
    for (int j = 0; j < size_right_boundary; ++j) {
      QTreeWidgetItem *item_segment = new QTreeWidgetItem(item_child);
      item_segment->setText(0, QString("segment %1").arg(j + 1));
    }
  }

  m_pTreeWidget->expandAll();
}

void QProjectManagerWidget::onActionAddLane()
{
  QAddLaneDialog dlg;
  QRect rect(0, 0, 350, 120);
  moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }
  int type = dlg.type();
  int direction = dlg.direction();

  apollo::hdmap::Map &map = m_pObjProject->mapData();
  const int size_lane = map.lane_size();
  if ( (type == QAddLaneDialog::Central && size_lane > 0) ||
       (type != QAddLaneDialog::Central && size_lane == 0) ) {
    return;
  }

  if (type == QAddLaneDialog::Central) {
    apollo::hdmap::Lane *lane = map.add_lane();
    lane->mutable_id()->set_id("lane_central");
    lane->mutable_central_curve()->add_segment();

    lane->mutable_left_boundary()->mutable_curve()->add_segment();
    lane->mutable_right_boundary()->mutable_curve()->add_segment();
  }
  else if (type == QAddLaneDialog::Left) {
    auto lane_reference = map.mutable_lane(0);

    apollo::hdmap::Lane *lane = map.add_lane();
    decltype(lane->mutable_id()) id;
    if (direction == QAddLaneDialog::Forward) {
      id = lane_reference->add_left_neighbor_forward_lane_id();
      const int size = lane_reference->left_neighbor_forward_lane_id_size();
      id->set_id("lane_left_forward_" + std::to_string(size));
    }
    else {
      id = lane_reference->add_left_neighbor_reverse_lane_id();
      const int size = lane_reference->left_neighbor_reverse_lane_id_size();
      id->set_id("lane_left_reverse_" + std::to_string(size));
    }
    lane->mutable_id()->Swap(id);

    lane->mutable_left_boundary()->mutable_curve()->add_segment();
    lane->mutable_right_boundary()->mutable_curve()->add_segment();
  }
  else {
    auto lane_reference = map.mutable_lane(0);

    apollo::hdmap::Lane *lane = map.add_lane();
    decltype(lane->mutable_id()) id;
    if (direction == QAddLaneDialog::Forward) {
      id = lane_reference->add_right_neighbor_forward_lane_id();
      const int size = lane_reference->right_neighbor_forward_lane_id_size();
      id->set_id("lane_right_forward_" + std::to_string(size));
    }
    else {
      id = lane_reference->add_right_neighbor_reverse_lane_id();
      const int size = lane_reference->right_neighbor_reverse_lane_id_size();
      id->set_id("lane_right_reverse_" + std::to_string(size));
    }
    lane->mutable_id()->Swap(id);

    lane->mutable_left_boundary()->mutable_curve()->add_segment();
    lane->mutable_right_boundary()->mutable_curve()->add_segment();

    lane->mutable_left_boundary()->mutable_curve()->add_segment();
    lane->mutable_right_boundary()->mutable_curve()->add_segment();
  }

  this->doUpdate();
}

void QProjectManagerWidget::onActionAddBoundary()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddBoundaryDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }
  int index_lane, index_segment, index_side;
  dlg.getIndex(index_lane, index_segment, index_side);

}

