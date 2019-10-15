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
  : QWidget(parent)
  , m_pObjProject(obj)
{
  this->setContextMenuPolicy(Qt::DefaultContextMenu);
  m_pTreeWidget = new QTreeWidget(this);
  m_pTreeWidget->header()->hide();
  m_pTreeWidget->setItemsExpandable(true);
}

QSize QProjectManagerWidget::sizeHint() const
{
  QSize size = this->size();
  size.setWidth(250);
  return size;
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

  // signal
  QMenu *menu_signal = menu->addMenu(tr("Signal"));
  action = new QAction(tr("Add SignalSign"), menu_signal);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddSignalSign);
  menu_signal->addAction(action);

  action = new QAction(tr("Add Crosswalk"), menu_signal);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddCrosswalk);
  menu_signal->addAction(action);

  action = new QAction(tr("Add StopSign"), menu_signal);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddStopSign);
  menu_signal->addAction(action);

  action = new QAction(tr("Add YieldSign"), menu_signal);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddYieldSign);
  menu_signal->addAction(action);

  action = new QAction(tr("Add ClearArea"), menu_signal);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddClearArea);
  menu_signal->addAction(action);

  action = new QAction(tr("Add SpeedBump"), menu_signal);
  connect(action, &QAction::triggered,
          this, &QProjectManagerWidget::onActionAddSpeedBump);
  menu_signal->addAction(action);

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
  apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddLaneDialog dlg(map, this);
  QRect rect(0, 0, 350, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }
  int type = dlg.type();
  int direction = dlg.direction();

  const int size_lane = map.lane_size();
  if ( (type == static_cast<int>(LaneSide::Central) && size_lane > 0) ||
       (type != static_cast<int>(LaneSide::Central) && size_lane == 0) ) {
    return;
  }

  if (type == static_cast<int>(LaneSide::Central)) {
    apollo::hdmap::Lane *lane = map.add_lane();
    lane->mutable_id()->set_id("lane_central");
    lane->mutable_central_curve()->add_segment();

    lane->mutable_left_boundary()->mutable_curve()->add_segment();
    lane->mutable_right_boundary()->mutable_curve()->add_segment();
  }
  else if (type == static_cast<int>(LaneSide::Left)) {
    auto lane_reference = map.mutable_lane(0);

    apollo::hdmap::Lane *lane = map.add_lane();
    decltype(lane->mutable_id()) id;
    if (direction == static_cast<int>(LaneDirection::Forward)) {
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
    if (direction == static_cast<int>(LaneDirection::Forward)) {
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
  }

  this->doUpdate();
}

void QProjectManagerWidget::onActionAddBoundary()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddBoundaryDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }
  int index_lane, index_segment, index_side;
  dlg.getIndex(index_lane, index_segment, index_side);

  emit addBoundary(index_lane, index_segment, index_side);
}

void QProjectManagerWidget::onActionAddSignalSign()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddSignalSignDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }

  emit operateSignal(MapOperation::SignalSign);
}

void QProjectManagerWidget::onActionAddCrosswalk()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddCrosswalkDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }

  emit operateSignal(MapOperation::Crosswalk);
}

void QProjectManagerWidget::onActionAddStopSign()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddStopSignDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }

  emit operateSignal(MapOperation::StopSign);
}

void QProjectManagerWidget::onActionAddYieldSign()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddYieldSignDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }

  emit operateSignal(MapOperation::YieldSign);
}

void QProjectManagerWidget::onActionAddClearArea()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddClearAreaDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }

  emit operateSignal(MapOperation::ClearArea);
}

void QProjectManagerWidget::onActionAddSpeedBump()
{
  const apollo::hdmap::Map &map = m_pObjProject->mapData();
  QAddSpeedBumpDialog dlg(map);
  QRect rect(0, 0, 400, 120);
  dlg.moveRectToCenter(rect);
  dlg.setGeometry(rect);
  if (dlg.exec() == QDialog::Rejected) {
    return;
  }

  emit operateSignal(MapOperation::SpeedBump);
}
