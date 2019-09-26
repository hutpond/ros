#include <QTreeWidget>
#include <QHeaderView>
#include <QMenu>
#include <QContextMenuEvent>
#include "QProjectManagerWidget.h"
#include "QProjectObject.h"

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

  QAction *action = new QAction(tr("Add Lane"), menu);
  connect(action, &QAction::triggered, this, &QProjectManagerWidget::onActionAddLane);
  menu->addAction(action);
  menu->addSeparator();

  menu->exec(e->globalPos());
  delete menu;
}

void QProjectManagerWidget::doUpdate()
{
  m_pTreeWidget->clear();
  QTreeWidgetItem *itemRoot = new QTreeWidgetItem(m_pTreeWidget);
  itemRoot->setText(0, "Map");

  apollo::hdmap::Map &map = m_pObjProject->mapData();

  auto size_lane = map.lane_size();
  for (int i = 0; i < size_lane; ++i) {
    const auto &lane = map.lane(i);

    QTreeWidgetItem *item = new QTreeWidgetItem(itemRoot);
    item->setText(0, QString::fromStdString(lane.id().id()));
    m_pTreeWidget->expand(m_pTreeWidget->model()->index(i, 0));
  }
}

void QProjectManagerWidget::onActionAddLane()
{
  apollo::hdmap::Map &map = m_pObjProject->mapData();
  apollo::hdmap::Lane *lane = map.add_lane();
  auto size_lane = map.lane_size();
  lane->mutable_id()->set_id("lane_" + std::to_string(size_lane));
  lane->mutable_central_curve()->add_segment();

  this->doUpdate();
}
