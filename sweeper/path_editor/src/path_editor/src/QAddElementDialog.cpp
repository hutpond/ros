#include <QComboBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QGuiApplication>
#include <QScreen>
#include "QAddElementDialog.h"

QAddElementDialog::QAddElementDialog(QWidget *parent)
  : QDialog(parent)
{
  m_pBtnOk = new QPushButton("OK", this);
  m_pBtnCancel = new QPushButton("CANCEL", this);

  connect(m_pBtnOk, &QPushButton::clicked, this, &QAddLaneDialog::accept);
  connect(m_pBtnCancel, &QPushButton::clicked, this, &QAddLaneDialog::reject);
}


QAddLaneDialog::QAddLaneDialog(QWidget *parent)
  : QAddElementDialog(parent)
{
  m_pCmbType = new QComboBox(this);
  m_pCmbType->addItem("Central");
  m_pCmbType->addItem("Left");
  m_pCmbType->addItem("Right");

  m_pCmbDirection = new QComboBox(this);
  m_pCmbDirection->addItem("Forward");
  m_pCmbDirection->addItem("Reverse");

  QVBoxLayout *layout = new QVBoxLayout;
  QHBoxLayout *hlayout = new QHBoxLayout;
  hlayout->addStretch();
  hlayout->addWidget(m_pCmbType);
  hlayout->addStretch();
  hlayout->addWidget(m_pCmbDirection);
  hlayout->addStretch();
  layout->addLayout(hlayout);

  QHBoxLayout *hlayout2 = new QHBoxLayout;
  hlayout2->addStretch();
  hlayout2->addWidget(m_pBtnOk);
  hlayout2->addStretch();
  hlayout2->addWidget(m_pBtnCancel);
  hlayout2->addStretch();
  layout->addLayout(hlayout2);

  this->setLayout(layout);

  this->setWindowTitle("Add Lane");
}

int QAddLaneDialog::type()
{
  return m_pCmbType->currentIndex();
}

int QAddLaneDialog::direction()
{
  return m_pCmbDirection->currentIndex();
}


#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_util.h"
QAddBoundaryDialog::QAddBoundaryDialog(const apollo::hdmap::Map &map, QWidget *parent)
  : m_rMap(map)
  , QAddElementDialog(parent)
{
  m_pCmbLane = new QComboBox(this);
  m_pCmbSegment = new QComboBox(this);
  m_pCmbSide = new QComboBox(this);
  m_pCmbSide->addItem("Left");
  m_pCmbSide->addItem("Right");

  connect(m_pCmbLane, SIGNAL(currentIndexChanged(int)),
          this, SLOT(onLaneAndSideChanged()));
  const int size_lane = m_rMap.lane_size();
  for (int i = 0; i < size_lane; ++i)
  {
    const auto &lane = m_rMap.lane(i);
    m_pCmbLane->addItem(QString::fromStdString(lane.id().id()));
  }

  connect(m_pCmbSide, SIGNAL(currentIndexChanged(int)),
          this, SLOT(onLaneAndSideChanged()));

  if (size_lane > 0) {
    m_pCmbLane->setCurrentIndex(0);
  }

  QVBoxLayout *layout = new QVBoxLayout;
  QHBoxLayout *hlayout = new QHBoxLayout;
  hlayout->addStretch();
  hlayout->addWidget(m_pCmbLane);
  hlayout->addStretch();
  hlayout->addWidget(m_pCmbSegment);
  hlayout->addStretch();
  hlayout->addWidget(m_pCmbSide);
  hlayout->addStretch();
  layout->addLayout(hlayout);

  QHBoxLayout *hlayout2 = new QHBoxLayout;
  hlayout2->addStretch();
  hlayout2->addWidget(m_pBtnOk);
  hlayout2->addStretch();
  hlayout2->addWidget(m_pBtnCancel);
  hlayout2->addStretch();
  layout->addLayout(hlayout2);

  this->setLayout(layout);

  this->setWindowTitle("Add Lane");

}

void QAddBoundaryDialog::getIndex(int &index_lane, int &index_segment, int &index_side)
{
  index_lane = -1;
  index_segment = -1;
  index_side = -1;
  if (m_pCmbLane->count() > 0) {
    index_lane = m_pCmbLane->currentIndex();
  }
  if (m_pCmbSegment->count() > 0) {
    index_segment = m_pCmbSegment->currentIndex();
  }
  if (m_pCmbSide->count() > 0) {
    index_side = m_pCmbSide->currentIndex();
  }
}

void QAddBoundaryDialog::onLaneAndSideChanged()
{
  int index_lane = m_pCmbLane->currentIndex();
  int index_side = m_pCmbSide->currentIndex();
  const auto &lane = m_rMap.lane(index_lane);

  m_pCmbSegment->clear();
  if (index_side == Left) {
    const int size_left_boundary = lane.left_boundary().curve().segment_size();
    for (int j = 0; j < size_left_boundary; ++j) {
      m_pCmbSegment->addItem(QString("segment %1").arg(j + 1));
    }
  }
  else {
    const int size_right_boundary = lane.right_boundary().curve().segment_size();
    for (int j = 0; j < size_right_boundary; ++j) {
      m_pCmbSegment->addItem(QString("segment %1").arg(j + 1));
    }
  }
}


void moveRectToCenter(QRect &rect)
{
  QList<QScreen *> screens = QGuiApplication::screens();
  QRect rectScreen = screens.at(0)->availableGeometry();
  rect.moveCenter(rectScreen.center());
}
