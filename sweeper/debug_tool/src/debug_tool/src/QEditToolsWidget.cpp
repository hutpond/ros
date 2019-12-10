#include "QEditToolsWidget.h"

#include <QPushButton>

static const char *SELECT_BKG = "background-color: rgb(114, 159, 207);";

QEditToolsWidget::QEditToolsWidget(QWidget *parent)
  : QWidget(parent)
  , m_nBtnIndex(Move)
{
  for (int i = 0; i < Count; ++i) {
    m_pBtnTool[i] = new QPushButton(this);
    m_pBtnTool[i]->setFocusPolicy(Qt::NoFocus);
    connect(m_pBtnTool[i], &QPushButton::clicked, this, &QEditToolsWidget::onBtnClicked);
    m_bFlagCheckable[i] = true;
  }
  m_pBtnTool[Move]->setText("M");
  m_pBtnTool[Move]->setToolTip("Move");
  m_pBtnTool[Move]->setStyleSheet(SELECT_BKG);
  m_pBtnTool[Target]->setText("T");
  m_pBtnTool[Target]->setToolTip("Create Target");
  m_pBtnTool[Garbage]->setText("G");
  m_pBtnTool[Garbage]->setToolTip("Create Garbage");
  m_pBtnTool[Save]->setText("S");
  m_pBtnTool[Save]->setToolTip("Save Change to File");
  m_bFlagCheckable[Save] = false;
}

void QEditToolsWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();

  const int SPACE = 3;
  const int ITEM_W = (WIDTH - 3 * SPACE) / 2;
  const int ITEM_H = ITEM_W;

  int pos_x = SPACE;
  int pos_y = SPACE;
  for (int i = 0; i < Count; i += 2) {
    pos_x = SPACE;
    m_pBtnTool[i]->setGeometry(pos_x, pos_y, ITEM_W, ITEM_H);

    if (i + 1 == Count) {
      break;
    }
    pos_x += ITEM_W + SPACE;
    m_pBtnTool[i + 1]->setGeometry(pos_x, pos_y, ITEM_W, ITEM_H);

    pos_y += ITEM_W + SPACE;
  }
}

void QEditToolsWidget::onBtnClicked()
{
  int index = -1;
  QObject *sender = this->sender();
  for (int i = 0; i < Count; ++i) {
    if (sender == m_pBtnTool[i]) {
      index = i;
      break;
    }
  }
  if (index != -1 && index != m_nBtnIndex) {
    if (m_bFlagCheckable[index]) {
      m_pBtnTool[m_nBtnIndex]->setStyleSheet("background-color: lightGray;");
      m_nBtnIndex = index;
      m_pBtnTool[m_nBtnIndex]->setStyleSheet(SELECT_BKG);
    }
    emit selectTool(index, m_bFlagCheckable[index]);
  }
}
