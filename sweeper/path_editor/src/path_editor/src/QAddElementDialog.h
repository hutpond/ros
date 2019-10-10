#ifndef QADDELEMENTDIALOG_H
#define QADDELEMENTDIALOG_H

#include <QDialog>

class QComboBox;

class QAddElementDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QAddElementDialog(QWidget * = Q_NULLPTR);

protected:
  QPushButton *m_pBtnOk;
  QPushButton *m_pBtnCancel;
};

class QAddLaneDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  enum
  {
    Central,
    Left,
    Right
  };

  enum
  {
    Forward,
    Reverse
  };

public:
  explicit QAddLaneDialog(QWidget * = Q_NULLPTR);

  int type();
  int direction();

private:
  QComboBox *m_pCmbType;
  QComboBox *m_pCmbDirection;
};


namespace apollo {
namespace hdmap {
  class Map;
}
}
class QAddBoundaryDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  enum
  {
    Left,
    Right
  };

public:
  explicit QAddBoundaryDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
  void getIndex(int &, int &, int &);

protected slots:
  void onLaneAndSideChanged();

private:
  const apollo::hdmap::Map &m_rMap;

  QComboBox *m_pCmbLane;
  QComboBox *m_pCmbSide;
  QComboBox  *m_pCmbSegment;
};

void moveRectToCenter(QRect &);

#endif // QADDELEMENTDIALOG_H
