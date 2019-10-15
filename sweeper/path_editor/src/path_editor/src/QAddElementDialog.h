#ifndef QADDELEMENTDIALOG_H
#define QADDELEMENTDIALOG_H

#include <QDialog>

class QComboBox;
namespace apollo {
namespace hdmap {
  class Map;
}
}

class QAddElementDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QAddElementDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);

  void moveRectToCenter(QRect &);

protected:
  const apollo::hdmap::Map &m_rMap;

  QPushButton *m_pBtnOk;
  QPushButton *m_pBtnCancel;
};

class QAddLaneDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddLaneDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);

  int type();
  int direction();

private:
  QComboBox *m_pCmbType;
  QComboBox *m_pCmbDirection;
};

class QAddBoundaryDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddBoundaryDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
  void getIndex(int &, int &, int &);

protected slots:
  void onLaneAndSideChanged();

private:
  QComboBox *m_pCmbLane;
  QComboBox *m_pCmbSide;
  QComboBox  *m_pCmbSegment;
};

class QAddSignalSignDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddSignalSignDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
};

class QAddCrosswalkDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddCrosswalkDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
};

class QAddStopSignDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddStopSignDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
};

class QAddYieldSignDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddYieldSignDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
};

class QAddClearAreaDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddClearAreaDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
};

class QAddSpeedBumpDialog : public QAddElementDialog
{
  Q_OBJECT

public:
  explicit QAddSpeedBumpDialog(const apollo::hdmap::Map &, QWidget * = Q_NULLPTR);
};

#endif // QADDELEMENTDIALOG_H
