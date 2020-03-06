#ifndef QADDSEGMENTDIALOG_H
#define QADDSEGMENTDIALOG_H

#include <QDialog>

namespace Ui {
class QAddSegmentDialog;
}

class QAddSegmentDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QAddSegmentDialog(QWidget *parent = nullptr);
  ~QAddSegmentDialog();
  int type();

private:
  Ui::QAddSegmentDialog *ui;
};

#endif // QADDSEGMENTDIALOG_H
