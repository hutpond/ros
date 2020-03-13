#ifndef QWAITINGDIALOG_H
#define QWAITINGDIALOG_H

#include <QDialog>

namespace Ui {
class QWaitingDialog;
}

class QWaitingDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QWaitingDialog(QWidget *parent = nullptr);
  ~QWaitingDialog();

private:
  Ui::QWaitingDialog *ui;
};

#endif // QWAITINGDIALOG_H
