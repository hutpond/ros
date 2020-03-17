#ifndef QDISPLAYAREADIALOG_H
#define QDISPLAYAREADIALOG_H

#include <QDialog>

namespace Ui {
class QDisplayAreaDialog;
}

class QDisplayAreaDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QDisplayAreaDialog(QWidget *parent = nullptr);
  ~QDisplayAreaDialog();

protected:
  void onBtnResetEast();
  void onBtnResetNorth();
  void onBtnResetUp();
  void onBtnSetting();

private:
  Ui::QDisplayAreaDialog *ui;
};

#endif // QDISPLAYAREADIALOG_H
