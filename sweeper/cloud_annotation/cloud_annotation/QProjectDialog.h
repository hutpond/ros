#ifndef QPROJECTDIALOG_H
#define QPROJECTDIALOG_H

#include <QDialog>
#include <GlobalDefine.h>

namespace Ui {
class QProjectDialog;
}

class QProjectDialog : public QDialog
{
  Q_OBJECT

public:
  explicit QProjectDialog(QWidget *parent = nullptr);
  ~QProjectDialog();
  QString projectPath();
  QString projectName();
  QString cloudPointName();
  QString referenceName();
  Point cloudPointOrigin(bool * = nullptr);

protected:
  QString getProjectName();
  void checkBtnEnabled();

protected slots:
  void onBtnPathBrowse();
  void onBtnPointBrowse();
  void onBtnReferenceBrowse();
  void onTextChanged();

private:
  Ui::QProjectDialog *ui;
};

#endif // QPROJECTDIALOG_H
