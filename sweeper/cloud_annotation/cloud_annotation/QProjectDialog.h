#ifndef QPROJECTDIALOG_H
#define QPROJECTDIALOG_H

#include <QDialog>

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

protected:
  QString getProjectName();

protected slots:
  void onBtnBrowse();
  void onTextChanged(const QString &);

private:
  Ui::QProjectDialog *ui;
};

#endif // QPROJECTDIALOG_H
