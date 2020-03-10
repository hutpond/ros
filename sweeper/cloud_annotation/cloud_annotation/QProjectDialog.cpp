#include "QProjectDialog.h"
#include "ui_QProjectDialog.h"

#include <QDir>
#include <QFileDialog>

QProjectDialog::QProjectDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QProjectDialog)
{
  ui->setupUi(this);
  ui->lineEdit_2->setReadOnly(true);
  connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onBtnBrowse()));
  connect(ui->lineEdit, SIGNAL(textChanged(const QString &)),
          this, SLOT(onTextChanged(const QString &)));

  ui->lineEdit_2->setText(getenv("HOME"));
  ui->lineEdit->setText(this->getProjectName());
}

QProjectDialog::~QProjectDialog()
{
  delete ui;
}

QString QProjectDialog::projectPath()
{
  QString pathName = ui->lineEdit_2->text();
  if (!pathName.endsWith('/')) {
    pathName.append('/');
  }
  return pathName;
}

QString QProjectDialog::projectName()
{
  return ui->lineEdit->text();
}

QString QProjectDialog::getProjectName()
{
  QString pathName = ui->lineEdit_2->text();
  if (!pathName.endsWith('/')) {
    pathName.append('/');
  }
  QString projectName;
  int index = 1;
  do {
    projectName = QString("HdMapProject_%1").arg(index ++);
  } while (QDir(pathName + projectName).exists());

  return projectName;
}

void QProjectDialog::onBtnBrowse()
{
  QString pathName = QFileDialog::getExistingDirectory(
        this, tr("Open Directory"),
        getenv("HOME"),
        QFileDialog::ShowDirsOnly
        | QFileDialog::DontResolveSymlinks);
  if (!pathName.isEmpty()) {
    ui->lineEdit_2->setText(pathName);
    if (!pathName.endsWith('/')) {
      pathName.append('/');
    }
    pathName += ui->lineEdit->text();
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(
          !QDir(pathName).exists());
  }
}

void QProjectDialog::onTextChanged(const QString &name)
{
  QString pathName = ui->lineEdit_2->text();
  if (!pathName.endsWith('/')) {
    pathName.append('/');
  }
  pathName += name;
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(
        !QDir(pathName).exists());
}
