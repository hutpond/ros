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
  ui->lineEdit_3->setReadOnly(true);
  ui->lineEdit_7->setReadOnly(true);
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(false);

  connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(onBtnPathBrowse()));
  connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(onBtnPointBrowse()));
  connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(onBtnReferenceBrowse()));

  connect(ui->lineEdit, SIGNAL(textChanged(const QString &)),
          this, SLOT(onTextChanged()));
  connect(ui->lineEdit_4, SIGNAL(textChanged(const QString &)),
          this, SLOT(onTextChanged()));
  connect(ui->lineEdit_5, SIGNAL(textChanged(const QString &)),
          this, SLOT(onTextChanged()));
  connect(ui->lineEdit_6, SIGNAL(textChanged(const QString &)),
          this, SLOT(onTextChanged()));

  QString path = getenv("HOME");
  if (!path.endsWith('/')) {
    path.append('/');
  }
  path += "Documents/HdMap/";
  ui->lineEdit_2->setText(path);
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

QString QProjectDialog::cloudPointName()
{
  return ui->lineEdit_3->text();
}

QString QProjectDialog::referenceName()
{
  return ui->lineEdit_7->text();
}

Point QProjectDialog::cloudPointOrigin(bool *ok)
{
  Point point;
  bool okLon, okLat, okHeight;
  point.x = ui->lineEdit_4->text().toDouble(&okLon);
  point.y = ui->lineEdit_5->text().toDouble(&okLat);
  point.z = ui->lineEdit_6->text().toDouble(&okHeight);

  if (ok != nullptr) {
    *ok = (okLon && okLat && okHeight);
  }

  return point;
}

void QProjectDialog::onBtnPathBrowse()
{
  QString pathName = QFileDialog::getExistingDirectory(
        this, tr("Open Directory"),
        getenv("HOME"),
        QFileDialog::ShowDirsOnly
        | QFileDialog::DontResolveSymlinks);
  if (!pathName.isEmpty()) {
    ui->lineEdit_2->setText(pathName);
  }
  this->checkBtnEnabled();
}

void QProjectDialog::onBtnPointBrowse()
{
  QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Cloud Points File"), getenv("HOME"), tr("Cloud Points Files (*.ply)"));
  if (!fileName.isEmpty()) {
    ui->lineEdit_3->setText(fileName);
  }
  this->checkBtnEnabled();
}

void QProjectDialog::onBtnReferenceBrowse()
{
  QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Cloud Points File"), getenv("HOME"), tr("Cloud Points Files (*.xord)"));
  if (!fileName.isEmpty()) {
    ui->lineEdit_7->setText(fileName);
  }
  this->checkBtnEnabled();
}

void QProjectDialog::onTextChanged()
{
  this->checkBtnEnabled();
}

void QProjectDialog::checkBtnEnabled()
{
  QString pathName = ui->lineEdit_2->text();
  if (!pathName.endsWith('/')) {
    pathName.append('/');
  }
  pathName += ui->lineEdit->text();
  bool ok;
  this->cloudPointOrigin(&ok);

  bool enabled = ( (!QDir(pathName).exists()) &&
                   (!this->cloudPointName().isEmpty()) &&
                   (!this->referenceName().isEmpty()) &&
                   (ok) );
  ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(enabled);
}
