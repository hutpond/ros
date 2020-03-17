#include "QDisplayAreaDialog.h"
#include "ui_QDisplayAreaDialog.h"

#include "qcloudpoints.h"

QDisplayAreaDialog::QDisplayAreaDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QDisplayAreaDialog)
{
  ui->setupUi(this);

  auto beg = QCloudPoints::instance().beginPointDisplay();
  auto end = QCloudPoints::instance().endPointDisplay();
  ui->lineEdit_east->setText(QString("%1").arg(beg.x, 0, 'f', 3));
  ui->lineEdit_east_2->setText(QString("%1").arg(end.x, 0, 'f', 3));
  ui->lineEdit_north->setText(QString("%1").arg(beg.y, 0, 'f', 3));
  ui->lineEdit_north_2->setText(QString("%1").arg(end.y, 0, 'f', 3));
  ui->lineEdit_up->setText(QString("%1").arg(beg.z, 0, 'f', 3));
  ui->lineEdit_up_2->setText(QString("%1").arg(end.z, 0, 'f', 3));

  connect(ui->pushButton_east, &QPushButton::clicked,
          this, &QDisplayAreaDialog::onBtnResetEast);
  connect(ui->pushButton_north, &QPushButton::clicked,
          this, &QDisplayAreaDialog::onBtnResetNorth);
  connect(ui->pushButton_up, &QPushButton::clicked,
          this, &QDisplayAreaDialog::onBtnResetUp);
  connect(ui->buttonBox->button(QDialogButtonBox::Ok), &QPushButton::clicked,
          this, &QDisplayAreaDialog::onBtnSetting);
}

QDisplayAreaDialog::~QDisplayAreaDialog()
{
  delete ui;
}

void QDisplayAreaDialog::onBtnResetEast()
{
  auto beg = QCloudPoints::instance().beginPoint();
  auto end = QCloudPoints::instance().endPoint();
  ui->lineEdit_east->setText(QString("%1").arg(beg.x, 0, 'f', 3));
  ui->lineEdit_east_2->setText(QString("%1").arg(end.x, 0, 'f', 3));
}

void QDisplayAreaDialog::onBtnResetNorth()
{
  auto beg = QCloudPoints::instance().beginPoint();
  auto end = QCloudPoints::instance().endPoint();
  ui->lineEdit_north->setText(QString("%1").arg(beg.y, 0, 'f', 3));
  ui->lineEdit_north_2->setText(QString("%1").arg(end.y, 0, 'f', 3));
}

void QDisplayAreaDialog::onBtnResetUp()
{
  auto beg = QCloudPoints::instance().beginPoint();
  auto end = QCloudPoints::instance().endPoint();
  ui->lineEdit_up->setText(QString("%1").arg(beg.z, 0, 'f', 3));
  ui->lineEdit_up_2->setText(QString("%1").arg(end.z, 0, 'f', 3));
}

void QDisplayAreaDialog::onBtnSetting()
{
  pcl::PointXYZ beg, end;
  beg.x = ui->lineEdit_east->text().toDouble();
  beg.y = ui->lineEdit_north->text().toDouble();
  beg.z = ui->lineEdit_up->text().toDouble();

  end.x = ui->lineEdit_east_2->text().toDouble();
  end.y = ui->lineEdit_north_2->text().toDouble();
  end.z = ui->lineEdit_up_2->text().toDouble();

  QCloudPoints::instance().setPointDisplay(beg, end);
}
