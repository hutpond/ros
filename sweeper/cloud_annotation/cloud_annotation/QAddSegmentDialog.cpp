#include "QAddSegmentDialog.h"
#include "ui_QAddSegmentDialog.h"

QAddSegmentDialog::QAddSegmentDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::QAddSegmentDialog)
{
  ui->setupUi(this);
}

QAddSegmentDialog::~QAddSegmentDialog()
{
  delete ui;
}

int QAddSegmentDialog::type()
{
  ui->comboBox->currentIndex();
}
