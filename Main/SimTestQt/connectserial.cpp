#include "connectserial.h"
#include "ui_connectserial.h"

ConnectSerial::ConnectSerial(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectSerial)
{
    ui->setupUi(this);
}

ConnectSerial::~ConnectSerial()
{
    delete ui;
}

void ConnectSerial::accepted()
{
    QDialog::accepted();
}
