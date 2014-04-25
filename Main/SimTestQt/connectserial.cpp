#include "connectserial.h"
#include "ui_connectserial.h"

#include <QDialogButtonBox>
#include <QPushButton>

ConnectSerial::ConnectSerial(int robots,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectSerial)
{
    ui->setupUi(this);
    SetNumRobots(robots);
    OnTextEdit();
}

ConnectSerial::~ConnectSerial()
{
    delete ui;
}

void ConnectSerial::accept()
{
    emit MakeConnect(ui->cmb_robot->itemData(ui->cmb_robot->currentIndex()).Int,
                     ui->line_host->text(),ui->spn_port->value(),ui->spn_rate->value());
    QDialog::accepted();
}

void ConnectSerial::SetNumRobots(int n){
   ui->cmb_robot->clear();
   for(int i=0;i<n;i++){
       ui->cmb_robot->addItem(QString::number(i),i);
   }
}

void ConnectSerial::OnTextEdit(){
    ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(!ui->line_host->text().isEmpty());
}
