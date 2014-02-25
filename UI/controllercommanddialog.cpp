#include "controllercommanddialog.h"
#include "ui_controllercommanddialog.h"

#include <boost/foreach.hpp>

ControllerCommandDialog::ControllerCommandDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ControllerCommandDialog)
{
    ui->setupUi(this);
}

ControllerCommandDialog::~ControllerCommandDialog()
{
    delete ui;
}

void ControllerCommandDialog::Refresh(){
    BOOST_FOREACH(string str,commands){
        ui->comboBox->addItem(QString::fromStdString(str));
    }
}

void ControllerCommandDialog::SendCommand(){
    if(ui->lineEdit->text().isEmpty()) ui->lineEdit->setFocus();
    else emit ControllerCommand(ui->comboBox->currentText().toStdString(),ui->lineEdit->text().toStdString());
}
