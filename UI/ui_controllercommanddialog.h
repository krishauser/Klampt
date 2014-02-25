/********************************************************************************
** Form generated from reading UI file 'controllercommanddialog.ui'
**
** Created: Sun Feb 23 21:57:52 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROLLERCOMMANDDIALOG_H
#define UI_CONTROLLERCOMMANDDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_ControllerCommandDialog
{
public:
    QGridLayout *gridLayout;
    QLineEdit *lineEdit;
    QComboBox *comboBox;
    QPushButton *pushButton;
    QDialogButtonBox *buttonBox;
    QFrame *line_2;

    void setupUi(QDialog *ControllerCommandDialog)
    {
        if (ControllerCommandDialog->objectName().isEmpty())
            ControllerCommandDialog->setObjectName(QString::fromUtf8("ControllerCommandDialog"));
        ControllerCommandDialog->resize(320, 78);
        gridLayout = new QGridLayout(ControllerCommandDialog);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        lineEdit = new QLineEdit(ControllerCommandDialog);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        gridLayout->addWidget(lineEdit, 0, 1, 1, 1);

        comboBox = new QComboBox(ControllerCommandDialog);
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout->addWidget(comboBox, 0, 0, 1, 1);

        pushButton = new QPushButton(ControllerCommandDialog);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        gridLayout->addWidget(pushButton, 0, 2, 1, 1);

        buttonBox = new QDialogButtonBox(ControllerCommandDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 3, 0, 1, 2);

        line_2 = new QFrame(ControllerCommandDialog);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line_2, 2, 0, 1, 3);


        retranslateUi(ControllerCommandDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ControllerCommandDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ControllerCommandDialog, SLOT(reject()));
        QObject::connect(pushButton, SIGNAL(clicked()), ControllerCommandDialog, SLOT(SendCommand()));

        QMetaObject::connectSlotsByName(ControllerCommandDialog);
    } // setupUi

    void retranslateUi(QDialog *ControllerCommandDialog)
    {
        ControllerCommandDialog->setWindowTitle(QApplication::translate("ControllerCommandDialog", "Send Command", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("ControllerCommandDialog", "Send", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ControllerCommandDialog: public Ui_ControllerCommandDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROLLERCOMMANDDIALOG_H
