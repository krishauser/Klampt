#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QFileDialog>

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
    QFileDialog f;
public slots:
    void findIn();
    void findOut();
    void findPrefix();
    void findRoot();
    void execute();

private:
    Ui::Dialog *ui;
};

#endif // DIALOG_H
