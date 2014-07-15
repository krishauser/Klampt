#ifndef COLLISIONOUTPUT_H
#define COLLISIONOUTPUT_H

#include <QDialog>

namespace Ui {
class CollisionOutput;
}

class CollisionOutput : public QDialog
{
    Q_OBJECT
    
public:
    explicit CollisionOutput(QWidget *parent = 0);
    ~CollisionOutput();
    
    void SetText(QString text);
private:
    Ui::CollisionOutput *ui;
};

#endif // COLLISIONOUTPUT_H
