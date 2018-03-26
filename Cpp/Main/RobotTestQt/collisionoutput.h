#ifndef COLLISIONOUTPUT_H
#define COLLISIONOUTPUT_H

#include <QDialog>
#include <string>

namespace Ui {
class CollisionOutput;
}

class CollisionOutput : public QDialog
{
    Q_OBJECT
    
public:
    explicit CollisionOutput(QWidget *parent = 0);
    ~CollisionOutput();
    
    void SetValue(const std::string& value);
public slots:
    void SetFormat(int fmt);
private:
    Ui::CollisionOutput *ui;
    QString rob_text,urdf_text;
};

#endif // COLLISIONOUTPUT_H
