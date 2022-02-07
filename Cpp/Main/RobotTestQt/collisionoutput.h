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
    
    void SetValue(const std::string& value,const std::string& value_with_names);
    void UpdateText();
public slots:
    void SetFormat(int fmt);
    void OnNamesClicked(int state);
private:
    Ui::CollisionOutput *ui;
    QString rob_text,urdf_text;
    QString rob_names_text,urdf_names_text;
};

#endif // COLLISIONOUTPUT_H
