#ifndef PLAYRESOURCEFRAME_H
#define PLAYRESOURCEFRAME_H

#include <QFrame>

namespace Ui {
class PlayResourceFrame;
}

class PlayResourceFrame : public QFrame
{
    Q_OBJECT
    
public:
    explicit PlayResourceFrame(QWidget *parent = 0);
    ~PlayResourceFrame();
public slots:
    void Play();
    void Pause();
    void Record();
private:
    Ui::PlayResourceFrame *ui;
};

#endif // PLAYRESOURCEFRAME_H
