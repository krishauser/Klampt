#ifndef SHOWTEXT_H
#define SHOWTEXT_H

#include <QDialog>

namespace Ui {
class ShowText;
}

class ShowText : public QDialog
{
  Q_OBJECT
  
public:
  QString rob;
  explicit ShowText(QWidget *parent = 0);
  ~ShowText();
  
  void SetText(QString str);
public slots:
  void copyText();
  void appendText();
private:
  Ui::ShowText *ui;
};

#endif // SHOWTEXT_H
