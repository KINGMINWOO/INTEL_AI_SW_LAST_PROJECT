#ifndef TAB1_BUTTON_H
#define TAB1_BUTTON_H

#include <QWidget>

namespace Ui {
class Tab1_button;
}

class Tab1_button : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1_button(QWidget *parent = nullptr);
    ~Tab1_button();

private:
    Ui::Tab1_button *ui;

signals:
    void goToTab2();
    void goToTab3();
};

#endif // TAB1_BUTTON_H
