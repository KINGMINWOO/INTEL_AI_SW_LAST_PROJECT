#ifndef TAB1_BUTTON_H
#define TAB1_BUTTON_H

#include <QWidget>
#include <QTimer>
#include <QDateTime>

namespace Ui {
class Tab1_button;
}

class Tab1_button : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1_button(QWidget *parent = nullptr);
    ~Tab1_button();

private slots:
    void updateDateTime();

signals:
    void goToTab2();
    void goToTab3();
    void goToTab4();

private:
    Ui::Tab1_button *ui;
    QTimer *timer;
};

#endif // TAB1_BUTTON_H
