#ifndef TAB1_CONTROL_H
#define TAB1_CONTROL_H

#include <QWidget>
#include <QTimer>
#include <QDateTime>

namespace Ui {
class Tab1_control;
}

class Tab1_control : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1_control(QWidget *parent = nullptr);
    ~Tab1_control();

private slots:
    void updateDateTime();   // 시간/날짜 업데이트 함수
    void on_pPBtemp_clicked();  // pPBtemp 클릭 슬롯

    void on_pPBillu_clicked();

    void on_pPBhumi_clicked();

private:
    Ui::Tab1_control *ui;
    QTimer *timer;           // 주기적으로 업데이트할 타이머
};

#endif // TAB1_CONTROL_H
