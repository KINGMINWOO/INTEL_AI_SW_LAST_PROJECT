#ifndef TAB1_CONTROL_H
#define TAB1_CONTROL_H

#include <QWidget>
#include <QTimer>
#include <QDateTime>
#include "socketclient.h"

namespace Ui {
class Tab1_control;
}

class Tab1_control : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1_control(QWidget *parent = nullptr);
    ~Tab1_control();

    // ✅ Tab1이 외부에서 소켓을 주입받도록
    void setSocketClient(SocketClient* sock) { pSocketClient = sock; }

private slots:
    void updateDateTime();
    void on_pPBtemp_clicked();
    void on_pPBillu_clicked();
    void on_pPBhumi_clicked();

private:
    Ui::Tab1_control *ui;
    QTimer *timer;
    SocketClient *pSocketClient = nullptr;  // ✅ 안전 초기화
};

#endif // TAB1_CONTROL_H
