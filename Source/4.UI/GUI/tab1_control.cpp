#include "tab1_control.h"
#include "ui_tab1_control.h"
#include <QLCDNumber>
#include <QLabel>
#include <QDebug>   // 디버깅 출력
#include <QMessageBox> // Qt 창에 메시지 띄우기

Tab1_control::Tab1_control(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab1_control),
    timer(new QTimer(this))
{
    ui->setupUi(this);

    // 날짜 라벨 글자 크기 크게 설정
    ui->pDate->setStyleSheet("font-size: 30pt; font-weight: bold;");
    ui->pDate->setAlignment(Qt::AlignCenter);
    // 타이머 연결 (1초마다 updateDateTime 호출)
    connect(timer, &QTimer::timeout, this, &Tab1_control::updateDateTime);
    timer->start(1000);

    // 실행 시 바로 갱신되도록 한 번 호출
    updateDateTime();
}

Tab1_control::~Tab1_control()
{
    delete ui;
}

void Tab1_control::updateDateTime()
{
    // 현재 날짜와 시간 가져오기
    QDateTime current = QDateTime::currentDateTime();

    // 시간 → QLCDNumber pTime에 표시
    if (ui->pTime) {
        ui->pTime->display(current.time().toString("HH:mm:ss"));
    }

    // 날짜 → QLabel pDate에 표시 (요일 포함)
    if (ui->pDate) {
        ui->pDate->setText(current.date().toString("yyyy-MM-dd (ddd)"));
    }
}

// pPBtemp 클릭 시 처리
void Tab1_control::on_pPBtemp_clicked()
{
    QString msg = "[CCTV1]Pan@ON";

    qDebug() << "Send message:" << msg;
    // QMessageBox::information(this, "Debug", "Send message: " + msg);

    // ✅ 안전 체크: 소켓 없으면 안내하고 종료
    if (!pSocketClient) {
        QMessageBox::warning(this, "Socket", "서버와 아직 연결되지 않았습니다.\n연결 후 다시 시도하세요.");
        return;
    }
    pSocketClient->socketWriteDataSlot(msg);
}

// pPBillu 클릭 시 처리
void Tab1_control::on_pPBillu_clicked()
{
    QString msg = "[CCTV1]Light@ON";

    qDebug() << "Send message:" << msg;
    // QMessageBox::information(this, "Debug", "Send message: " + msg);

    // ✅ 안전 체크
    if (!pSocketClient) {
        QMessageBox::warning(this, "Socket", "서버와 아직 연결되지 않았습니다.\n연결 후 다시 시도하세요.");
        return;
    }
    pSocketClient->socketWriteDataSlot(msg);
}

// pPBhumi 클릭 시 처리
void Tab1_control::on_pPBhumi_clicked()
{
    QString msg = "[CCTV1]Water@ON";

    qDebug() << "Send message:" << msg;
    // QMessageBox::information(this, "Debug", "Send message: " + msg);

    // ✅ 안전 체크
    if (!pSocketClient) {
        QMessageBox::warning(this, "Socket", "서버와 아직 연결되지 않았습니다.\n연결 후 다시 시도하세요.");
        return;
    }
    pSocketClient->socketWriteDataSlot(msg);
}
