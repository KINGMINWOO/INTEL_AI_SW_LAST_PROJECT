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
    QString msg = "[ROS]Pan@ON";

    // 1. 콘솔(터미널) 출력
    qDebug() << "Send message:" << msg;

    // 2. Qt 메시지 박스로 출력
    QMessageBox::information(this, "Debug", "Send message: " + msg);

    // 3. 실제 서버 연결되면 아래처럼 전송 가능
    // socket->write(msg.toUtf8());
}

void Tab1_control::on_pPBillu_clicked()
{
    QString msg = "[ROS]Light@ON";

    // 1. 콘솔(터미널) 출력
    qDebug() << "Send message:" << msg;

    // 2. Qt 메시지 박스로 출력
    QMessageBox::information(this, "Debug", "Send message: " + msg);

    // 3. 실제 서버 연결되면 아래처럼 전송 가능
    // socket->write(msg.toUtf8());
}


void Tab1_control::on_pPBhumi_clicked()
{
    QString msg = "[ROS]Water@ON";

    // 1. 콘솔(터미널) 출력
    qDebug() << "Send message:" << msg;

    // 2. Qt 메시지 박스로 출력
    QMessageBox::information(this, "Debug", "Send message: " + msg);

    // 3. 실제 서버 연결되면 아래처럼 전송 가능
    // socket->write(msg.toUtf8());
}

