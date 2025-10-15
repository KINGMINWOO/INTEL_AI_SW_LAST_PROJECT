#include "mainwidget.h"
#include "ui_mainwidget.h"
#include <QCloseEvent>
#include <QTimer>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    pSocketClient = new SocketClient(this);

    ui->stackedWidget->setCurrentWidget(ui->pTab1);

    QTimer::singleShot(0, this, [this](){
        bool askIpPopup = true;
        pSocketClient->connectToServerSlot(askIpPopup);
    });

    pTab1_button = new Tab1_button(ui->pTab2);
    ui->pTab1->setLayout(pTab1_button->layout());

    pTab2_set = new Tab2_set(ui->pTab2);
    ui->pTab2->setLayout(pTab2_set->layout());

    pTab3_cctv = new Tab3_cctv(ui->pTab3);
    ui->pTab3->setLayout(pTab3_cctv->layout());

    pTab4_tomato = new Tab4_tomato(ui->pTab4);
    pTab4_tomato->setDbParams(
        "10.10.16.29",   // DB 호스트
        3306,             // 포트
        "smart_farm",      // DB 이름
        "user01",         // 사용자
        "user1234" // 비밀번호
    );

    Tab4_tomato::Schema sch;
    pTab4_tomato->setSchema(sch);
    ui->pTab4->setLayout(pTab4_tomato->layout());

    connect(pTab1_button, &Tab1_button::goToTab2, this, [this](){
        ui->stackedWidget->setCurrentWidget(ui->pTab2);
    });
    connect(pTab1_button, &Tab1_button::goToTab3, this, [this](){
        ui->stackedWidget->setCurrentWidget(ui->pTab3);
    });

    connect(pTab3_cctv, &Tab3_cctv::goToHome, this, [this](){
        ui->stackedWidget->setCurrentWidget(ui->pTab1);
    });

    connect(pTab2_set, &Tab2_set::goToHome, this, [this](){
        ui->stackedWidget->setCurrentWidget(ui->pTab1);
    });

    connect(pTab2_set, &Tab2_set::sendToServer,
            pSocketClient, &SocketClient::socketWriteDataSlot);

    connect(pTab1_button, &Tab1_button::goToTab4, this, [this](){
        ui->stackedWidget->setCurrentWidget(ui->pTab4);
    });

    connect(pTab4_tomato, &Tab4_tomato::goToHome, this, [this](){
        ui->stackedWidget->setCurrentWidget(ui->pTab1);
    });

    connect(pSocketClient, &SocketClient::socketRecvDataSig,
            pTab2_set,     &Tab2_set::onSocketMessage);
}

void MainWidget::closeEvent(QCloseEvent *e)
{
    e->ignore();
    ui->stackedWidget->setCurrentWidget(ui->pTab1);
    this->hide();
}

void MainWidget::bringToFront()
{
    this->showNormal();
    this->raise();
    this->activateWindow();
}

MainWidget::~MainWidget()
{
    delete ui;
}
