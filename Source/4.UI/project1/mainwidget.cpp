#include "mainwidget.h"
#include "ui_mainwidget.h"

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    pSocketClient = new SocketClient(this);

    connect(pSocketClient, SIGNAL(socketRecvDataSig(QString)),
            this, SLOT(updateRecvDataSlot(QString)));

    QTimer::singleShot(0, this, [this](){
        bool askIpPopup = false;
        pSocketClient->connectToServerSlot(askIpPopup);
    });

    pTab1_button = new Tab1_button(ui->pTab2);
    ui->pTab1->setLayout(pTab1_button->layout());

    pTab2_set = new Tab2_set(ui->pTab2);
    ui->pTab2->setLayout(pTab2_set->layout());

    pTab3_cctv = new Tab3_cctv(ui->pTab3);
    ui->pTab3->setLayout(pTab3_cctv->layout());

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
}

MainWidget::~MainWidget()
{
    delete ui;
}
