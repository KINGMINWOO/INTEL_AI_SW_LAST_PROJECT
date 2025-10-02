#include "tab_widget.h"
#include "ui_tab_widget.h"
#include <QTimer>  // ✅ 추가: singleShot 사용

Tab_widget::Tab_widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab_widget)
{
    ui->setupUi(this);

    // ===== ✅ 서버·클라이언트(소켓) 부분만 변경 =====
    pSocketClient = new SocketClient(this);

    // 수신 로그만 간단히 찍어 확인
    connect(pSocketClient, SIGNAL(socketRecvDataSig(QString)),
            this, SLOT(updateRecvDataSlot(QString)));

    // 앱 시작 직후 자동으로 "서버 IP 선택" 팝업 띄우기
    QTimer::singleShot(0, this, [this](){
        bool askIpPopup = false;   // ⚠️ 팝업이 뜨는 값으로 설정하세요.
        //    (프로젝트 규약에 따라 true/false 중 팝업 뜨는 쪽으로!)
        pSocketClient->connectToServerSlot(askIpPopup);
    });
    // ===== ✅ 여기까지만 서버·클라 부분 수정 =====

    pTab1_control = new Tab1_control(ui->pTab1);
    pTab1_control->setSocketClient(pSocketClient);
    ui->pTab1->setLayout(pTab1_control->layout());

    pTab2_CropStatus = new Tab2_CropStatus(ui->pTab2);
    ui->pTab2->setLayout(pTab2_CropStatus->layout());

    pTab3_TemHumiIlluDB = new Tab3_TemHumiIlluDB(ui->pTab3);
    ui->pTab3->setLayout(pTab3_TemHumiIlluDB->layout());

    pTab4_Camera = new Tab4_Camera(ui->pTab4);
    ui->pTab4->setLayout(pTab4_Camera->layout());
}

void Tab_widget::updateRecvDataSlot(const QString &msg)
{
    qDebug() << "[Socket] recv:" << msg;
}

Tab_widget::~Tab_widget()
{
    delete ui;
}
