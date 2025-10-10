#include "tab2_set.h"
#include "ui_tab2_set.h"

#include <QPoint>

Tab2_set::Tab2_set(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab2_set)
{
    ui->setupUi(this);

    // ChangeSetting 하나만 생성하여 재사용
    mChangeDlg = new ChangeSetting(this);
    connect(mChangeDlg, &ChangeSetting::decided,
            this, &Tab2_set::onSettingDecided);

    // 버튼 연결(각 항목에 맞는 범위/초깃값은 예시, 프로젝트 값으로 바꾸세요)
    connect(ui->pPBAtemp, &QPushButton::clicked, this, &Tab2_set::on_pPBAtemp_clicked);
    connect(ui->pPBAhumi, &QPushButton::clicked, this, &Tab2_set::on_pPBAhumi_clicked);
    connect(ui->pPBillu,  &QPushButton::clicked, this, &Tab2_set::on_pPBillu_clicked);
    connect(ui->pPBair,   &QPushButton::clicked, this, &Tab2_set::on_pPBair_clicked);
    connect(ui->pPBStemp, &QPushButton::clicked, this, &Tab2_set::on_pPBStemp_clicked);
    connect(ui->pPBShumi, &QPushButton::clicked, this, &Tab2_set::on_pPBShumi_clicked);
    connect(ui->pPBec,    &QPushButton::clicked, this, &Tab2_set::on_pPBec_clicked);
    connect(ui->pPBph,    &QPushButton::clicked, this, &Tab2_set::on_pPBph_clicked);

    connect(ui->pPBhome,  &QPushButton::clicked, this, &Tab2_set::goToHome);
}

Tab2_set::~Tab2_set()
{
    delete ui;
}

// ===== 버튼 핸들러들 =====
void Tab2_set::on_pPBAtemp_clicked()
{
    int current = 25;   // TODO: 현재값 가져와서 사용
    openSetting(ChangeSetting::Mode::Temp, current, -20, 60);
}

void Tab2_set::on_pPBAhumi_clicked()
{
    int current = 45;   // TODO
    openSetting(ChangeSetting::Mode::Humi, current, 0, 100);
}

void Tab2_set::on_pPBillu_clicked()
{
    int current = 300;  // TODO
    openSetting(ChangeSetting::Mode::Illu, current, 0, 1000);
}

void Tab2_set::on_pPBair_clicked()
{
    int current = 1;    // 예: 공기유량/팬 강도 등 정수 값
    openSetting(ChangeSetting::Mode::Air, current, 0, 10);
}

void Tab2_set::on_pPBStemp_clicked()
{
    int current = 20;   // 토양 온도
    openSetting(ChangeSetting::Mode::SoilTemp, current, -10, 50);
}

void Tab2_set::on_pPBShumi_clicked()
{
    int current = 50;   // 토양 습도
    openSetting(ChangeSetting::Mode::SoilHumi, current, 0, 100);
}

void Tab2_set::on_pPBec_clicked()
{
    int current = 2;    // EC(전기전도도) 정수 단계 예시
    openSetting(ChangeSetting::Mode::EC, current, 0, 10);
}

void Tab2_set::on_pPBph_clicked()
{
    int current = 7;    // pH 정수 단계 예시(필요시 0~14)
    openSetting(ChangeSetting::Mode::PH, current, 0, 14);
}


// ===== 공통 오픈 =====
void Tab2_set::openSetting(ChangeSetting::Mode mode, int current, int minVal, int maxVal)
{
    mChangeDlg->setMode(mode, current, minVal, maxVal);
    showOverlay(*mChangeDlg, this);
}

void Tab2_set::showOverlay(ChangeSetting &dlg, QWidget *host)
{
    if (!host) host = this;

    dlg.setWindowFlags(dlg.windowFlags() | Qt::FramelessWindowHint | Qt::Dialog);
    dlg.setAttribute(Qt::WA_TranslucentBackground);
    dlg.setWindowModality(Qt::WindowModal);

    dlg.resize(host->size());
    dlg.move(host->mapToGlobal(QPoint(0, 0)));

    dlg.exec();
}

// ===== OK 눌렀을 때 서버용 문자열 전송 =====
void Tab2_set::onSettingDecided(ChangeSetting::Mode mode, int value)
{
    const QString tag = tagForMode(mode);
    const QString msg = QString("[CCTV]%1@%2").arg(tag).arg(value);
    emit sendToServer(msg);  // 부모에서 SocketClient로 연결하세요.
}

QString Tab2_set::tagForMode(ChangeSetting::Mode m) const
{
    switch (m) {
    case ChangeSetting::Mode::Temp:     return "airtemp";   // pPBAtemp
    case ChangeSetting::Mode::Humi:     return "airhumi";   // pPBAhumi
    case ChangeSetting::Mode::Illu:     return "airillu";   // pPBillu
    case ChangeSetting::Mode::Air:      return "air";       // pPBair
    case ChangeSetting::Mode::SoilTemp: return "soiltemp";  // pPBStemp
    case ChangeSetting::Mode::SoilHumi: return "soilhumi";  // pPBShumi
    case ChangeSetting::Mode::EC:       return "ec";        // pPBec
    case ChangeSetting::Mode::PH:       return "ph";        // pPBph
    }
    return "unknown";
}
