#include "tab2_set.h"
#include "ui_tab2_set.h"
#include <QDebug>
#include <QPoint>
#include <QString>
#include <QFontDatabase>
#include <QFile>

// LED 텍스트 <-> 단계(0~3) 변환
static inline QString ledText(int idx) {
    switch (idx) {
    case 0: return "OFF";
    case 1: return "LOW";
    case 2: return "MID";
    case 3: return "HIGH";
    }
    return "OFF";
}

Tab2_set::Tab2_set(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab2_set)
{
    ui->setupUi(this);

    // 다이얼로그 1개만 재사용
    mChangeDlg = new ChangeSetting(this);
    connect(mChangeDlg, &ChangeSetting::decided,
            this, &Tab2_set::onSettingDecided);

    connect(ui->pPBhome, &QPushButton::clicked, this, &Tab2_set::goToHome);

    if (ui->pLblLedState) {
            m_ledLevel = 0;
            ui->pLblLedState->setText("OFF");
            static bool s_loaded = false;
            static QString s_family;
            if (!s_loaded) {
                const char* kRes = ":/fonts/fonts/DSEG14Classic-Regular.ttf";
                if (QFile::exists(kRes)) {
                    int id = QFontDatabase::addApplicationFont(kRes);
                    if (id >= 0) s_family = QFontDatabase::applicationFontFamilies(id).value(0);
                }
                s_loaded = true;
            }
            if (!s_family.isEmpty()) {
                QFont f(s_family);
                f.setPointSize(36);
                f.setLetterSpacing(QFont::AbsoluteSpacing, 1);
                ui->pLblLedState->setFont(f);
                ui->pLblLedState->setAlignment(Qt::AlignCenter);
            }
        }
}

Tab2_set::~Tab2_set()
{
    delete ui;
}

// ───── 버튼 핸들러들 ─────
void Tab2_set::on_pPBAtemp_clicked()
{
    int current = 25;   // TODO: 실제 현재값으로 대체
    openSetting(ChangeSetting::Mode::Temp, current, -20, 60);
}

void Tab2_set::on_pPBAhumi_clicked()
{
    int current = 45;   // TODO
    openSetting(ChangeSetting::Mode::Humi, current, 0, 100);
}

void Tab2_set::on_pPBair_clicked()
{
    int current = 1;    // 예: 팬 강도 단계
    openSetting(ChangeSetting::Mode::Air, current, 0, 10);
}

void Tab2_set::on_pPBShumi_clicked()
{
    int current = 50;   // 토양 습도
    openSetting(ChangeSetting::Mode::SoilHumi, current, 0, 100);
}

void Tab2_set::on_pPBec_clicked()
{
    int current = 2;    // EC 단계
    openSetting(ChangeSetting::Mode::EC, current, 0, 10);
}

void Tab2_set::on_pPBph_clicked()
{
    int current = 7;    // pH 정수 단계(필요 시 소수점 지원은 별도)
    openSetting(ChangeSetting::Mode::PH, current, 0, 14);
}

// LED: OFF→LOW→MID→HIGH 단계 설정 열기
void Tab2_set::on_pPBled_clicked()
{
    openSetting(ChangeSetting::Mode::LED, m_ledLevel, 0, 3);
}

// ───── 공통 오픈/오버레이 ─────
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

// ───── OK 눌렀을 때 서버 문자열 전송 ─────
void Tab2_set::onSettingDecided(ChangeSetting::Mode mode, int value)
{
    if (mode == ChangeSetting::Mode::LED) {
            // 라벨 갱신
            const QString level = ledText(value);  // "OFF/LOW/MID/HIGH"
            if (ui->pLblLedState) ui->pLblLedState->setText(level);

            const QString msg = QString("[CCTV]LED@%1").arg(level);
            emit sendToServer(msg);
            qDebug() << "[Tab2_set] sendToServer LED:" << msg;
            return;
        }

    // 숫자 항목 공통 처리
    const Topic t = topicForMode(mode);
    const QString msg = QString("[CCTV]%1@%2@%3")
                            .arg(t.domain, t.key)
                            .arg(value);
    emit sendToServer(msg);
    qDebug() << "[Tab2_set] sendToServer:" << msg;
}

Tab2_set::Topic Tab2_set::topicForMode(ChangeSetting::Mode m) const
{
    switch (m) {
    // ── 공기(Air) ──
    case ChangeSetting::Mode::Temp:     return {"Air",  "temp"};
    case ChangeSetting::Mode::Humi:     return {"Air",  "humi"};
    case ChangeSetting::Mode::Air:      return {"Air",  "air"};

    // ── 토양(Land) ──
    case ChangeSetting::Mode::SoilHumi: return {"Land", "humi"};
    case ChangeSetting::Mode::EC:       return {"Land", "ec"};
    case ChangeSetting::Mode::PH:       return {"Land", "ph"};
    }
    return {"Air", "unknown"};
}
