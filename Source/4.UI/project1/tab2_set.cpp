#include "tab2_set.h"
#include "ui_tab2_set.h"

#include <QDebug>
#include <QPoint>
#include <QString>
#include <QFontDatabase>
#include <QFile>
#include <QRegularExpression>
#include <QCoreApplication>
#include <QSettings>
#include <QDir>
#include <QFileInfo>
#include <QTime>
#include <QDateTime>
#include <QTimer>
#include <QByteArray>

// ──────────────────────────────
// LED 라벨 텍스트
static inline QString ledText(int idx) {
    switch (idx) {
    case 0: return "OFF";
    case 1: return "LOW";
    case 2: return "MID";
    case 3: return "HIGH";
    }
    return "OFF";
}

static inline QString settingsPath() {
    // 0) 환경변수 우선
    const QByteArray v = qgetenv("SETTINGS_FILE");
    if (!v.isEmpty()) {
        const QString p = QString::fromLocal8Bit(v);
        // 부모 디렉터리 만들어두기
        QDir().mkpath(QFileInfo(p).path());
        return QDir::cleanPath(p);
    }

    // 1) 라즈베리파이: NFS 고정 경로 (존재 체크 없이 반환)
    QFile f("/proc/device-tree/model");
    bool isPi = false;
    if (f.exists() && f.open(QIODevice::ReadOnly)) {
        const QByteArray model = f.readAll();
        isPi = model.contains("Raspberry Pi");
    }
    if (isPi) {
        const QString nfsPath = "/mnt/nfs_ubuntu/project1/setting.txt";
        QDir().mkpath(QFileInfo(nfsPath).path());
        return QDir::cleanPath(nfsPath);
    }
    const QString appDir = QCoreApplication::applicationDirPath();
    QString srcPath = QDir(appDir).absoluteFilePath("setting.txt");
    srcPath = QDir::cleanPath(srcPath);
    QDir().mkpath(QFileInfo(srcPath).path());
    return srcPath;
}
// ──────────────────────────────

Tab2_set::Tab2_set(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab2_set)
{
    ui->setupUi(this);

    // 파일에서 보관값 로드 → UI 반영
    loadFromFile();
    refreshUiFromCurrent();

    // 변경 다이얼로그 연결
    mChangeDlg = new ChangeSetting(this);
    connect(mChangeDlg, &ChangeSetting::decided,
            this, &Tab2_set::onSettingDecided);

    // 홈 버튼
    connect(ui->pPBhome, &QPushButton::clicked, this, &Tab2_set::goToHome);

    // LED 라벨 폰트(옵션)
    if (ui->pLblLedState) {
        static bool s_loaded = false;
        static QString s_family;
        if (!s_loaded) {
            const char* kRes = ":/fonts/fonts/DSEG14Classic-Regular.ttf";
            if (QFile::exists(kRes)) {
                const int id = QFontDatabase::addApplicationFont(kRes);
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

    // ===== 싱글샷 스케줄러: 다음 트리거 시각으로 예약 =====
    m_oneShot.setSingleShot(true);
    m_oneShot.setTimerType(Qt::CoarseTimer); // 분 단위면 충분
    connect(&m_oneShot, &QTimer::timeout, this, &Tab2_set::onScheduledFire);
    scheduleNextFire();
}

Tab2_set::~Tab2_set()
{
    delete ui;
}

// ===== 버튼 클릭 =====
void Tab2_set::on_pPBAtemp_clicked()   { openSetting(ChangeSetting::Mode::Temp,     m_airTemp,   -20, 60); }
void Tab2_set::on_pPBAhumi_clicked()   { openSetting(ChangeSetting::Mode::Humi,     m_airHumi,     0,100); }
void Tab2_set::on_pPBair_clicked()     { openSetting(ChangeSetting::Mode::Air,      m_air,         0, 10); }
void Tab2_set::on_pPBShumi_clicked()   { openSetting(ChangeSetting::Mode::SoilHumi, m_soilHumi,    0,100); }
void Tab2_set::on_pPBec_clicked()      { openSetting(ChangeSetting::Mode::EC,       m_ec,          0, 10); }
void Tab2_set::on_pPBph_clicked()      { openSetting(ChangeSetting::Mode::PH,       m_ph,          0, 14); }
void Tab2_set::on_pPBled_clicked()     { openSetting(ChangeSetting::Mode::LED,      m_ledLevel,    0,  3); }
// TIME: 분 단위(0~1430), 30분 스냅은 ChangeSetting 내부 처리
void Tab2_set::on_pPBtime_clicked()    { openSetting(ChangeSetting::Mode::TIME,     m_timeMin,     0, 24*60-30); }

// ===== 다이얼로그 열기 =====
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

// ===== 파일 I/O =====
void Tab2_set::loadFromFile()
{
    const QString p = settingsPath();
    QSettings s(p, QSettings::IniFormat);

    // AIR — illu 키는 설정 파일에 없을 수 있음
    m_airTemp  = s.value("AIR/temp",  m_airTemp).toInt();
    m_airHumi  = s.value("AIR/humi",  m_airHumi).toInt();
    m_illu     = s.value("AIR/illu",  m_illu).toInt();
    m_air      = s.value("AIR/air",   m_air).toInt();

    // LAND
    m_soilHumi = s.value("LAND/humi", m_soilHumi).toInt();
    m_ec       = s.value("LAND/ec",   m_ec).toInt();
    m_ph       = s.value("LAND/ph",   m_ph).toInt();

    // SYS
    m_ledLevel = s.value("SYS/led",   m_ledLevel).toInt();

    // time: "HH:MM" 또는 정수(분) 허용
    const QVariant tv = s.value("SYS/time");
    if (tv.isValid()) {
        bool ok = false;
        const QTime t = QTime::fromString(tv.toString().trimmed(), "HH:mm");
        if (t.isValid()) {
            m_timeMin = t.hour()*60 + t.minute();
            ok = true;
        }
        if (!ok) {
            int mins = tv.toInt(&ok);
            if (ok && mins >= 0 && mins < 24*60) m_timeMin = mins;
        }
    }
}

void Tab2_set::saveToFile() const
{
    const QString p = settingsPath();
    QSettings s(p, QSettings::IniFormat);

    s.setValue("AIR/temp",  m_airTemp);
    s.setValue("AIR/humi",  m_airHumi);
    s.setValue("AIR/illu",  m_illu);
    s.setValue("AIR/air",   m_air);

    s.setValue("LAND/humi", m_soilHumi);
    s.setValue("LAND/ec",   m_ec);
    s.setValue("LAND/ph",   m_ph);

    s.setValue("SYS/led",   m_ledLevel);

    // time은 "HH:MM"로 저장
    const QString hhmm = QString("%1:%2")
                           .arg((m_timeMin/60)%24, 2, 10, QChar('0'))
                           .arg(m_timeMin%60,      2, 10, QChar('0'));
    s.setValue("SYS/time",  hhmm);

    s.sync();
}

// ===== UI 반영 =====
void Tab2_set::refreshUiFromCurrent()
{
    if (ui->pLcdtemp)  ui->pLcdtemp->display(m_airTemp);
    if (ui->pLcdhumi)  ui->pLcdhumi->display(m_airHumi);
    if (ui->pLcdair)   ui->pLcdair->display(m_air);
    if (ui->pLcdShumi) ui->pLcdShumi->display(m_soilHumi);
    if (ui->pLcdec)    ui->pLcdec->display(m_ec);
    if (ui->pLcdph)    ui->pLcdph->display(m_ph);
    if (ui->pLblLedState) ui->pLblLedState->setText(ledText(m_ledLevel));

    // 설정된 시간 표시 ("HH:MM")
    if (ui->pLcdtime) {
        const QString hhmm = QString("%1:%2")
                                .arg((m_timeMin/60)%24, 2, 10, QChar('0'))
                                .arg(m_timeMin%60,      2, 10, QChar('0'));
        ui->pLcdtime->display(hhmm);
    }
}

// ===== OK/Now 눌렀을 때 =====
// ===== OK/Now 눌렀을 때 =====
void Tab2_set::onSettingDecided(ChangeSetting::Mode mode, int value)
{
    auto sendCCTV = [&](const QString& payload){
        const QString msg = QString("[CCTV01]%1").arg(payload);
        emit sendToServer(msg);
        qDebug() << "[Tab2_set] sendToServer:" << msg;
    };

    // ─── LED ───
    if (mode == ChangeSetting::Mode::LED) {
        // ChangeSetting이 이미 파일에 저장했더라도, 혹시 모를 불일치 방지:
        // 1) 내부 상태 업데이트 → 2) 파일 저장 → 3) 파일에서 다시 읽어 LCD를 "파일값"으로 맞춤
        m_ledLevel = qBound(0, value, 3);
        saveToFile();
        loadFromFile();
        refreshUiFromCurrent();

        // (원하면) 송신 유지
        sendCCTV(QString("LED@%1").arg(ledText(m_ledLevel)));
        return;
    }

    // ─── TIME ───
    if (mode == ChangeSetting::Mode::TIME) {
        if (value == -1) {
            // NOW: 현재 시각으로 즉시 반영 + 저장 + 발사 + 재예약
            const QTime now = QTime::currentTime();
            m_timeMin = now.hour()*60 + now.minute();

            if (ui->pLcdtime)
                ui->pLcdtime->display(now.toString("HH:mm"));

            // 파일에도 반영(여기서 저장)
            {
                const QString p = settingsPath();
                QSettings s(p, QSettings::IniFormat);
                s.setValue("SYS/time", now.toString("HH:mm"));
                s.sync();
            }

            emit sendToServer("[TURTLE01]turtle@go");
            qDebug() << "[Tab2_set] NOW fired [TURTLE01]turtle@go";

            scheduleNextFire();
            return;
        } else {
            // 다이얼 OK: ChangeSetting이 setting.txt에 저장 완료 → 즉시 파일 재로드해서 LCD 갱신 + 재예약
            loadFromFile();         // ← 파일에서 HH:MM 다시 읽음
            refreshUiFromCurrent(); // ← LCD 즉시 반영
            scheduleNextFire();     // ← 다음 발사 재예약
            return;
        }
    }

    // ─── 숫자 항목들(온도/습도/조도/공기/토양습도/EC/pH) ───
    // 버튼으로 바꾼 값은 무조건 파일에 저장하고, "파일에서 다시 읽어" LCD를 파일값 그대로 보여주도록 고정
    const Topic t = topicForMode(mode);
    switch (mode) {
    case ChangeSetting::Mode::Temp:     m_airTemp   = value; break;
    case ChangeSetting::Mode::Humi:     m_airHumi   = value; break;
    case ChangeSetting::Mode::Air:      m_air       = value; break;
    case ChangeSetting::Mode::SoilHumi: m_soilHumi  = value; break;
    case ChangeSetting::Mode::EC:       m_ec        = value; break;
    case ChangeSetting::Mode::PH:       m_ph        = value; break;
    default: break;
    }

    // 1) 파일 저장 → 2) 파일 재로드 → 3) LCD 갱신 (항상 파일 기준으로 표기)
    saveToFile();
    loadFromFile();
    refreshUiFromCurrent();

    // (원하면) 송신 유지
    sendCCTV(QString("%1@%2@%3").arg(t.domain).arg(t.key).arg(value));
}

// ===== 모드→토픽 매핑 =====
Tab2_set::Topic Tab2_set::topicForMode(ChangeSetting::Mode m) const
{
    switch (m) {
    // ── 공기(AIR) ──
    case ChangeSetting::Mode::Temp:     return {"AIR",  "TEMP"};
    case ChangeSetting::Mode::Humi:     return {"AIR",  "HUMI"};
    case ChangeSetting::Mode::Air:      return {"AIR",  "AIR"};
    // ── 토양(LAND) ──
    case ChangeSetting::Mode::SoilHumi: return {"LAND", "HUMI"};
    case ChangeSetting::Mode::EC:       return {"LAND", "EC"};
    case ChangeSetting::Mode::PH:       return {"LAND", "PH"};
    // ── 기타 ──
    default:                            return {"AIR", "UNKNOWN"};
    }
}

// ===== 싱글샷: 다음 트리거 예약 & 발사 =====
void Tab2_set::scheduleNextFire()
{
    const QDateTime now = QDateTime::currentDateTime();
    QDateTime next(now.date(), QTime(m_timeMin/60, m_timeMin%60));

    if (next <= now)
        next = next.addDays(1);   // 오늘 시각 지났으면 내일

    const qint64 ms = now.msecsTo(next);
    m_oneShot.start(static_cast<int>(ms)); // 일일 알람이라 int로 안전
    qDebug() << "[Tab2_set] next fire at" << next.toString("yyyy-MM-dd HH:mm");
}

void Tab2_set::onScheduledFire()
{
    emit sendToServer("[TURTLE01]turtle@go");
    qDebug() << "[Tab2_set] fired [TURTLE01]turtle@go";

    // 다음 날 같은 시각으로 다시 예약
    scheduleNextFire();
}
