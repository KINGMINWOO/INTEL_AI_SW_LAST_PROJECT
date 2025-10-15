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

// ===== 플랫폼별 설정 파일 경로 결정 =====

// (옵션) 환경변수로 강제 지정: SETTINGS_FILE=/path/to/setting.txt
static inline QString envSettingsFile() {
    const QByteArray v = qgetenv("SETTINGS_FILE");
    return v.isEmpty() ? QString() : QString::fromLocal8Bit(v);
}

// 라즈베리파이 런타임 감지: /proc/device-tree/model 확인
static inline bool isRaspberryPi() {
    QFile f("/proc/device-tree/model");
    if (!f.exists() || !f.open(QIODevice::ReadOnly)) return false;
    const QByteArray model = f.readAll();
    return model.contains("Raspberry Pi");
}

// 설정 파일 경로:
// 0) 환경변수 SETTINGS_FILE 지정 시 그 파일
// 1) 라즈베리파이: /mnt/nfs_ubuntu/project1/setting.txt
// 2) (우분투 등) 실행파일 기준 ../../setting.txt (소스 루트 상정)
// 3) 폴백: 빌드 폴더 하위 project1/setting.txt
static inline QString settingsPath() {
    const QString env = envSettingsFile();
    if (!env.isEmpty() && QFile::exists(env))
        return env;

    if (isRaspberryPi()) {
        const QString nfsPath = "/mnt/nfs_ubuntu/project1/setting.txt";
        if (QFile::exists(nfsPath))
            return nfsPath;
        // 필요 시 존재체크 없이 nfsPath를 그대로 반환해도 됨
    }

    const QString appDir = QCoreApplication::applicationDirPath();
    QString srcPath = QDir(appDir).absoluteFilePath("../../setting.txt");
    srcPath = QDir::cleanPath(srcPath);
    if (QFile::exists(srcPath))
        return srcPath;

    QString buildPath = QDir(appDir).absoluteFilePath("project1/setting.txt");
    buildPath = QDir::cleanPath(buildPath);
    QDir().mkpath(QFileInfo(buildPath).path());
    return buildPath;
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

// ===== OK 눌렀을 때 =====
void Tab2_set::onSettingDecided(ChangeSetting::Mode mode, int value)
{
    auto send = [&](const QString& payload){
        const QString msg = QString("[CCTV01]%1").arg(payload);
        emit sendToServer(msg);
        qDebug() << "[Tab2_set] sendToServer:" << msg;
    };

    if (mode == ChangeSetting::Mode::LED) {
        m_ledLevel = qBound(0, value, 3);
        refreshUiFromCurrent();
        saveToFile();
        send(QString("LED@%1").arg(ledText(m_ledLevel)));
        return;
    }

    if (mode == ChangeSetting::Mode::TIME) {
        value = (value / 30) * 30;   // 30분 스냅
        m_timeMin = value;

        // UI 즉시 반영
        const QString hhmm = QString("%1:%2")
                               .arg((m_timeMin/60)%24, 2, 10, QChar('0'))
                               .arg(m_timeMin%60,      2, 10, QChar('0'));
        if (ui->pLcdtime) ui->pLcdtime->display(hhmm);
        saveToFile();
        send(QString("TIME@%1").arg(hhmm));

        // 새 시간으로 알람 재예약
        scheduleNextFire();
        return;
    }

    // 숫자 항목들
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

    refreshUiFromCurrent();
    saveToFile();
    send(QString("%1@%2@%3").arg(t.domain).arg(t.key).arg(value));
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

// ===== 서버 수신 파싱 =====
bool Tab2_set::parseCCTV01(const QString& msg)
{
    // 기대 포맷: "[CCTV01]22.5@48.1@321@1@35@2@7@2"
    // (temp@humi@illu@air@soil@ec@ph@led)
    if (!msg.startsWith("[CCTV01]", Qt::CaseInsensitive))
        return false;

    const QString payload = msg.mid(msg.indexOf(']') + 1).trimmed();
    const QStringList toks = payload.split('@', Qt::KeepEmptyParts);
    if (toks.size() < 8) return false;

    bool ok = true;
    const int airTemp  = qRound(toks[0].toDouble(&ok)); if (!ok) return false;
    const int airHumi  = qRound(toks[1].toDouble(&ok)); if (!ok) return false;
    const int illu     = qRound(toks[2].toDouble(&ok)); if (!ok) return false;
    const int air      = qRound(toks[3].toDouble(&ok)); if (!ok) return false;
    const int soilHumi = qRound(toks[4].toDouble(&ok)); if (!ok) return false;
    const int ec       = qRound(toks[5].toDouble(&ok)); if (!ok) return false;
    const int ph       = qRound(toks[6].toDouble(&ok)); if (!ok) return false;
    int led            = toks[7].toInt(&ok);            if (!ok) return false;

    // 보관값 갱신(수신값)
    m_airTemp   = airTemp;
    m_airHumi   = airHumi;
    m_illu      = illu;
    m_air       = air;
    m_soilHumi  = soilHumi;
    m_ec        = ec;
    m_ph        = ph;
    m_ledLevel  = qBound(0, led, 3);

    // UI 즉시 갱신
    refreshUiFromCurrent();

    qDebug() << "[Tab2_set] 기본값 업데이트(서버 수신)"
             << m_airTemp << m_airHumi << m_illu << m_air
             << m_soilHumi << m_ec << m_ph << m_ledLevel;

    // 수신으로 파일도 갱신하려면 아래 한 줄 활성화
    // saveToFile();

    return true;
}

void Tab2_set::onSocketMessage(const QString& msg)
{
    const QStringList lines = msg.split(QRegularExpression("[\\r\\n]+"),
                                        Qt::SkipEmptyParts);
    for (const QString& line : lines)
        parseCCTV01(line.trimmed());
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
    emit sendToServer("[TURTLE]go");
    qDebug() << "[Tab2_set] fired [TURTLE]go";

    // 다음 날 같은 시각으로 다시 예약
    scheduleNextFire();
}
