#include "changesetting.h"
#include "ui_changesetting.h"

#include <QDialogButtonBox>
#include <QPushButton>
#include <QLCDNumber>
#include <QLabel>
#include <QSlider>
#include <QRadioButton>
#include <QDial>
#include <QtMath>
#include <QFontDatabase>
#include <QFile>
#include <QFileInfo>
#include <QTime>
#include <QSettings>
#include <QCoreApplication>
#include <QDir>
#include <QSignalBlocker>
#include <QPalette>
#include <QMessageBox>

// ======================== 헬퍼/상수 ========================
static inline QString ledText(int idx)
{
    switch (idx) {
    case 0: return "OFF";
    case 1: return "LOW";
    case 2: return "MID";
    case 3: return "HIGH";
    }
    return "OFF";
}

// Tab2와 동일한 설정 파일 경로 규칙
static bool s_inRed = false; // RED 구간 ‘안’에 있는지 단순 상태

static inline QString settingsPath() {
    // 0) 환경변수 우선
    const QByteArray v = qgetenv("SETTINGS_FILE");
    if (!v.isEmpty()) {
        const QString p = QString::fromLocal8Bit(v);
        QDir().mkpath(QFileInfo(p).path());
        return QDir::cleanPath(p);
    }

    // 1) 라즈베리파이: NFS 고정 경로
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

    // 2) 일반: 실행파일 디렉터리/setting.txt
    const QString appDir = QCoreApplication::applicationDirPath();
    QString srcPath = QDir(appDir).absoluteFilePath("setting.txt");
    srcPath = QDir::cleanPath(srcPath);
    QDir().mkpath(QFileInfo(srcPath).path());
    return srcPath;
}

// 다이얼: 12시간 × 30분 = 24눈금, 눈금 사이 표시용까지 48스텝
static constexpr int kDialSteps        = 48;  // 0..47
static constexpr int kBottomToTopSteps = 24;  // 보정
enum class Dir { HigherIsWorse, LowerIsWorse };

struct Rule { int y; int r; Dir dir; };

struct BiRule {
    int low_y  = INT_MIN;
    int low_r  = INT_MIN;
    bool useLow = false;

    int high_y = INT_MAX;
    int high_r = INT_MAX;
    bool useHigh = false;
};

static inline QColor kOrange() { return QColor(255, 165, 0); }
static inline QColor kRed()        { return Qt::red; }
static inline QColor kBlack()      { return Qt::black; }

static inline BiRule ruleFor(ChangeSetting::Mode m) {
    using M = ChangeSetting::Mode;
    switch (m) {
    case M::Temp: {
        BiRule r;
        r.low_y   = 23;  r.low_r   = 19;  r.useLow  = true;
        r.high_y  = 27;  r.high_r  = 31;  r.useHigh = true;
        return r;
    }
    case M::Humi: {
        BiRule r;
        r.low_y   = 64;  r.low_r   = 54;  r.useLow  = true;
        r.high_y = 76;  r.high_r  = 86;  r.useHigh = true;
        return r;
    }
    case M::Air: {
        BiRule r;
        r.low_y   = 25;  r.low_r   = 23;  r.useLow  = true;
        r.high_y = 31;  r.high_r  = 33;  r.useHigh = true;
        return r;
    }
    case M::SoilHumi: {
        BiRule r;
        r.low_y   = 44;  r.low_r   = 32;  r.useLow  = true;
        r.high_y = 56;  r.high_r  = 66;  r.useHigh = true;
        return r;
    }
    case M::EC: {
        BiRule r;
        r.low_y   = 2.4;  r.low_r   = 1.9;  r.useLow = true;
        r.high_y = 3.1;   r.high_r = 3.6;    r.useHigh = true;
        return r;
    }
    case M::PH: {
        BiRule r;
        r.low_y = 5.7;    r.low_r = 5.4;     r.useLow  = true;
        r.high_y = 6.3;   r.high_r = 6.6;    r.useHigh = true;
        return r;
    }
    default:
        return BiRule{}; // 둘 다 비활성
    }
}

// 값→색 (빨강 우선, 그 다음 노랑)
static inline QColor pickColor(ChangeSetting::Mode m, int v) {
    const BiRule r = ruleFor(m);

    // 하한 체크(작을수록 위험)
    if (r.useLow) {
        if (v <= r.low_r) return kRed();
        if (v <= r.low_y) return kOrange();
    }
    // 상한 체크(클수록 위험)
    if (r.useHigh) {
        if (v >= r.high_r) return kRed();
        if (v >= r.high_y) return kOrange();
    }
    return kBlack();
}

static inline void setLcdColor(QLCDNumber* lcd, const QColor& c) {
    if (!lcd) return;
    QPalette p = lcd->palette();
    p.setColor(QPalette::WindowText, c);  // QLCDNumber Filled 세그먼트
    lcd->setPalette(p);
}
// ======================== 본체 ========================
ChangeSetting::ChangeSetting(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ChangeSetting)
{
    ui->setupUi(this);

    // Up/Down + 오토리핏
    connect(ui->pPBup,   &QPushButton::clicked, this, &ChangeSetting::onUp);
    connect(ui->pPBdown, &QPushButton::clicked, this, &ChangeSetting::onDown);
    if (ui->pPBup) {
        ui->pPBup->setAutoRepeat(true);
        ui->pPBup->setAutoRepeatDelay(300);
        ui->pPBup->setAutoRepeatInterval(60);
    }
    if (ui->pPBdown) {
        ui->pPBdown->setAutoRepeat(true);
        ui->pPBdown->setAutoRepeatDelay(300);
        ui->pPBdown->setAutoRepeatInterval(60);
    }

    // OK/Cancel
    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ChangeSetting::onAccepted);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &ChangeSetting::reject);

    // 숫자 LCD 기본 스타일
    if (ui->set) ui->set->setSegmentStyle(QLCDNumber::Filled);

    // LED 슬라이더
    if (ui->pSliderLed) {
        ui->pSliderLed->setRange(0, 3);
        ui->pSliderLed->setSingleStep(1);
        ui->pSliderLed->setPageStep(1);
        connect(ui->pSliderLed, &QSlider::valueChanged, this, [this](int v){
            if (m_mode != Mode::LED) return;
            int nv = qBound(0, v, 3);
            if (m_led != nv) {
                m_led = nv;
                updateView();
                emit valueChanged(m_led);
            }
        });
    }

    // TIME 다이얼
    if (ui->pDialtime) {
        ui->pDialtime->setRange(0, kDialSteps - 1);
        ui->pDialtime->setSingleStep(2);      // 30분 단위만
        ui->pDialtime->setPageStep(2);
        ui->pDialtime->setWrapping(true);
        ui->pDialtime->setNotchesVisible(false);

        connect(ui->pDialtime, &QDial::valueChanged, this, [this](int v){
            // 다이얼 조작 → 정확 모드 해제
            m_nowExactActive = false;

            // 홀수값 들어오면 짝수로 스냅
            int t = (v + kDialSteps - kBottomToTopSteps) % kDialSteps;
            if (t % 2 != 0) {
                int even = (t < kDialSteps-1 ? t+1 : t-1);
                int desired = (even + kBottomToTopSteps) % kDialSteps;
                QSignalBlocker b(ui->pDialtime);
                ui->pDialtime->setValue(desired);
                t = even;
            }
            updateTimeFromDial((t + kBottomToTopSteps) % kDialSteps);
        });
    }

    // AM/PM 토글: 사용자 조작 시에만 정확모드 해제
    if (ui->pBam) connect(ui->pBam, &QRadioButton::toggled, this, [this](bool){
        m_nowExactActive = false;
        if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
    });
    if (ui->pBpm) connect(ui->pBpm, &QRadioButton::toggled, this, [this](bool){
        m_nowExactActive = false;
        if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
    });

    // NOW 버튼: 정확 모드 ON
    if (ui->pPBnow) {
        connect(ui->pPBnow, &QPushButton::clicked, this, [this]{
            updateTimeFromDial(-1); // 내부에서 m_nowExactActive=true
        });
    }
}

ChangeSetting::~ChangeSetting()
{
    delete ui;
}

void ChangeSetting::setMode(Mode m, int current, int minVal, int maxVal)
{
    m_mode = m;

    // 페이지 전환
    if (ui->stackedWidget) {
        QWidget* target = nullptr;
        switch (m_mode) {
        case Mode::LED:  target = ui->page_2; break;
        case Mode::TIME: target = ui->page_3; break;
        default:         target = ui->page;   break;
        }
        if (target) ui->stackedWidget->setCurrentWidget(target);
    }

    // ───────── LED ─────────
    if (m_mode == Mode::LED) {
        m_led = qBound(0, current, 3);

        if (ui->pLblLed) {
            ui->pLblLed->setVisible(true);
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
                ui->pLblLed->setFont(f);
            }
        }
        if (ui->pSliderLed) {
            if (ui->pSliderLed->maximum() != 3) ui->pSliderLed->setMaximum(3);
            ui->pSliderLed->setValue(m_led);
        }
        if (ui->set) ui->set->setVisible(false);
        updateView();
        return;
    }

    // ───────── TIME ─────────
    if (m_mode == Mode::TIME) {
        // 1) setting.txt에서 HH:MM 읽기(없으면 current)
        int minutesExact = -1; // 0..1439
        {
            const QString p = settingsPath();
            QSettings s(p, QSettings::IniFormat);
            s.beginGroup("SYS");
            const QString tv = s.value("time").toString().trimmed(); // "HH:MM"
            s.endGroup();
            const QTime t = QTime::fromString(tv, "HH:mm");
            if (t.isValid()) minutesExact = t.hour()*60 + t.minute();
        }
        if (minutesExact < 0) minutesExact = qMax(0, current);  // fallback

        // 12시간 기준
        const int hh24     = (minutesExact / 60) % 24;
        const int mm       = minutesExact % 60;
        const int minutes12 = (hh24 % 12) * 60 + mm; // 0..719
        const int baseHalf  = minutes12 / 30;        // 0..23
        const int rem       = minutes12 % 30;        // 0..29

        if (ui->set)     ui->set->setVisible(false);
        if (ui->pLblLed) ui->pLblLed->setVisible(false);

        // 2) 다이얼 값 설정(신호 차단)
        if (ui->pDialtime) {
            QSignalBlocker blockDial(ui->pDialtime);
            ui->pDialtime->setRange(0, kDialSteps - 1);
            ui->pDialtime->setSingleStep(2);
            ui->pDialtime->setPageStep(2);
            ui->pDialtime->setWrapping(true);
            ui->pDialtime->setNotchesVisible(false);

            int sBase = baseHalf * 2 + (rem == 0 ? 0 : 1); // rem>0이면 언저리(홀수)
            int dialVal = (sBase + kBottomToTopSteps) % kDialSteps;
            ui->pDialtime->setValue(dialVal);
        }

        // 3) LCD 정확 분 표시
        if (ui->pTime) {
            const int h12 = hh24 % 12;
            const int dispHour = (h12 == 0 ? 12 : h12);
            ui->pTime->display(QString("%1:%2")
                               .arg(dispHour, 2, 10, QChar('0'))
                               .arg(mm,       2, 10, QChar('0')));
        }

        // 4) AM/PM 체크 설정 시 **신호 차단**으로 정확모드 보존
        if (ui->pBam && ui->pBpm) {
            QSignalBlocker blockAm(ui->pBam);
            QSignalBlocker blockPm(ui->pBpm);
            (hh24 >= 12) ? ui->pBpm->setChecked(true) : ui->pBam->setChecked(true);
        }

        // 5) 초기 상태는 '정확 모드'
        m_nowExactActive  = true;
        m_nowExactMinutes = minutesExact;
        return;
    }

    // ───────── 숫자 ─────────
    m_value = current;
    m_min   = minVal;
    m_max   = maxVal;
    clamp();

    if (ui->pLblLed) ui->pLblLed->setVisible(false);
    if (ui->set)     ui->set->setVisible(true);
    updateView();
}

void ChangeSetting::onUp()
{
    if (m_mode == Mode::LED) {
        if (m_led < 3) {
            ++m_led;
            updateView();
            emit valueChanged(m_led);
        }
        return;
    }
    if (m_mode == Mode::TIME) {
        if (ui->pDialtime) {
            int v = (ui->pDialtime->value() + 2) % kDialSteps; // +30분
            QSignalBlocker b(ui->pDialtime);
            ui->pDialtime->setValue(v);
            updateTimeFromDial(v);
        }
        return;
    }
    if (m_value < m_max) {
        ++m_value;
        updateView();
        emit valueChanged(m_value);
    }
}

void ChangeSetting::onDown()
{
    if (m_mode == Mode::LED) {
        if (m_led > 0) {
            --m_led;
            updateView();
            emit valueChanged(m_led);
        }
        return;
    }
    if (m_mode == Mode::TIME) {
        if (ui->pDialtime) {
            int v = (ui->pDialtime->value() + kDialSteps - 2) % kDialSteps; // -30분
            QSignalBlocker b(ui->pDialtime);
            ui->pDialtime->setValue(v);
            updateTimeFromDial(v);
        }
        return;
    }
    if (m_value > m_min) {
        --m_value;
        updateView();
        emit valueChanged(m_value);
    }
}

void ChangeSetting::onAccepted()
{
    if (m_mode == Mode::LED) {
        emit decided(m_mode, m_led);
        accept();
        return;
    }

    if (m_mode == Mode::TIME) {
        // 정확 모드면 NOW(-1) 처리
        if (m_nowExactActive) {
            const int minutes = qMax(0, m_nowExactMinutes) % (24*60);
            const QString hhmm = QString("%1:%2")
                                     .arg((minutes/60)%24, 2, 10, QChar('0'))
                                     .arg(minutes%60,      2, 10, QChar('0'));
            const QString p = settingsPath();
            QSettings s(p, QSettings::IniFormat);
            s.setValue("SYS/time", hhmm);
            s.sync();

            emit decided(m_mode, -1);
            accept();
            return;
        }

        // 다이얼(짝수) + AM/PM 반영 저장
        if (!ui->pDialtime) { accept(); return; }

        int v = ui->pDialtime->value();
        int t = (v + kDialSteps - kBottomToTopSteps) % kDialSteps; // 0..47
        if (t % 2 != 0) t -= 1;

        const int halfIdx = t / 2;         // 0..23
        const int hour12  = halfIdx / 2;   // 0..11
        const int minute  = (halfIdx % 2) * 30;

        const bool isPm = (ui->pBpm && ui->pBpm->isChecked());
        int hh24 = isPm ? ((hour12 + 12) % 24) : hour12;

        const int minutes = hh24*60 + minute;
        const QString hhmm = QString("%1:%2")
                                 .arg((minutes/60)%24, 2, 10, QChar('0'))
                                 .arg(minutes%60,      2, 10, QChar('0'));
        const QString p = settingsPath();
        QSettings s(p, QSettings::IniFormat);
        s.setValue("SYS/time", hhmm);
        s.sync();

        emit decided(m_mode, minutes);
        accept();
        return;
    }

    // 숫자 모드
    emit decided(m_mode, m_value);
    accept();
}

void ChangeSetting::clamp()
{
    if (m_value < m_min) m_value = m_min;
    if (m_value > m_max) m_value = m_max;
}

void ChangeSetting::updateView()
{
    if (m_mode == Mode::LED) {
        if (ui->pLblLed) ui->pLblLed->setText(ledText(m_led));
        if (ui->pSliderLed && ui->pSliderLed->value() != m_led)
            ui->pSliderLed->setValue(m_led);
        return;
    }

    if (m_mode == Mode::TIME) {
        if (m_nowExactActive) {
            const int minutes = qMax(0, m_nowExactMinutes) % (24*60);
            const int hh24    = (minutes / 60) % 24;
            const int mm      = minutes % 60;
            const int h12     = hh24 % 12;
            const int disp    = (h12 == 0 ? 12 : h12);
            if (ui->pTime) {
                ui->pTime->display(QString("%1:%2")
                                   .arg(disp, 2, 10, QChar('0'))
                                   .arg(mm,   2, 10, QChar('0')));
            }
            if (ui->pBam && ui->pBpm) {
                // 여기서는 신호 차단 없이 읽기만 함
                (hh24 >= 12) ? ui->pBpm->setChecked(true) : ui->pBam->setChecked(true);
            }
        } else {
            if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
        }
        return;
    }

    // 숫자 모드
    if (!ui->set) return;
    int absVal = qAbs(m_value);
    int digits = 1;
    if (absVal >= 10)  digits = 2;
    if (absVal >= 100) digits = 3;
    if (m_value < 0)   digits += 1;

    ui->set->setDigitCount(digits);
    ui->set->display(m_value);

    const QColor c = pickColor(m_mode, m_value);
    setLcdColor(ui->set, c);

    if (c == Qt::red && !s_inRed) {
        s_inRed = true;
        QMessageBox::warning(this, "경고", "설정값이 위험 범위에 진입했습니다.");
    } else if (c != Qt::red) {
        s_inRed = false;
    }
}

void ChangeSetting::updateTimeFromDial(int dialVal)
{
    // NOW: LCD는 정확 분, 다이얼은 언저리/정확 위치
    if (dialVal == -1) {
        const QTime now = QTime::currentTime();
        const int minutesExact = now.hour()*60 + now.minute();
        const int minutes12    = (now.hour()%12)*60 + now.minute();

        // LCD 정확 표시
        if (ui->pTime) {
            const int h12 = now.hour() % 12;
            const int disp = (h12 == 0 ? 12 : h12);
            ui->pTime->display(QString("%1:%2")
                               .arg(disp, 2, 10, QChar('0'))
                               .arg(now.minute(), 2, 10, QChar('0')));
        }
        if (ui->pBam && ui->pBpm) {
            QSignalBlocker blockAm(ui->pBam);
            QSignalBlocker blockPm(ui->pBpm);
            (now.hour() >= 12) ? ui->pBpm->setChecked(true) : ui->pBam->setChecked(true);
        }

        if (ui->pDialtime) {
            const int baseHalf = minutes12 / 30;
            const int rem      = minutes12 % 30;
            int sBase = baseHalf * 2 + (rem == 0 ? 0 : 1); // rem>0 → 홀수
            int desired = (sBase + kBottomToTopSteps) % kDialSteps;
            QSignalBlocker b(ui->pDialtime);
            ui->pDialtime->setValue(desired);
        }

        m_nowExactActive  = true;
        m_nowExactMinutes = minutesExact;
        return;
    }

    // 일반: 다이얼(짝수 스냅) → 00/30 표시
    int v = dialVal;
    int t = (v + kBottomToTopSteps) % kDialSteps; // 이미 위 기준 값으로 들어옴
    // 안전: 혹시 홀수면 짝수로
    int tTop = (v + kDialSteps - kBottomToTopSteps) % kDialSteps;
    if (tTop % 2 != 0) tTop -= 1;

    const int halfIdx = tTop / 2;       // 0..23
    const int hour12  = halfIdx / 2;    // 0..11
    const int minute  = (halfIdx % 2) * 30;
    const int dispHour = (hour12 == 0 ? 12 : hour12);

    if (ui->pTime) {
        ui->pTime->display(QString("%1:%2")
                           .arg(dispHour, 2, 10, QChar('0'))
                           .arg(minute,   2, 10, QChar('0')));
    }
}
