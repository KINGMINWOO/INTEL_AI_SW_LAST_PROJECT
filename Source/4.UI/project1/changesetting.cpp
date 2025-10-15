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

// ======================== 상수/헬퍼 ========================
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

// 12시간 × 30분 = 24 스텝(half-hour)
static constexpr int kSpan = 24;

// 화면상 포인터는 위(12시)에서 시작하지만, pTime 표시가 9시로 어긋나던 현상 보정.
// 1시간 = 2스텝 → 9시간 = 18스텝.
// 필요 시 6(=3시), 12(=6시), 18(=9시)로 미세조정 가능.
static constexpr int kDisplayOffset = 18; // (참고용: 현재 로직은 BOTTOM_TO_TOP=12 방식 사용)

// ======================== 본체 ========================
ChangeSetting::ChangeSetting(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ChangeSetting)
{
    ui->setupUi(this);

    // Up/Down 연결 + 오토리핏
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

    // ===== LED 슬라이더 초기화 =====
    if (ui->pSliderLed) {
        ui->pSliderLed->setRange(0, 3);     // 0~3만 사용 (0=OFF,1=LOW,2=MID,3=HIGH)
        ui->pSliderLed->setSingleStep(1);
        ui->pSliderLed->setPageStep(1);

        connect(ui->pSliderLed, &QSlider::valueChanged, this, [this](int v){
            if (m_mode != Mode::LED) return;         // LED 모드에서만 반응
            int nv = qBound(0, v, 3);
            if (m_led != nv) {
                m_led = nv;
                updateView();                         // 라벨 텍스트 갱신
                emit valueChanged(m_led);             // 실시간 변경 알림(필요 시)
            }
        });
    }

    // ===== TIME 페이지 초기화 =====
    // 12시간 × 30분 = 24 스텝(0..23)
    if (ui->pDialtime) {
        ui->pDialtime->setRange(0, kSpan - 1);
        ui->pDialtime->setSingleStep(1);   // 30분 단위
        ui->pDialtime->setPageStep(4);     // 2시간(=4스텝)
        ui->pDialtime->setWrapping(true);
        ui->pDialtime->setNotchesVisible(true);

        connect(ui->pDialtime, &QDial::valueChanged, this, [this](int v){
            updateTimeFromDial(v);
        });
    }
    // AM/PM 토글 시에도 표시 갱신(표시는 12시간제로 동일하지만 타이핑 일관성 유지)
    if (ui->pBam) connect(ui->pBam, &QRadioButton::toggled, this, [this](bool){
        if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
    });
    if (ui->pBpm) connect(ui->pBpm, &QRadioButton::toggled, this, [this](bool){
        if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
    });
}

ChangeSetting::~ChangeSetting()
{
    delete ui;
}

void ChangeSetting::setMode(Mode m, int current, int minVal, int maxVal)
{
    m_mode = m;

    // 보여줄 페이지 전환
    if (ui->stackedWidget) {
        QWidget* target = nullptr;
        switch (m_mode) {
        case Mode::LED:  target = ui->page_2; break;
        case Mode::TIME: target = ui->page_3; break;
        default:         target = ui->page;   break;
        }
        if (target) ui->stackedWidget->setCurrentWidget(target);
    }

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
            if (ui->pSliderLed->maximum() != 3)
                ui->pSliderLed->setMaximum(3); // .ui에서 4로 되어 있어도 여기서 보정
            ui->pSliderLed->setValue(m_led);   // 슬라이더 위치 동기화
        }

        if (ui->set) ui->set->setVisible(false);
        updateView();
        return;
    }

    if (m_mode == Mode::TIME) {
        int minutes = qMax(0, current);
        minutes = (minutes / 30) * 30;               // 30분 스냅

        const int SPAN = 24;
        const int BOTTOM_TO_TOP = 12;                // ★ 아래(0) → 위(0) 보정 = 12스텝

        // 위(12시)=0 기준 half-step (0..23)
        int stepFromTop = (minutes / 30) % SPAN;

        // (위 기준 스텝) → (다이얼=아래 기준 값)으로 변환해서 setValue
        int dialVal = (stepFromTop + BOTTOM_TO_TOP) % SPAN;   // ★ 포인터를 '위'에 보이게
        if (ui->pDialtime) ui->pDialtime->setValue(dialVal);

        // AM/PM 기본 선택
        if (ui->pBam && ui->pBpm) {
            int hh24 = (minutes / 60) % 24;
            (hh24 >= 12) ? ui->pBpm->setChecked(true) : ui->pBam->setChecked(true);
        }

        if (ui->set) ui->set->setVisible(false);
        if (ui->pLblLed) ui->pLblLed->setVisible(false);

        if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
        return;
    }

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
        if (ui->pDialtime)
            ui->pDialtime->setValue( (ui->pDialtime->value() + 1) % kSpan ); // +30분
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
        if (ui->pDialtime)
            ui->pDialtime->setValue( (ui->pDialtime->value() + kSpan - 1) % kSpan ); // -30분
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
        emit decided(m_mode, m_led);   // 0~3
        accept();
        return;
    }

    if (m_mode == Mode::TIME) {
        if (!ui->pDialtime) { accept(); return; }

        const int SPAN = 24;
        const int BOTTOM_TO_TOP = 12;
        int t = (ui->pDialtime->value() + SPAN - BOTTOM_TO_TOP) % SPAN;  // 0..23

        int hour12 = t / 2;            // 0..11 (0=12)
        int minute = (t % 2) * 30;

        const bool isPm = (ui->pBpm && ui->pBpm->isChecked());
        int sendHour = isPm ? (hour12 + 12) : hour12;

        emit decided(m_mode, sendHour * 60 + minute);
        accept();
        return;
    }

    emit decided(m_mode, m_value); // 숫자 모드
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
            ui->pSliderLed->setValue(m_led);   // 버튼으로 바꿔도 슬라이더 위치 맞추기
        return;
    }

    if (m_mode == Mode::TIME) {
        if (ui->pDialtime) updateTimeFromDial(ui->pDialtime->value());
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
}

void ChangeSetting::updateTimeFromDial(int dialVal)
{
    const int SPAN = 24;
    const int BOTTOM_TO_TOP = 12;

    int t = (dialVal + SPAN - BOTTOM_TO_TOP) % SPAN;   // 0..23 (위 기준)

    int hour12 = t / 2;            // 0..11 (0=12)
    int minute = (t % 2) * 30;     // 0 or 30

    if (ui->pTime) {
        ui->pTime->display(QString("%1:%2")
                           .arg(hour12, 2, 10, QChar('0'))
                           .arg(minute,  2, 10, QChar('0')));
    }
}
