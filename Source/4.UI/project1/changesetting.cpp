#include "changesetting.h"
#include "ui_changesetting.h"

#include <QDialogButtonBox>
#include <QPushButton>
#include <QLCDNumber>
#include <QLabel>
#include <QtMath>
#include <QFontDatabase>
#include <QFile>

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

ChangeSetting::ChangeSetting(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ChangeSetting)
{
    ui->setupUi(this);

    // Up/Down 연결
    connect(ui->pPBup,   &QPushButton::clicked, this, &ChangeSetting::onUp);
    connect(ui->pPBdown, &QPushButton::clicked, this, &ChangeSetting::onDown);

    // 길게 누르면 반복
    ui->pPBup->setAutoRepeat(true);
    ui->pPBup->setAutoRepeatDelay(300);
    ui->pPBup->setAutoRepeatInterval(60);
    ui->pPBdown->setAutoRepeat(true);
    ui->pPBdown->setAutoRepeatDelay(300);
    ui->pPBdown->setAutoRepeatInterval(60);

    // OK/Cancel
    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ChangeSetting::onAccepted);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &ChangeSetting::reject);

    // 숫자 LCD 기본 스타일
    if (ui->set) ui->set->setSegmentStyle(QLCDNumber::Filled);

    // LED 텍스트 라벨은 기본 비가시(숫자 모드가 기본이니까)
    if (ui->pLblLed) ui->pLblLed->setVisible(false);
}

ChangeSetting::~ChangeSetting()
{
    delete ui;
}

void ChangeSetting::setMode(Mode m, int current, int minVal, int maxVal)
{
    m_mode = m;

    if (m_mode == Mode::LED) {
            m_led = qBound(0, current, 3);

            if (ui->pLblLed) {
                ui->pLblLed->setVisible(true);
                if (ui->set) ui->set->setVisible(false);

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

            updateView();
            return;
        }

    // 숫자 모드
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
        return;
    }

    if (!ui->set) return;
    int absVal = qAbs(m_value);
    int digits = 1;
    if (absVal >= 10)  digits = 2;
    if (absVal >= 100) digits = 3;
    if (m_value < 0)   digits += 1;

    ui->set->setDigitCount(digits);
    ui->set->display(m_value);
}
