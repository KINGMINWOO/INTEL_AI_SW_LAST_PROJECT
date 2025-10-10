#include "changesetting.h"
#include "ui_changesetting.h"

#include <QDialogButtonBox>
#include <QtMath>

ChangeSetting::ChangeSetting(QWidget *parent)
    : QDialog(parent), ui(new Ui::ChangeSetting)
{
    ui->setupUi(this);

    // Up(=pushButton_2), Down(=pushButton) 연결
    connect(ui->pPBup, &QPushButton::clicked, this, &ChangeSetting::onUp);
    connect(ui->pPBdown,   &QPushButton::clicked, this, &ChangeSetting::onDown);

    // 길게 누르면 반복(선택)
    ui->pPBup->setAutoRepeat(true);
    ui->pPBup->setAutoRepeatDelay(300);
    ui->pPBup->setAutoRepeatInterval(60);

    ui->pPBdown->setAutoRepeat(true);
    ui->pPBdown->setAutoRepeatDelay(300);
    ui->pPBdown->setAutoRepeatInterval(60);

    // OK/Cancel
    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &ChangeSetting::onAccepted);
    // rejected()는 기본 reject() 슬롯 연결로 충분

    // LCD 표시 옵션(디자이너에서도 가능)
    ui->set->setSegmentStyle(QLCDNumber::Filled);
}

ChangeSetting::~ChangeSetting()
{
    delete ui;
}

void ChangeSetting::setMode(Mode m, int value, int minVal, int maxVal)
{
    m_mode  = m;
    m_value = value;
    m_min   = minVal;
    m_max   = maxVal;
    clamp();
    updateLCD();
}

void ChangeSetting::onUp()
{
    if (m_value < m_max) {
        ++m_value;               // 1씩 증가
        updateLCD();
        emit valueChanged(m_value);
    }
}

void ChangeSetting::onDown()
{
    if (m_value > m_min) {
        --m_value;               // 1씩 감소
        updateLCD();
        emit valueChanged(m_value);
    }
}

void ChangeSetting::onAccepted()
{
    emit decided(m_mode, m_value);
    accept();
}

void ChangeSetting::clamp()
{
    if (m_value < m_min) m_value = m_min;
    if (m_value > m_max) m_value = m_max;
}

void ChangeSetting::updateLCD()
{
    // 자리수 자동(음수/3자리 대응)
    int absVal = qAbs(m_value);
    int digits = 1;
    if (absVal >= 10)  digits = 2;
    if (absVal >= 100) digits = 3;
    if (m_value < 0)   digits += 1;  // '-' 포함

    ui->set->setDigitCount(digits);
    ui->set->display(m_value);
}
