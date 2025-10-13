#ifndef CHANGESETTING_H
#define CHANGESETTING_H

#include <QDialog>

namespace Ui { class ChangeSetting; }

class ChangeSetting : public QDialog
{
    Q_OBJECT
public:
    enum class Mode {
        Temp, Humi, Air,
        SoilHumi, EC, PH,
        LED, TIME
    };

    explicit ChangeSetting(QWidget *parent = nullptr);
    ~ChangeSetting();

    // 숫자 모드: current/min/max 사용, LED 모드: current(0~3)만 사용
    void setMode(Mode m, int current, int minVal, int maxVal);

signals:
    void decided(ChangeSetting::Mode mode, int value); // OK/확인 시 최종 값
    void valueChanged(int value); // Up/Down 시 실시간 브로드캐스트(옵션)
    void sendCommand(const QString& line);

private slots:
    void onUp();
    void onDown();
    void onAccepted();

private:
    void clamp();        // 숫자 모드 범위 보정
    void updateView();   // LCD or LED 라벨 업데이트
    void updateTimeFromDial(int dialVal);

private:
    Ui::ChangeSetting *ui;
    Mode  m_mode { Mode::Temp };

    // 숫자 모드
    int   m_value { 0 };
    int   m_min   { 0 };
    int   m_max   { 100 };

    // LED 모드 전용 (0:OFF, 1:LOW, 2:MID, 3:HIGH)
    int   m_led   { 0 };
};

#endif // CHANGESETTING_H
