#ifndef CHANGESETTING_H
#define CHANGESETTING_H

#include <QDialog>

namespace Ui { class ChangeSetting; }

class ChangeSetting : public QDialog
{
    Q_OBJECT
public:
    enum class Mode { Temp, Humi, Illu, Air, SoilTemp, SoilHumi, EC, PH };

    explicit ChangeSetting(QWidget *parent = nullptr);
    ~ChangeSetting();

    // 버튼별로 다른 항목 설정
    void setMode(Mode m, int value, int minVal, int maxVal);

    int  value() const { return m_value; }
    Mode mode()  const { return m_mode;  }

signals:
    void valueChanged(int v);                           // 증감 중 실시간
    void decided(ChangeSetting::Mode mode, int value);  // OK 눌렀을 때

private slots:
    void onUp();
    void onDown();
    void onAccepted();

private:
    void updateLCD();
    void clamp();

private:
    Ui::ChangeSetting *ui;
    Mode m_mode{Mode::Temp};
    int  m_value{0};
    int  m_min{0};
    int  m_max{100};
};
#endif
