#ifndef TAB2_SET_H
#define TAB2_SET_H

#include <QWidget>
#include <QTimer>
#include <QString>

#include "changesetting.h"   // ChangeSetting::Mode 사용

QT_BEGIN_NAMESPACE
namespace Ui { class Tab2_set; }
QT_END_NAMESPACE

class ChangeSetting;

class Tab2_set : public QWidget
{
    Q_OBJECT
public:
    explicit Tab2_set(QWidget *parent = nullptr);
    ~Tab2_set();

signals:
    void goToHome();
    void sendToServer(const QString& msg);

public slots:
    void onSocketMessage(const QString& msg);

private slots:
    // 버튼 핸들러
    void on_pPBAtemp_clicked();
    void on_pPBAhumi_clicked();
    void on_pPBair_clicked();
    void on_pPBShumi_clicked();
    void on_pPBec_clicked();
    void on_pPBph_clicked();
    void on_pPBled_clicked();
    void on_pPBtime_clicked();

    // 변경 다이얼로그 OK
    void onSettingDecided(ChangeSetting::Mode mode, int value);

    // 스케줄(싱글샷) 타이머 콜백
    void onScheduledFire();

private:
    Ui::Tab2_set *ui = nullptr;
    ChangeSetting* mChangeDlg = nullptr;

    // 현재 보관값
    int m_airTemp   = 22;
    int m_airHumi   = 50;
    int m_illu      = 300;   // 파일에 없으면 기본값 유지
    int m_air       = 1;
    int m_soilHumi  = 35;
    int m_ec        = 2;
    int m_ph        = 7;
    int m_ledLevel  = 1;     // 0:OFF 1:LOW 2:MID 3:HIGH
    int m_timeMin   = 540;   // 분(예: 09:00)

    // 설정 파일 I/O
    void loadFromFile();
    void saveToFile() const;

    // UI 반영
    void refreshUiFromCurrent();

    // 서버 수신 파싱
    bool parseCCTV01(const QString& msg);

    // 다이얼로그 유틸
    void openSetting(ChangeSetting::Mode mode, int current, int minVal, int maxVal);
    void showOverlay(ChangeSetting& dlg, QWidget* host = nullptr);

    // 모드→토픽 매핑
    struct Topic { QString domain; QString key; };
    Topic topicForMode(ChangeSetting::Mode m) const;

    // 싱글샷 스케줄러
    QTimer m_oneShot;              // 다음 트리거까지 한 번만 대기
    void scheduleNextFire();       // m_timeMin 기준으로 다음 발생 예약
};

#endif // TAB2_SET_H
