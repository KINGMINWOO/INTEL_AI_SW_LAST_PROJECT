#ifndef TAB2_SET_H
#define TAB2_SET_H

#include <QWidget>
#include "changesetting.h"

namespace Ui { class Tab2_set; }

class Tab2_set : public QWidget
{
    Q_OBJECT
public:
    explicit Tab2_set(QWidget *parent = nullptr);
    ~Tab2_set();

signals:
    // 부모(또는 상위 탭/위젯)에서 SocketClient::socketWriteDataSlot(QString)에 연결
    void sendToServer(const QString& msg);

private slots:
    void on_pPBAtemp_clicked();
    void on_pPBAhumi_clicked();
    void on_pPBillu_clicked();
    void on_pPBair_clicked();
    void on_pPBStemp_clicked();
    void on_pPBShumi_clicked();
    void on_pPBec_clicked();
    void on_pPBph_clicked();

    void goToHome(); // 기존 연결 유지용

    // ChangeSetting OK 신호 처리
    void onSettingDecided(ChangeSetting::Mode mode, int value);

private:
    void openSetting(ChangeSetting::Mode mode, int current, int minVal, int maxVal);
    void showOverlay(ChangeSetting &dlg, QWidget *host = nullptr);
    QString tagForMode(ChangeSetting::Mode m) const;

private:
    Ui::Tab2_set *ui;
    ChangeSetting *mChangeDlg { nullptr };   // 단 하나만 공유
};
#endif
