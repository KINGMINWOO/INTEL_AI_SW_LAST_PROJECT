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
    void sendToServer(const QString& msg);

private slots:
    // 버튼 핸들러
    void on_pPBAtemp_clicked();
    void on_pPBAhumi_clicked();
    void on_pPBair_clicked();
    void on_pPBShumi_clicked();
    void on_pPBec_clicked();
    void on_pPBph_clicked();
    void on_pPBled_clicked();  // LED 단계 다이얼로그

    void onSettingDecided(ChangeSetting::Mode mode, int value);
    void on_pPBtime_clicked();

signals:
    void goToHome();

private:
    // 공통
    void openSetting(ChangeSetting::Mode mode, int current, int minVal, int maxVal);
    void showOverlay(ChangeSetting &dlg, QWidget *host = nullptr);

    struct Topic { QString domain; QString key; };
    Topic topicForMode(ChangeSetting::Mode m) const;

private:
    Ui::Tab2_set *ui;
    ChangeSetting *mChangeDlg { nullptr };
    int m_ledLevel = 0;
};

#endif // TAB2_SET_H
