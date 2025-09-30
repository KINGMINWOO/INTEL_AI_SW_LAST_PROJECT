#ifndef TAB2_CROPSTATUS_H
#define TAB2_CROPSTATUS_H

#include <QWidget>
#include <QtSql>
#include <QTableWidgetItem>

namespace Ui {
class Tab2_CropStatus;
}

class Tab2_CropStatus : public QWidget
{
    Q_OBJECT

public:
    explicit Tab2_CropStatus(QWidget *parent = nullptr);
    ~Tab2_CropStatus();

private slots:
    void on_pPBcrop_clicked();       // 조회 버튼
    void on_pPBclearcrop_clicked();  // 초기화 버튼

private:
    Ui::Tab2_CropStatus *ui;
    QSqlDatabase qSqlDatabase;

    // 테이블 위젯 아이템 동적 생성
    QTableWidgetItem *pQTableWidgetItemId = nullptr;
    QTableWidgetItem *pQTableWidgetItemDate = nullptr;
    QTableWidgetItem *pQTableWidgetItemFresh = nullptr;
    QTableWidgetItem *pQTableWidgetItemUnripe = nullptr;
    QTableWidgetItem *pQTableWidgetItemRotten = nullptr;

    void initDatabase();
};

#endif // TAB2_CROPSTATUS_H
