#ifndef TAB3_TEMHUMIILLUDB_H
#define TAB3_TEMHUMIILLUDB_H

#include <QWidget>
#include <QtSql>
#include <QTableWidgetItem>

namespace Ui {
class Tab3_TemHumiIlluDB;
}

class Tab3_TemHumiIlluDB : public QWidget
{
    Q_OBJECT

public:
    explicit Tab3_TemHumiIlluDB(QWidget *parent = nullptr);
    ~Tab3_TemHumiIlluDB();

private slots:
    void on_pPBthi_clicked();       // 조회 버튼
    void on_pPBclearthi_clicked();  // 초기화 버튼

private:
    Ui::Tab3_TemHumiIlluDB *ui;
    QSqlDatabase qSqlDatabase;

    // 테이블 위젯 아이템 동적 생성
    QTableWidgetItem *pQTableWidgetItemId = nullptr;
    QTableWidgetItem *pQTableWidgetItemDate = nullptr;
    QTableWidgetItem *pQTableWidgetItemTemp = nullptr;
    QTableWidgetItem *pQTableWidgetItemIllu = nullptr;
    QTableWidgetItem *pQTableWidgetItemHum = nullptr;

    void initDatabase();
};

#endif // TAB3_TEMHUMIILLUDB_H
