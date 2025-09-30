#include "tab3_temhumiilludb.h"
#include "ui_tab3_temhumiilludb.h"
#include <QDebug>

Tab3_TemHumiIlluDB::Tab3_TemHumiIlluDB(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab3_TemHumiIlluDB)
{
    ui->setupUi(this);

    // DB 초기화
    initDatabase();

    // 슬롯 연결
    connect(ui->pPBthi, &QPushButton::clicked, this, &Tab3_TemHumiIlluDB::on_pPBthi_clicked);
    connect(ui->pPBclearthi, &QPushButton::clicked, this, &Tab3_TemHumiIlluDB::on_pPBclearthi_clicked);
}

Tab3_TemHumiIlluDB::~Tab3_TemHumiIlluDB()
{
    delete ui;
}

// DB 연결 및 테이블 생성
void Tab3_TemHumiIlluDB::initDatabase()
{
    qSqlDatabase = QSqlDatabase::addDatabase("QSQLITE");
    qSqlDatabase.setDatabaseName("aiot.db");
    if(qSqlDatabase.open())
        qDebug() << "Success DB Connection";
    else
        qDebug() << "Fail DB Connection";

    QString strQuery = "CREATE TABLE IF NOT EXISTS sensor_tb ("
                       "ID VARCHAR(20),"
                       "Date DATETIME PRIMARY KEY,"
                       "Temperature INT,"
                       "Illuminance INT,"
                       "Humidity INT)";
    QSqlQuery sqlQuery;
    sqlQuery.exec(strQuery);
}

// 조회 버튼 클릭
void Tab3_TemHumiIlluDB::on_pPBthi_clicked()
{
    QDateTime startDT = ui->pStartdatetime_thi->dateTime();
    QDateTime endDT   = ui->pEnddatetime_thi->dateTime();

    QString strQuery = QString("SELECT * FROM sensor_tb WHERE '%1' <= Date AND Date <= '%2'")
                        .arg(startDT.toString("yyyy/MM/dd hh:mm:ss"))
                        .arg(endDT.toString("yyyy/MM/dd hh:mm:ss"));

    QSqlQuery sqlQuery;
    if(sqlQuery.exec(strQuery))
    {
        // 테이블 초기화
        ui->pDBthi->clearContents();
        int rowCount = 0;
        while(sqlQuery.next()) rowCount++;

        // 기존 아이템 삭제
        if(pQTableWidgetItemId) { delete [] pQTableWidgetItemId; pQTableWidgetItemId=nullptr; }
        if(pQTableWidgetItemDate) { delete [] pQTableWidgetItemDate; pQTableWidgetItemDate=nullptr; }
        if(pQTableWidgetItemTemp) { delete [] pQTableWidgetItemTemp; pQTableWidgetItemTemp=nullptr; }
        if(pQTableWidgetItemIllu) { delete [] pQTableWidgetItemIllu; pQTableWidgetItemIllu=nullptr; }
        if(pQTableWidgetItemHum) { delete [] pQTableWidgetItemHum; pQTableWidgetItemHum=nullptr; }

        pQTableWidgetItemId = new QTableWidgetItem[rowCount];
        pQTableWidgetItemDate = new QTableWidgetItem[rowCount];
        pQTableWidgetItemTemp = new QTableWidgetItem[rowCount];
        pQTableWidgetItemIllu = new QTableWidgetItem[rowCount];
        pQTableWidgetItemHum = new QTableWidgetItem[rowCount];

        sqlQuery.first();
        int i=0;
        while(sqlQuery.next())
        {
            ui->pDBthi->setRowCount(i+1);

            (pQTableWidgetItemId+i)->setText(sqlQuery.value("ID").toString());
            (pQTableWidgetItemDate+i)->setText(sqlQuery.value("Date").toString());
            (pQTableWidgetItemTemp+i)->setText(sqlQuery.value("Temperature").toString());
            (pQTableWidgetItemIllu+i)->setText(sqlQuery.value("Illuminance").toString());
            (pQTableWidgetItemHum+i)->setText(sqlQuery.value("Humidity").toString());

            ui->pDBthi->setItem(i,0,(pQTableWidgetItemId+i));
            ui->pDBthi->setItem(i,1,(pQTableWidgetItemDate+i));
            ui->pDBthi->setItem(i,2,(pQTableWidgetItemTemp+i));
            ui->pDBthi->setItem(i,3,(pQTableWidgetItemIllu+i));
            ui->pDBthi->setItem(i,4,(pQTableWidgetItemHum+i));

            i++;
        }

        // 컬럼 너비 자동 조정
        for(int c=0;c<5;c++)
            ui->pDBthi->resizeColumnToContents(c);
    }
}

// 초기화 버튼 클릭
void Tab3_TemHumiIlluDB::on_pPBclearthi_clicked()
{
    ui->pDBthi->clearContents();
    ui->pDBthi->setRowCount(0);
}
