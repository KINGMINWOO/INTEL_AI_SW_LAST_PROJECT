#include "tab2_cropstatus.h"
#include "ui_tab2_cropstatus.h"
#include <QDebug>

Tab2_CropStatus::Tab2_CropStatus(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab2_CropStatus)
{
    ui->setupUi(this);

    // DB 초기화
    initDatabase();

    // 슬롯 연결
    connect(ui->pPBcrop, &QPushButton::clicked, this, &Tab2_CropStatus::on_pPBcrop_clicked);
    connect(ui->pPBclearcrop, &QPushButton::clicked, this, &Tab2_CropStatus::on_pPBclearcrop_clicked);
}

Tab2_CropStatus::~Tab2_CropStatus()
{
    delete ui;
}

// DB 연결 및 테이블 생성
void Tab2_CropStatus::initDatabase()
{
    qSqlDatabase = QSqlDatabase::addDatabase("QSQLITE");
    qSqlDatabase.setDatabaseName("aiot.db");
    if(qSqlDatabase.open())
        qDebug() << "Success DB Connection";
    else
        qDebug() << "Fail DB Connection";

    // 테이블 생성 (없으면)
    QString strQuery = "CREATE TABLE IF NOT EXISTS tomato_tb ("
                       "id VARCHAR(20),"
                       "date DATETIME PRIMARY KEY,"
                       "Fresh INT,"
                       "Unripe INT,"
                       "Rotten INT)";

    QSqlQuery sqlQuery;
    sqlQuery.exec(strQuery);
}

// 조회 버튼 클릭
void Tab2_CropStatus::on_pPBcrop_clicked()
{
    QDateTime startDT = ui->pStartdatetime_crop->dateTime();
    QDateTime endDT   = ui->pEnddatetime_crop->dateTime();

    QString strQuery = QString("SELECT * FROM tomato_tb WHERE '%1' <= date AND date <= '%2'")
                        .arg(startDT.toString("yyyy/MM/dd hh:mm:ss"))
                        .arg(endDT.toString("yyyy/MM/dd hh:mm:ss"));

    QSqlQuery sqlQuery;
    if(sqlQuery.exec(strQuery))
    {
        // 기존 테이블 초기화
        ui->pTWcrop->clearContents();
        int rowCount = 0;
        while(sqlQuery.next()) rowCount++;

        // 기존 아이템 삭제
        if(pQTableWidgetItemId) { delete [] pQTableWidgetItemId; pQTableWidgetItemId=nullptr; }
        if(pQTableWidgetItemDate) { delete [] pQTableWidgetItemDate; pQTableWidgetItemDate=nullptr; }
        if(pQTableWidgetItemFresh) { delete [] pQTableWidgetItemFresh; pQTableWidgetItemFresh=nullptr; }
        if(pQTableWidgetItemUnripe) { delete [] pQTableWidgetItemUnripe; pQTableWidgetItemUnripe=nullptr; }
        if(pQTableWidgetItemRotten) { delete [] pQTableWidgetItemRotten; pQTableWidgetItemRotten=nullptr; }

        pQTableWidgetItemId = new QTableWidgetItem[rowCount];
        pQTableWidgetItemDate = new QTableWidgetItem[rowCount];
        pQTableWidgetItemFresh = new QTableWidgetItem[rowCount];
        pQTableWidgetItemUnripe = new QTableWidgetItem[rowCount];
        pQTableWidgetItemRotten = new QTableWidgetItem[rowCount];

        sqlQuery.first();
        int i=0;
        while(sqlQuery.next())
        {
            ui->pTWcrop->setRowCount(i+1);

            (pQTableWidgetItemId+i)->setText(sqlQuery.value("ID").toString());  // 여기
            (pQTableWidgetItemDate+i)->setText(sqlQuery.value("Date").toString());
            (pQTableWidgetItemFresh+i)->setText(sqlQuery.value("Fresh").toString());
            (pQTableWidgetItemUnripe+i)->setText(sqlQuery.value("Unripe").toString());
            (pQTableWidgetItemRotten+i)->setText(sqlQuery.value("Rotten").toString());

            ui->pTWcrop->setItem(i,0,(pQTableWidgetItemId+i));
            ui->pTWcrop->setItem(i,1,(pQTableWidgetItemDate+i));
            ui->pTWcrop->setItem(i,2,(pQTableWidgetItemFresh+i));
            ui->pTWcrop->setItem(i,3,(pQTableWidgetItemUnripe+i));
            ui->pTWcrop->setItem(i,4,(pQTableWidgetItemRotten+i));

            i++;
        }

        // 컬럼 너비 자동 조정
        for(int c=0;c<5;c++)
            ui->pTWcrop->resizeColumnToContents(c);
    }
}

// 초기화 버튼 클릭
void Tab2_CropStatus::on_pPBclearcrop_clicked()
{
    ui->pTWcrop->clearContents();
    ui->pTWcrop->setRowCount(0);
}
