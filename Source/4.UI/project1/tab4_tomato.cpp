#include "tab4_tomato.h"
#include "ui_tab4_tomato.h"

#include <QDebug>
#include <QtSql/QSqlError>
#include <QtSql/QSqlQuery>
#include <QMessageBox>
#include <QDateTime>

Tab4_tomato::Tab4_tomato(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab4_tomato)
{
    ui->setupUi(this);

    connect(ui->pPBhome, &QPushButton::clicked, this, &Tab4_tomato::goToHome);

    // 날짜 제약: 오늘까지
    const QDate today = QDate::currentDate();
    ui->pStartdate->setMaximumDate(today);
    ui->pLastdate->setMaximumDate(today);

    connect(ui->pStartdate, &QDateEdit::dateChanged, this, [this](const QDate& d){
        ui->pLastdate->setMinimumDate(d);
        if (ui->pLastdate->date() < d)
            ui->pLastdate->setDate(d);
    });
    ui->pLastdate->setMinimumDate(ui->pStartdate->date());
    if (ui->pLastdate->date() < ui->pStartdate->date())
        ui->pLastdate->setDate(ui->pStartdate->date());

    // 검색 버튼
    connect(ui->pPBsearch, &QPushButton::clicked, this, &Tab4_tomato::on_pPBsearch_clicked);
}

Tab4_tomato::~Tab4_tomato()
{
    if (mDb.isValid() && mDb.isOpen())
        mDb.close();
    delete ui;
}

void Tab4_tomato::setDbParams(const QString& host, int port,
                              const QString& dbName, const QString& user, const QString& password)
{
    mHost = host; mPort = port; mDbName = dbName; mUser = user; mPass = password;
}

bool Tab4_tomato::ensureConnection()
{
    // 이미 열려 있으면 재사용
    if (mDb.isValid() && mDb.isOpen())
        return true;

    // 고유 커넥션명
    const QString connName = "tomato_ro";
    if (QSqlDatabase::contains(connName)) {
        mDb = QSqlDatabase::database(connName);
    } else {
        mDb = QSqlDatabase::addDatabase("QMYSQL", connName);
    }

    mDb.setHostName(mHost);
    mDb.setPort(mPort);
    mDb.setDatabaseName(mDbName);
    mDb.setUserName(mUser);
    mDb.setPassword(mPass);

    if (!mDb.open()) {
        qWarning() << "[Tab4_tomato] DB open failed:" << mDb.lastError().text();
        QMessageBox::warning(this, tr("DB 연결 실패"),
                             tr("MariaDB 연결에 실패했습니다.\n%1").arg(mDb.lastError().text()));
        return false;
    }
    return true;
}

void Tab4_tomato::on_pPBsearch_clicked()
{
    if (!ensureConnection()) return;

    const QDate sDate = ui->pStartdate->date();
    const QDate eDate = ui->pLastdate->date();
    runQueryAndApply(sDate, eDate);
}

static inline QString betweenClause(const QString& tsCol,
                                    bool withDevice,
                                    const QString& deviceCol)
{
    // captured_at BETWEEN :s AND :e [AND device_id=:dev]
    QString where = QString("%1 BETWEEN :s AND :e").arg(tsCol);
    if (withDevice) {
        where += QString(" AND %1 = :dev").arg(deviceCol);
    }
    return where;
}

void Tab4_tomato::runQueryAndApply(const QDate& sDate, const QDate& eDate)
{
    const QDateTime sdt(sDate, QTime(0,0,0));
    const QDateTime edt(eDate, QTime(23,59,59));
    const bool useDevice = !mDeviceId.isEmpty();

    // ========================= (1) 토마토 익음/썩음 합계
    int fresh = 0, rotten = 0;
    {
        const QString sql =
            QString("SELECT "
                    "COALESCE(SUM(%1),0) AS fresh, "
                    "COALESCE(SUM(%2),0) AS rotten "
                    "FROM %3 "
                    "WHERE %4")
            .arg(mSchema.tomatoRipe,
                 mSchema.tomatoRotten,
                 mSchema.tomatoTable,
                 betweenClause(mSchema.tomatoTs, useDevice, mSchema.deviceCol));

        QSqlQuery q(mDb);
        q.prepare(sql);
        q.bindValue(":s", sdt);
        q.bindValue(":e", edt);
        if (useDevice) q.bindValue(":dev", mDeviceId);

        if (!q.exec()) {
            qWarning() << "[Tab4_tomato] tomato sum query failed:" << q.lastError().text();
        } else if (q.next()) {
            fresh  = q.value("fresh").toInt();
            rotten = q.value("rotten").toInt();
        }
    }
    applyTomatoCounts(fresh, rotten);

    // ========================= (2) 환경 평균 - 공기
    float air_temp = 0.f, air_humi = 0.f;   // ★ float
    int air_quality = 0, illu = 0;          // ★ int
    {
        const QString sql =
            QString("SELECT "
                    "AVG(%1) AS air_temp, "
                    "AVG(%2) AS air_humi, "
                    "AVG(%3) AS air_quality, "
                    "AVG(%4) AS illu "
                    "FROM %5 "
                    "WHERE %6")
            .arg(mSchema.airTemp,
                 mSchema.airHumi,
                 mSchema.airQuality,
                 mSchema.illu,
                 mSchema.airTable,
                 betweenClause(mSchema.airTs, useDevice, mSchema.deviceCol));

        QSqlQuery q(mDb);
        q.prepare(sql);
        q.bindValue(":s", sdt);
        q.bindValue(":e", edt);
        if (useDevice) q.bindValue(":dev", mDeviceId);

        if (!q.exec()) {
            qWarning() << "[Tab4_tomato] air avg query failed:" << q.lastError().text();
        } else if (q.next()) {
            air_temp    = q.value("air_temp").toFloat();
            air_humi    = q.value("air_humi").toFloat();
            air_quality = q.value("air_quality").toInt();
            illu        = q.value("illu").toInt();
        }
    }

    // ========================= (3) 환경 평균 - 토양
    float land_temp = 0.f, land_humi = 0.f, soil_ph = 0.f;  // ★ float
    int soil_ec = 0;                                        // ★ int
    {
        const QString sql =
            QString("SELECT "
                    "AVG(%1) AS land_temp, "
                    "AVG(%2) AS land_humi, "
                    "AVG(%3) AS soil_ec, "
                    "AVG(%4) AS soil_ph "
                    "FROM %5 "
                    "WHERE %6")
            .arg(mSchema.landTemp,
                 mSchema.landHumi,
                 mSchema.ec,
                 mSchema.ph,
                 mSchema.landTable,
                 betweenClause(mSchema.landTs, useDevice, mSchema.deviceCol));

        QSqlQuery q(mDb);
        q.prepare(sql);
        q.bindValue(":s", sdt);
        q.bindValue(":e", edt);
        if (useDevice) q.bindValue(":dev", mDeviceId);

        if (!q.exec()) {
            qWarning() << "[Tab4_tomato] land avg query failed:" << q.lastError().text();
        } else if (q.next()) {
            land_temp = q.value("land_temp").toFloat();
            land_humi = q.value("land_humi").toFloat();
            soil_ec   = q.value("soil_ec").toInt();
            soil_ph   = q.value("soil_ph").toFloat();
        }
    }

    applyEnvAverages(air_temp, land_temp, air_humi, land_humi,
                     air_quality, soil_ec, illu, soil_ph);
}

void Tab4_tomato::applyTomatoCounts(int fresh, int rotten)
{
    const int total = fresh + rotten;
    if (ui->pLblfresh)  ui->pLblfresh->setText(QString::number(fresh));
    if (ui->pLblrotten) ui->pLblrotten->setText(QString::number(rotten));
    const int percent = (total > 0) ? qRound(100.0 * fresh / total) : 0;
    if (ui->pLblpercent) ui->pLblpercent->setText(QString::number(percent) + "%");
}

void Tab4_tomato::applyEnvAverages(float air_temp, float land_temp, float air_humi,
                                   float land_humi, int air, int ec, int illu, float ph)
{
    // 모든 항목을 소수점 한 자리('f', 1)로 포맷
    if (ui->pLcdairtemp)  ui->pLcdairtemp->display(QString::number(air_temp, 'f', 1));
    if (ui->pLcdlandtemp) ui->pLcdlandtemp->display(QString::number(land_temp, 'f', 1));
    if (ui->pLcdairhumi)  ui->pLcdairhumi->display(QString::number(air_humi, 'f', 1));
    if (ui->pLcdlandhumi) ui->pLcdlandhumi->display(QString::number(land_humi, 'f', 1));
    if (ui->pLcdph)       ui->pLcdph->display(QString::number(ph, 'f', 1));
    if (ui->pLcdair)      ui->pLcdair->display(QString::number(static_cast<double>(air),  'f', 1));
    if (ui->pLcdec)       ui->pLcdec->display(QString::number(static_cast<double>(ec),   'f', 1));
    if (ui->pLcdairillu)  ui->pLcdairillu->display(QString::number(static_cast<double>(illu), 'f', 1));
}
