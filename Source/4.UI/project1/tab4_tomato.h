#ifndef TAB4_TOMATO_H
#define TAB4_TOMATO_H

#include <QWidget>
#include <QDate>
#include <QtSql/QSqlDatabase>

namespace Ui { class Tab4_tomato; }

class Tab4_tomato : public QWidget
{
    Q_OBJECT
public:
    explicit Tab4_tomato(QWidget *parent = nullptr);
    ~Tab4_tomato();

    // DB 접속 파라미터 주입 (읽기 전용 계정 권장)
    void setDbParams(const QString& host,
                     int port,
                     const QString& dbName,
                     const QString& user,
                     const QString& password);

    // (선택) 특정 디바이스만 보고 싶으면 설정
    void setDeviceIdFilter(const QString& deviceId) { mDeviceId = deviceId.trimmed(); }

    // SQLAlchemy 모델에 맞춘 스키마 매핑(필요하면 교체)
    struct Schema {
        // 공통
        QString deviceCol = "device_id";

        // 1) 토마토 스냅샷(합계)
        QString tomatoTable   = "farm_tomato_snapshots";
        QString tomatoTs      = "captured_at";
        QString tomatoRipe    = "ripe_count";
        QString tomatoRotten  = "rotten_count";

        // 2) 공기(평균)
        QString airTable      = "farm_air_samples";
        QString airTs         = "captured_at";
        QString airTemp       = "temperature_c";
        QString airHumi       = "humidity_pct";
        QString airQuality    = "air_quality";
        QString illu          = "illuminance_lux";

        // 3) 토양(평균)
        QString landTable     = "farm_land_samples";
        QString landTs        = "captured_at";
        QString landTemp      = "soil_temperature_c";
        QString landHumi      = "soil_humidity_pct";
        QString ec            = "soil_ec";
        QString ph            = "soil_ph";
    };
    void setSchema(const Schema& s) { mSchema = s; }

signals:
    void goToHome();

private slots:
    void on_pPBsearch_clicked();

private:
    Ui::Tab4_tomato *ui;
    QSqlDatabase mDb;
    QString mHost, mDbName, mUser, mPass;
    int mPort = 3306;
    Schema mSchema;
    QString mDeviceId; // 선택 필터

    bool ensureConnection();  // 연결 보장(없으면 열고, 있으면 재사용)
    void runQueryAndApply(const QDate& sDate, const QDate& eDate);

    void applyTomatoCounts(int fresh, int rotten);
    void applyEnvAverages(float air_temp, float land_temp, float air_humi, float land_humi,
                          int air, int ec, int illu, float ph);
};

#endif // TAB4_TOMATO_H
