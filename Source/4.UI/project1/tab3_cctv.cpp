#include "tab3_cctv.h"
#include "ui_tab3_cctv.h"

#include <QFileDialog>
#include <QDateTime>
#include <QPixmap>
#include <QDir>
#include <QImage>
#include <QDebug>
#include <opencv2/opencv.hpp>

Tab3_cctv::Tab3_cctv(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab3_cctv)
{
    ui->setupUi(this);

    ui->pPBcam->setText("Camera ON");

    camTimer = new QTimer(this);
    connect(camTimer, &QTimer::timeout, this, &Tab3_cctv::updateCameraFrame);

    connect(ui->pPBcam, &QPushButton::toggled,
            this, &Tab3_cctv::on_pPBcam_toggled);

    connect(ui->pPBhome, &QPushButton::clicked,
            this, &Tab3_cctv::goToHome);
}

Tab3_cctv::~Tab3_cctv()
{
    delete ui;
    cap.release();
    camTimer->stop();
}

void Tab3_cctv::on_pPBcam_toggled(bool checked)
{

    if (checked) {
        ui->pPBcam->setText("Camera OFF");

        // MJPG-Streamer 스트림 열기
        cap.open("http://10.10.16.60:8080/?action=stream");
        if (cap.isOpened())
            camTimer->start(30); // 30ms마다 갱신
        else
            qDebug() << "Failed to open camera stream!";
    } else {
        ui->pPBcam->setText("Camera ON");
        camTimer->stop();
        cap.release();
        ui->pScreen->clear();
    }
}

void Tab3_cctv::updateCameraFrame()
{
    cv::Mat frame;
    if (!cap.read(frame))
        return;

    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
    ui->pScreen->setPixmap(QPixmap::fromImage(img).scaled(
        ui->pScreen->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
