#include "tab4_camera.h"
#include "ui_tab4_camera.h"
#include <QFileDialog>
#include <QDateTime>
#include <QPixmap>
#include <QDir>
#include <QImage>
#include <QDebug>
#include <opencv2/opencv.hpp>

Tab4_Camera::Tab4_Camera(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab4_Camera)
{
    ui->setupUi(this);

    ui->pPBcapture->setEnabled(false);
    ui->pPBcam->setText("Camera ON");

    camTimer = new QTimer(this);
    connect(camTimer, &QTimer::timeout, this, &Tab4_Camera::updateCameraFrame);

    connect(ui->pPBcam, &QPushButton::toggled,
            this, &Tab4_Camera::on_pPBcam_toggled);

    connect(ui->pPBcapture, &QPushButton::clicked,
            this, &Tab4_Camera::on_pPBcapture_clicked);
}

Tab4_Camera::~Tab4_Camera()
{
    camTimer->stop();
    cap.release();
    delete ui;
}

void Tab4_Camera::on_pPBcam_toggled(bool checked)
{
    ui->pPBcapture->setEnabled(checked);

    if (checked) {
        ui->pPBcam->setText("Camera OFF");

        // MJPG-Streamer 스트림 열기
        cap.open("http://<IP>:8080/?action=stream"); // 실제 IP로 변경
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

void Tab4_Camera::updateCameraFrame()
{
    cv::Mat frame;
    if (!cap.read(frame))
        return;

    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
    ui->pScreen->setPixmap(QPixmap::fromImage(img).scaled(
        ui->pScreen->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}

void Tab4_Camera::on_pPBcapture_clicked()
{
    QPixmap pixmap = ui->pScreen->grab();
    QString folder = QDir::currentPath(); // 현재 경로에 저장
    QString filename = folder + "/" + QDateTime::currentDateTime()
                       .toString("yyyyMMdd_hhmmss") + ".png";

    if (!pixmap.save(filename, "PNG")) {
        qDebug("Failed to save image!");
    } else {
        qDebug("Image saved: %s", qPrintable(filename));
    }
}
