#ifndef TAB3_CCTV_H
#define TAB3_CCTV_H

#include <QWidget>
#include <QTimer>
#include <opencv2/opencv.hpp>

namespace Ui {
class Tab3_cctv;
}

class Tab3_cctv : public QWidget
{
    Q_OBJECT

public:
    explicit Tab3_cctv(QWidget *parent = nullptr);
    ~Tab3_cctv();

private:
    Ui::Tab3_cctv *ui;
    QTimer *camTimer;
    cv::VideoCapture cap;

signals:
    void goToHome();
private slots:
    void on_pPBcam_toggled(bool checked);
    void updateCameraFrame();
};

#endif // TAB3_CCTV_H
