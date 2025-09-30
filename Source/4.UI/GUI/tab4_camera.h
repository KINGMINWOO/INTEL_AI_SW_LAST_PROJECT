#ifndef TAB4_CAMERA_H
#define TAB4_CAMERA_H

#include <QWidget>
#include <QTimer>
#include <opencv2/opencv.hpp>  // OpenCV

namespace Ui {
class Tab4_Camera;
}

class Tab4_Camera : public QWidget
{
    Q_OBJECT

public:
    explicit Tab4_Camera(QWidget *parent = nullptr);
    ~Tab4_Camera();

private slots:
    void on_pPBcam_toggled(bool checked);
    void on_pPBcapture_clicked();  // pPBcapture 클릭 슬롯
    void updateCameraFrame();      // 타이머로 영상 갱신

private:
    Ui::Tab4_Camera *ui;
    QTimer *camTimer;
    cv::VideoCapture cap;
};

#endif // TAB4_CAMERA_H
