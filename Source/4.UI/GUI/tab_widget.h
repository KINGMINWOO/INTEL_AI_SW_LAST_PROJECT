#ifndef TAB_WIDGET_H
#define TAB_WIDGET_H

#include <QWidget>
#include "tab1_control.h"
#include "tab2_cropstatus.h"
#include "tab3_temhumiilludb.h"
#include "tab4_camera.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Tab_widget; }
QT_END_NAMESPACE

class Tab_widget : public QWidget
{
    Q_OBJECT

public:
    Tab_widget(QWidget *parent = nullptr);
    ~Tab_widget();

private:
    Ui::Tab_widget *ui;
    Tab1_control *pTab1_control;
    Tab2_CropStatus *pTab2_CropStatus;
    Tab3_TemHumiIlluDB *pTab3_TemHumiIlluDB;
    Tab4_Camera *pTab4_Camera;
};
#endif // TAB_WIDGET_H
