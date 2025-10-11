#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include "tab1_button.h"
#include "tab2_set.h"
#include "tab3_cctv.h"
#include "tab4_tomato.h"
#include "socketclient.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWidget;
}
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

private:
    Ui::MainWidget *ui;
    Tab1_button *pTab1_button;
    Tab2_set *pTab2_set;
    Tab3_cctv *pTab3_cctv;
    Tab4_tomato *pTab4_tomato;
    SocketClient *pSocketClient;
};
#endif // MAINWIDGET_H
