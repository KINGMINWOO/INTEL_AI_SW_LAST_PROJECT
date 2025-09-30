#include "tab_widget.h"
#include "ui_tab_widget.h"

Tab_widget::Tab_widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab_widget)
{
    ui->setupUi(this);
    pTab1_control = new Tab1_control(ui->pTab1);
    ui->pTab1->setLayout(pTab1_control->layout());

    pTab2_CropStatus = new Tab2_CropStatus(ui->pTab2);
    ui->pTab2->setLayout(pTab2_CropStatus->layout());

    pTab3_TemHumiIlluDB = new Tab3_TemHumiIlluDB(ui->pTab3);
    ui->pTab3->setLayout(pTab3_TemHumiIlluDB->layout());

    pTab4_Camera = new Tab4_Camera(ui->pTab4);
    ui->pTab4->setLayout(pTab4_Camera->layout());
}

Tab_widget::~Tab_widget()
{
    delete ui;
}

