#include "tab1_button.h"
#include "ui_tab1_button.h"

Tab1_button::Tab1_button(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab1_button)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Tab1_button::updateDateTime);
    timer->start(1000);

    updateDateTime();

    connect(ui->pPBset, &QPushButton::clicked,
            this, &Tab1_button::goToTab2);

    connect(ui->pPBcctv, &QPushButton::clicked,
            this, &Tab1_button::goToTab3);

    connect(ui->pPBtomato, &QPushButton::clicked,
            this, &Tab1_button::goToTab4);
}

Tab1_button::~Tab1_button()
{
    delete ui;
}

void Tab1_button::updateDateTime()
{
    QDateTime current = QDateTime::currentDateTime();

    if (ui->pTime) {
        ui->pTime->display(current.time().toString("HH:mm:ss"));
    }

    if (ui->pDate) {
        ui->pDate->setText(current.date().toString("yyyy-MM-dd (ddd)"));
    }
}
