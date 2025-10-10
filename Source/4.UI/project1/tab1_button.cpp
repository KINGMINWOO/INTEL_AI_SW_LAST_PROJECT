#include "tab1_button.h"
#include "ui_tab1_button.h"

Tab1_button::Tab1_button(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab1_button)
{
    ui->setupUi(this);

    connect(ui->pPBset, &QPushButton::clicked,
            this, &Tab1_button::goToTab2);

    connect(ui->pPBcctv, &QPushButton::clicked,
            this, &Tab1_button::goToTab3);
}

Tab1_button::~Tab1_button()
{
    delete ui;
}
