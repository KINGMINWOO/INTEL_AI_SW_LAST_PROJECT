#include "tab4_tomato.h"
#include "ui_tab4_tomato.h"

Tab4_tomato::Tab4_tomato(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab4_tomato)
{
    ui->setupUi(this);
    connect(ui->pPBhome, &QPushButton::clicked, this, &Tab4_tomato::goToHome);
}

Tab4_tomato::~Tab4_tomato()
{
    delete ui;
}
