#include "tab4_tomato.h"
#include "ui_tab4_tomato.h"

#include<QDate>

Tab4_tomato::Tab4_tomato(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Tab4_tomato)
{
    ui->setupUi(this);
    connect(ui->pPBhome, &QPushButton::clicked, this, &Tab4_tomato::goToHome);

    const QDate today = QDate::currentDate();

    ui->pStartdate->setMaximumDate(today);
    ui->pLastdate->setMaximumDate(today);

    connect(ui->pStartdate, &QDateEdit::dateChanged, this,
            [this](const QDate& d){
                ui->pLastdate->setMinimumDate(d);
                if (ui->pLastdate->date() < d)
                    ui->pLastdate->setDate(d);
            });

    ui->pLastdate->setMinimumDate(ui->pStartdate->date());
    if (ui->pLastdate->date() < ui->pStartdate->date())
        ui->pLastdate->setDate(ui->pStartdate->date());
}

Tab4_tomato::~Tab4_tomato()
{
    delete ui;
}

void Tab4_tomato::on_pPBsearch_clicked()
{

}

