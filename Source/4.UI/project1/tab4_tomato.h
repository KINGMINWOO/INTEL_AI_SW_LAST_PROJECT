#ifndef TAB4_TOMATO_H
#define TAB4_TOMATO_H

#include <QWidget>

namespace Ui {
class Tab4_tomato;
}

class Tab4_tomato : public QWidget
{
    Q_OBJECT

public:
    explicit Tab4_tomato(QWidget *parent = nullptr);
    ~Tab4_tomato();

private:
    Ui::Tab4_tomato *ui;

signals:
    void goToHome();
private slots:
    void on_pPBsearch_clicked();
};

#endif // TAB4_TOMATO_H
