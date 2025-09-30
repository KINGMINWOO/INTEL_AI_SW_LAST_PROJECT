#include "tab_widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Tab_widget w;
    w.show();
    return a.exec();
}
