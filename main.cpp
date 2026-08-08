#include "mainwindow7.h"
#include <QApplication>



int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow7 w;

    w.setMinimumSize(1080, 720);

    w.show();

    return a.exec();
}
