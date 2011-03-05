#include <QApplication>
#include <windows.h>
#include "mainwindow.h"
#include <QSplashScreen>
#include <Qt>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
