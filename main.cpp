#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowTitle("Parallel Scara Robot Parameters Generator");
    w.show();
    return a.exec();
}
