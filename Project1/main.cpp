#if _MSC_VER >= 1600
#pragma execution_character_set("utf-8")
#endif

#include <QApplication>
#include <QTextCodec>
#include "mainwindow.h"

int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
#endif

    MainWindow w;
    w.show();
    return a.exec();
}