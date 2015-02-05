#include <RefBoxWindow.h>
#include <QtGui>
#include <QApplication>

int main(int argc, char **argv) {
    QApplication app(argc, argv);

    msl_refbox::RefBoxWindow w;

    w.show();

    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

    return app.exec();
}
