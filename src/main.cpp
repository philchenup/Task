#include "MvCameraWorker.h"
#include "MainWindow.h"

#include <QApplication>
#include <QMetaType>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    qRegisterMetaType<cv::Mat>("cv::Mat");

    MainWindow window;
    window.show();
    return app.exec();
}
