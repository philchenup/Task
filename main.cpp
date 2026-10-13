#include "DecoderWindow.h"

#include <QApplication>

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    DecoderWindow window;
    window.show();
    return app.exec();
}
