#include "MainWindow.h"
#include "Types.h"

#include <QApplication>
#include <QPalette>

int main(int argc, char *argv[])
{
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);
    QApplication::setStyle(QStringLiteral("Fusion"));
    app.setFont(uiFont(13));
    app.setApplicationName(QStringLiteral("流程编辑器"));

    QPalette palette;
    palette.setColor(QPalette::Window, QColor("#1e1e1e"));
    palette.setColor(QPalette::WindowText, QColor("#d8d8d8"));
    palette.setColor(QPalette::Base, QColor("#252628"));
    palette.setColor(QPalette::AlternateBase, QColor("#222426"));
    palette.setColor(QPalette::Text, QColor("#d8d8d8"));
    palette.setColor(QPalette::Button, QColor("#2d3034"));
    palette.setColor(QPalette::ButtonText, QColor("#d8d8d8"));
    palette.setColor(QPalette::Highlight, QColor("#245e66"));
    palette.setColor(QPalette::HighlightedText, QColor("#ffffff"));
    palette.setColor(QPalette::ToolTipBase, QColor("#2a2d31"));
    palette.setColor(QPalette::ToolTipText, QColor("#e8e8e8"));
    palette.setColor(QPalette::PlaceholderText, QColor("#8b9096"));
    app.setPalette(palette);

    MainWindow window;
    window.show();
    return app.exec();
}
