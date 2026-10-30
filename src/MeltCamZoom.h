#pragma once

#include <QImage>
#include <QString>

class QLabel;

// 任意显示图片的 QLabel 都可以用这一个函数。每次拿到一帧就调用：
//
//     setZoomableImage(ui->weldCamLabel, image, QStringLiteral("焊缝相机"));
//
// zoomTitle 为空时，放大窗口标题用标签的 objectName；objectName 也空则用「图像」。
// 函数把 QImage 等比缩进标签，同时留下原始图像。第一次调用时给这个标签装上双击。
// 双击后弹出该标签自己的放大窗口，窗口里按原图显示；之后每次调用只刷新这一个标签
// 和它的放大窗口。关闭放大窗口（标题栏、Esc，或再双击放大图）后，这个标签的小图
// 按当前尺寸重新贴上，焦点回到原来的界面。其他标签不受影响。
// 可以在界面线程调用，也可以在取图线程调用。
void setZoomableImage(QLabel *label, const QImage &image, const QString &zoomTitle = QString());

// meltCamLabel 的写法。放大窗口标题固定为「熔池相机」。
void setMeltCamImage(QLabel *meltCamLabel, const QImage &image);
