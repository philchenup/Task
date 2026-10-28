#pragma once

#include <QImage>

class QLabel;

// 在界面线程或取图线程里，每次拿到熔池相机的一帧就调用这一个函数：
//
//     setMeltCamImage(ui->meltCamLabel, image);
//
// 函数把 QImage 等比缩进 meltCamLabel，同时留下原始图像。第一次调用时给
// 这个标签装上双击。双击后弹出「熔池相机」放大窗口，窗口里按原图显示；
// 之后每次调用都会同时刷新小图和放大窗口。关闭放大窗口（标题栏、Esc，
// 或再双击放大图）后，小图按标签当前尺寸重新贴上，焦点回到原来的界面。
// 同一标签上的图像都从这里进入，放大窗口才会跟着新帧更新。
void setMeltCamImage(QLabel *meltCamLabel, const QImage &image);
