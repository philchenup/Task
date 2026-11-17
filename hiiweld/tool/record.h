/**
 * @file record.h
 * @brief 屏幕录制对话框（ui_record）：ROI 截屏 + ffmpeg 编码 MP4。
 */
#ifndef SCREENRECORD_H
#define SCREENRECORD_H

#include <QByteArray>
#include <QDialog>
#include <QRect>
#include <QSize>
#include "base/customdialog.h"

class QCloseEvent;
class QImage;
class QProcess;
class QTimer;
class RoiPicker;

namespace Ui
{
    class Record;
}

/**
 * @brief 全屏/ROI 录屏：开始、暂停、停止后另存 MP4。
 */
class Record : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit Record(QWidget* parent = nullptr);
    ~Record() override;

    virtual void init();
    virtual void reset();
    virtual void deinit();

public slots:
    /** @brief 开始录制；再次调用结束并保存。 */
    void start();
    /** @brief 暂停或继续写帧。 */
    void pause();
    /** @brief 停止录制流程。 */
    void stop();
    /** @brief 鼠标框选录制区域。 */
    void selectRoi();
    /** @brief 退出并关闭对话框。 */
    void exitRecord();

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    friend class RoiPicker;

    void begin();
    bool finishWriter();
    void saveAs();
    void grabFrame();
    void writeFrame(const QImage& image);
    QImage capture() const;
    void applyRoi(const QRect& roi);
    void cancelRoi();
    void updateButtons();

    Ui::Record* ui;
    RoiPicker* m_picker = nullptr;
    QTimer* m_timer = nullptr;
    QProcess* m_ffmpeg = nullptr;
    QRect m_roi;
    QSize m_size;
    QByteArray m_buffer;
    QString m_tempFile;
    bool m_recording = false;
    bool m_paused = false;
    bool m_grabbing = false;
    bool m_saving = false;
};

#endif