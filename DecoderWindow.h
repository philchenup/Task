#pragma once

#include <QWidget>

class QPlainTextEdit;
class QLineEdit;
class QLabel;
class QSpinBox;
class QComboBox;

class DecoderWindow : public QWidget
{
    Q_OBJECT

public:
    explicit DecoderWindow(QWidget* parent = nullptr);

private slots:
    void generateCode();
    void copyCode();
    void clearAll();
    void updateExpirePreview();

private:
    QPlainTextEdit* m_serialEdit = nullptr;
    QSpinBox* m_amountSpin = nullptr;
    QComboBox* m_unitCombo = nullptr;
    QLabel* m_expireLabel = nullptr;
    QLineEdit* m_codeEdit = nullptr;
    QLabel* m_statusLabel = nullptr;
};
