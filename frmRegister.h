#pragma once

#include <QtWidgets/QWidget>
#include "ui_frmRegister.h"
#include <QSettings>
#include <QMessageBox>
#include <QTimer>
#include <QCryptographicHash>
#include <QCloseEvent>
#include <QByteArray>
#include <QStringList>

static int form_state;

class frmRegister : public QWidget
{
	Q_OBJECT

public:
	frmRegister(QWidget *parent = Q_NULLPTR);
	// 注册表 sn_code 保存带到期日的授权码。码必须由本机机器码生成，且今天不超过到期日。
	bool isRegistered();

	QString decodeProcessOutput(const QByteArray& raw);
	QStringList runLines(const QString& program, const QStringList& args);
	QString firstValue(const QStringList& lines, const QString& header);

signals:
	void closed();

private slots:
	void form_statues();
	void on_btnCmdCheckID_clicked();
	void on_btnCmdCheckID_pressed();
	void on_btnCmdCheckID_released();
	QString GetKey(QString str);
	QString getWMIC(const QString& cmd);
	QString getCpuSerialNumber();
	/*QString getMotherboardSerialNumber();
	QString getBIOSSerialNumber();*/
	QString getDiskSerialNumber();

private:

	QTimer *m_timer;	
	QString str_input;
	QString value;	
	QString str_key;
	Ui::frmRegisterClass ui;

protected:
	void closeEvent(QCloseEvent* event) override;

};
