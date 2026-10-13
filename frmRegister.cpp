#include "frmRegister.h"
#include "LicenseCode.h" // 与解码器共用同一套期限规则，工程里需要同时编译 LicenseCode.cpp
#include <QDesktopWidget>
#include <QStyle>
#include <QRegularExpression>
#include <QProcess.h>

frmRegister::frmRegister(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	QString key1 = getCpuSerialNumber();
	QString key2 = getDiskSerialNumber();
	str_key = key1.mid(0, 4) + "hs" + key2.mid(3, key2.length() - 2) + "6" + key1.mid(key1.length() - 4, key1.length() - 1) + "6";
	ui.txtSerialNumber->setText(str_key);

	QSettings settings("HKEY_CURRENT_USER\\Software\\MyApp", QSettings::NativeFormat);
	value = settings.value("sn_code", "error").toString();
}

bool frmRegister::isRegistered()
{
	// 启动处在窗口关闭后还会再查一次，所以这里同时核对机器码和到期日。
	return checkAuthorizationCode(str_key, value, QDate::currentDate()) == LicenseStatus::Valid;
}

void frmRegister::closeEvent(QCloseEvent* event)
{
	emit closed();
	QWidget::closeEvent(event);
}

void frmRegister::form_statues()
{		
	form_state = 1;
	m_timer->stop();
	m_timer->deleteLater();
	// 对由CPU和硬盘生成的序列号进行加密后输入二进制编码
	if (isRegistered())
	{		
		this->close();			
	}	
	else
	{		
		this->close();			
		frmRegister *fRegister = new frmRegister();		
		//窗体移至屏幕中间
		fRegister->move(QApplication::desktop()->screen()->rect().center() - fRegister->rect().center());
		fRegister->show();			
	}
}

void frmRegister::on_btnCmdCheckID_clicked()
{	
	// 判断输入的解码序列是否正确，并按授权码末尾的到期日限制使用。
	str_input = ui.txtAuthorizationCode->text().trimmed();
	QDate expireDate;
	const LicenseStatus status = checkAuthorizationCode(str_key, str_input, QDate::currentDate(), &expireDate);
	if (status == LicenseStatus::Valid)
	{
		QSettings settings("HKEY_CURRENT_USER\\Software\\MyApp", QSettings::NativeFormat);
		//写入注册表。保存完整授权码，启动时才能再次核对到期日。
		settings.setValue("sn_code", str_input);
		value = str_input;
		QMessageBox msgBox(QMessageBox::Icon::NoIcon, "Tips",
			QString("Registration is complete!\nValid until %1.").arg(licenseExpireText(expireDate)));
		msgBox.setWindowIcon(QIcon(":/res/ico/info.png"));
		msgBox.exec();			
		this->close();
	}
	else if (status == LicenseStatus::Expired)
	{
		QMessageBox msgBox(QMessageBox::Icon::NoIcon, "Error",
			QString("Registration code has expired!\nIt was valid until %1.").arg(licenseExpireText(expireDate)));
		msgBox.setWindowIcon(QIcon(":/res/ico/error.png"));
		msgBox.exec();
	}
	else
	{
		QMessageBox msgBox(QMessageBox::Icon::NoIcon, "Error", "Wrong registration code!");
		msgBox.setWindowIcon(QIcon(":/res/ico/error.png"));
		msgBox.exec();		
	}	
}

void frmRegister::on_btnCmdCheckID_pressed()
{
	ui.btnCmdCheckID->setStyleSheet("QPushButton{background-color: rgb(146, 217, 108);color: rgb(0, 0, 0);border: 2px groove gray; border-radius: 8px;}");
}

void frmRegister::on_btnCmdCheckID_released()
{
	ui.btnCmdCheckID->setStyleSheet("QPushButton{background-color: rgb(85, 170, 127);color: rgb(0, 0, 0);border: 2px groove gray; border-radius: 8px;}");
}

QString frmRegister::GetKey(QString str)
{
	QString md5;
	QByteArray data;
	data = QCryptographicHash::hash(str.toUtf8(), QCryptographicHash::Md5);
	md5.append(data.toHex());
	QString str1 = md5.mid(0, 10);
	QString str2 = md5.mid(10);
	QString str_all = "5" + str1 + "1" + str2 + "2";
	return str_all;
}

QString frmRegister::getWMIC(const QString& cmd)
{
	//查询cpu序列号：wmic cpu get processorid
	//查询主板序列号：wmic baseboard get serialnumber
	//查询BIOS序列号：wmic bios get serialnumber
	//查看硬盘序列号：wmic diskdrive get serialnumber
	QProcess p;
	p.start(cmd);
	p.waitForFinished();
	QString result = QString::fromLocal8Bit(p.readAllStandardOutput());
	QStringList list = cmd.split(" ");
	result = result.remove(list.last(), Qt::CaseInsensitive);
	result = result.replace("\r", "");
	result = result.replace("\n", "");
	result = result.simplified();
	return result;
}


QString frmRegister::decodeProcessOutput(const QByteArray& raw)
{
	if (raw.size() >= 2 &&
		static_cast<unsigned char>(raw.at(0)) == 0xFF &&
		static_cast<unsigned char>(raw.at(1)) == 0xFE)
	{
		QString out;
		const int n = (raw.size() - 2) / 2;
		out.resize(n);
		const auto* p = reinterpret_cast<const unsigned char*>(raw.constData() + 2);
		for (int i = 0; i < n; ++i)
			out[i] = QChar(p[i * 2] | (p[i * 2 + 1] << 8));
		return out;
	}
	if (raw.contains('\0'))
	{
		QString out;
		const int n = raw.size() / 2;
		out.resize(n);
		const auto* p = reinterpret_cast<const unsigned char*>(raw.constData());
		for (int i = 0; i < n; ++i)
			out[i] = QChar(p[i * 2] | (p[i * 2 + 1] << 8));
		return out;
	}
	return QString::fromLocal8Bit(raw);
}

QStringList frmRegister::runLines(const QString& program, const QStringList& args)
{
	QProcess p;
	p.start(program, args);
	if (!p.waitForStarted(5000) || !p.waitForFinished(20000))
	{
		p.kill();
		return {};
	}
	QString text = decodeProcessOutput(p.readAllStandardOutput());
	text.replace(QChar('\0'), QString());

	QStringList lines;
	const QStringList rawLines = text.split(QRegularExpression("[\\r\\n]"), Qt::SkipEmptyParts);
	for (QString line : rawLines)
	{
		line = line.trimmed();
		if (!line.isEmpty())
			lines << line;
	}
	return lines;
}

// 跳过表头，丢掉空值和 OEM 占位
QString frmRegister::firstValue(const QStringList& lines, const QString& header)
{
	QStringList values;
	for (const QString& line : lines)
	{
		if (line.compare(header, Qt::CaseInsensitive) == 0)
			continue;
		if (line.contains("O.E.M", Qt::CaseInsensitive) ||
			line.compare("Default string", Qt::CaseInsensitive) == 0 ||
			line.compare("To be filled by O.E.M.", Qt::CaseInsensitive) == 0 ||
			line.compare("None", Qt::CaseInsensitive) == 0)
			continue;
		values << line;
	}
	values.sort(Qt::CaseInsensitive);
	return values.isEmpty() ? QString() : values.first();
}

QString frmRegister::getCpuSerialNumber()
{
	QStringList lines = runLines("powershell.exe",
		{ "-NoProfile", "-NonInteractive", "-Command",
		 "(Get-CimInstance Win32_Processor).ProcessorId" });
	if (lines.isEmpty())
	{
		lines = runLines("wmic", { "cpu", "get", "processorid" });
		return firstValue(lines, "ProcessorId");
	}
	return lines.first().trimmed();
}

QString frmRegister::getDiskSerialNumber()
{
	QStringList lines = runLines("powershell.exe",
		{ "-NoProfile", "-NonInteractive", "-Command",
		 "(Get-CimInstance Win32_DiskDrive).SerialNumber" });
	if (lines.isEmpty())
		lines = runLines("wmic", { "diskdrive", "get", "serialnumber" });
	return firstValue(lines, "SerialNumber");
}
