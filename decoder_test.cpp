#include "DecoderWindow.h"
#include "LicenseCode.h"

#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QCryptographicHash>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QSpinBox>
#include <QTimer>

#include <cstdlib>
#include <iostream>

namespace {

int failures = 0;

void expect(bool condition, const char* message)
{
    if (condition)
        return;
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
}

QString wrapKey(const QByteArray& utf8)
{
    const QString md5 = QString::fromLatin1(
        QCryptographicHash::hash(utf8, QCryptographicHash::Md5).toHex());
    return QStringLiteral("5") + md5.mid(0, 10) + QStringLiteral("1") + md5.mid(10) + QStringLiteral("2");
}

void testLicenseCode()
{
    expect(makeAuthorizationCode(QStringLiteral("abc")) == QStringLiteral("5900150983c1d24fb0d6963f7d28e17f722"),
           "permanent code for abc");
    expect(makeAuthorizationCode(QStringLiteral("abc")) == wrapKey(QByteArray("abc")),
           "permanent code uses the original wrap");

    const QDate expire(2027, 10, 13);
    const QString serial = QStringLiteral("MACHINE-001");
    const QString code = makeAuthorizationCode(serial, expire);
    const QString expected = wrapKey(QByteArray("MACHINE-001|20271013")) + QStringLiteral("20271013");
    expect(code == expected, "timed code binds serial and yyyyMMdd");
    expect(code.size() == 43, "timed code length");
    expect(code.endsWith(QStringLiteral("20271013")), "timed code shows the expire date");
    expect(makeAuthorizationCode(serial, QDate()).isEmpty(), "invalid expire date yields empty code");

    QDate parsed;
    expect(authorizationCodeMatches(serial, code, &parsed), "matching code verifies");
    expect(parsed == expire, "matching code returns the expire date");
    expect(checkAuthorizationCode(serial, code, expire, &parsed) == LicenseStatus::Valid, "valid on the expire date");
    expect(parsed == expire, "check returns the expire date");
    expect(checkAuthorizationCode(serial, QStringLiteral(" ") + code + QStringLiteral(" "), expire.addDays(-1)) == LicenseStatus::Valid,
           "trimmed code is still valid before expiry");
    expect(checkAuthorizationCode(serial, code, expire.addDays(1)) == LicenseStatus::Expired, "the next day is expired");
    expect(checkAuthorizationCode(QStringLiteral("other-machine"), code, expire) == LicenseStatus::Invalid, "wrong machine is invalid");
    expect(checkAuthorizationCode(serial, makeAuthorizationCode(serial), expire) == LicenseStatus::Invalid, "code without a date is invalid");

    QString tampered = code;
    tampered[42] = tampered[42] == QLatin1Char('3') ? QLatin1Char('4') : QLatin1Char('3');
    expect(!authorizationCodeMatches(serial, tampered, &parsed), "edited expire date does not verify");
    expect(makeAuthorizationCode(serial, QDate(2026, 10, 14)) != makeAuthorizationCode(serial, QDate(2027, 10, 13)),
           "different durations produce different codes");

    expect(licenseExpireDate(QDate(2026, 10, 13), 1, LicenseTimeUnit::Day) == QDate(2026, 10, 14), "1 day");
    expect(licenseExpireDate(QDate(2026, 10, 13), 30, LicenseTimeUnit::Day) == QDate(2026, 11, 12), "30 days");
    expect(licenseExpireDate(QDate(2026, 1, 31), 1, LicenseTimeUnit::Month) == QDate(2026, 2, 28), "month end non-leap");
    expect(licenseExpireDate(QDate(2024, 1, 31), 1, LicenseTimeUnit::Month) == QDate(2024, 2, 29), "month end leap");
    expect(licenseExpireDate(QDate(2026, 10, 13), 3, LicenseTimeUnit::Month) == QDate(2027, 1, 13), "3 months");
    expect(licenseExpireDate(QDate(2026, 10, 13), 1, LicenseTimeUnit::Year) == QDate(2027, 10, 13), "1 year");
    expect(licenseExpireDate(QDate(2024, 2, 29), 1, LicenseTimeUnit::Year) == QDate(2025, 2, 28), "leap day plus one year");
    expect(!licenseExpireDate(QDate(2026, 10, 13), 0, LicenseTimeUnit::Day).isValid(), "zero amount is invalid");
    expect(licenseExpireText(QDate(2027, 10, 13)) == QStringLiteral("2027-10-13 23:59:59"), "expire text");
    expect(licensePeriodText(QDate(2026, 10, 13), 1, LicenseTimeUnit::Year)
               == QStringLiteral("2026-10-13 起，有效期至 2027-10-13 23:59:59（1 年）"),
           "period text");
    expect(licenseTimeUnitText(LicenseTimeUnit::Day) == QStringLiteral("天"), "day unit text");
    expect(licenseTimeUnitText(LicenseTimeUnit::Month) == QStringLiteral("月"), "month unit text");
}

void testWindow()
{
    DecoderWindow window;
    auto* serial = window.findChild<QPlainTextEdit*>(QStringLiteral("serial"));
    auto* amount = window.findChild<QSpinBox*>(QStringLiteral("amount"));
    auto* unit = window.findChild<QComboBox*>(QStringLiteral("unit"));
    auto* expire = window.findChild<QLabel*>(QStringLiteral("expire"));
    auto* code = window.findChild<QLineEdit*>(QStringLiteral("code"));
    auto* status = window.findChild<QLabel*>(QStringLiteral("status"));
    expect(serial && amount && unit && expire && code && status, "window controls exist");
    if (!serial || !amount || !unit || !expire || !code || !status)
        return;

    expect(amount->value() == 1, "default amount");
    expect(unit->currentText() == QStringLiteral("年"), "default unit is year");
    expect(expire->text() == licensePeriodText(QDate::currentDate(), 1, LicenseTimeUnit::Year), "default expire preview");

    unit->setCurrentText(QStringLiteral("天"));
    amount->setValue(10);
    expect(expire->text() == licensePeriodText(QDate::currentDate(), 10, LicenseTimeUnit::Day), "day preview");

    unit->setCurrentText(QStringLiteral("月"));
    amount->setValue(3);
    expect(expire->text() == licensePeriodText(QDate::currentDate(), 3, LicenseTimeUnit::Month), "month preview");

    unit->setCurrentText(QStringLiteral("年"));
    amount->setValue(2);
    serial->setPlainText(QStringLiteral("  SN-100 \n"));
    QMetaObject::invokeMethod(&window, "generateCode");
    const QDate twoYears = licenseExpireDate(QDate::currentDate(), 2, LicenseTimeUnit::Year);
    QDate parsed;
    expect(authorizationCodeMatches(QStringLiteral("SN-100"), code->text(), &parsed), "generated code matches trimmed serial");
    expect(parsed == twoYears, "generated code carries the two-year expire date");
    expect(status->text().contains(licensePeriodText(QDate::currentDate(), 2, LicenseTimeUnit::Year)), "status shows the period");
    expect(expire->text() == licensePeriodText(QDate::currentDate(), 2, LicenseTimeUnit::Year), "preview shows two years");
    expect(checkAuthorizationCode(QStringLiteral("SN-100"), code->text(), QDate::currentDate()) == LicenseStatus::Valid,
           "generated code is accepted today");

    QMetaObject::invokeMethod(&window, "copyCode");
    expect(QApplication::clipboard()->text() == code->text(), "copy puts the code on the clipboard");

    QTimer::singleShot(0, [] {
        if (QWidget* box = QApplication::activeModalWidget())
            box->close();
    });
    serial->clear();
    QMetaObject::invokeMethod(&window, "generateCode");
    expect(code->text().isEmpty(), "empty serial clears the code");
    expect(status->text() == QStringLiteral("请输入机器序列号"), "empty serial status");

    serial->setPlainText(QStringLiteral("SN"));
    amount->setValue(5);
    unit->setCurrentText(QStringLiteral("天"));
    QMetaObject::invokeMethod(&window, "clearAll");
    expect(serial->toPlainText().isEmpty(), "clear removes the serial");
    expect(code->text().isEmpty(), "clear removes the code");
    expect(amount->value() == 1 && unit->currentText() == QStringLiteral("年"), "clear restores 1 year");
    expect(status->text() == QStringLiteral("等待输入"), "clear restores the waiting status");
    expect(expire->text() == licensePeriodText(QDate::currentDate(), 1, LicenseTimeUnit::Year), "clear refreshes the preview");
}

} // namespace

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    testLicenseCode();
    testWindow();
    if (failures != 0)
    {
        std::cerr << failures << " failure(s)\n";
        return EXIT_FAILURE;
    }
    std::cout << "ok\n";
    return EXIT_SUCCESS;
}
