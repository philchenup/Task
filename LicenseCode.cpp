#include "LicenseCode.h"

#include <QCryptographicHash>

namespace {

QString wrapKey(const QByteArray& utf8)
{
    const QString md5 = QString::fromLatin1(
        QCryptographicHash::hash(utf8, QCryptographicHash::Md5).toHex());
    return QStringLiteral("5") + md5.mid(0, 10) + QStringLiteral("1") + md5.mid(10) + QStringLiteral("2");
}

} // namespace

QString makeAuthorizationCode(const QString& serial)
{
    return wrapKey(serial.toUtf8());
}

QString licenseTimeUnitText(LicenseTimeUnit unit)
{
    switch (unit)
    {
    case LicenseTimeUnit::Day:
        return QStringLiteral("天");
    case LicenseTimeUnit::Month:
        return QStringLiteral("月");
    case LicenseTimeUnit::Year:
        return QStringLiteral("年");
    }
    return QString();
}

QDate licenseExpireDate(const QDate& startDate, int amount, LicenseTimeUnit unit)
{
    if (!startDate.isValid() || amount < 1)
        return QDate();

    switch (unit)
    {
    case LicenseTimeUnit::Day:
        return startDate.addDays(amount);
    case LicenseTimeUnit::Month:
        return startDate.addMonths(amount);
    case LicenseTimeUnit::Year:
        return startDate.addYears(amount);
    }
    return QDate();
}

QString licenseExpireText(const QDate& expireDate)
{
    if (!expireDate.isValid())
        return QString();
    return expireDate.toString(QStringLiteral("yyyy-MM-dd")) + QStringLiteral(" 23:59:59");
}

QString licensePeriodText(const QDate& startDate, int amount, LicenseTimeUnit unit)
{
    const QDate expireDate = licenseExpireDate(startDate, amount, unit);
    if (!startDate.isValid() || !expireDate.isValid())
        return QStringLiteral("期限无效");

    return QStringLiteral("%1 起，有效期至 %2（%3 %4）")
        .arg(startDate.toString(QStringLiteral("yyyy-MM-dd")), licenseExpireText(expireDate))
        .arg(amount)
        .arg(licenseTimeUnitText(unit));
}

QString makeAuthorizationCode(const QString& serial, const QDate& expireDate)
{
    if (!expireDate.isValid())
        return QString();

    const QString stamp = expireDate.toString(QStringLiteral("yyyyMMdd"));
    const QString material = serial + QLatin1Char('|') + stamp;
    return wrapKey(material.toUtf8()) + stamp;
}

bool authorizationCodeMatches(const QString& serial, const QString& code, QDate* expireDate)
{
    const QString trimmed = code.trimmed();
    if (trimmed.size() != 43)
        return false;

    const QDate date = QDate::fromString(trimmed.right(8), QStringLiteral("yyyyMMdd"));
    if (!date.isValid() || makeAuthorizationCode(serial, date) != trimmed)
        return false;

    if (expireDate)
        *expireDate = date;
    return true;
}

LicenseStatus checkAuthorizationCode(const QString& serial, const QString& code, const QDate& today, QDate* expireDate)
{
    QDate parsed;
    if (!authorizationCodeMatches(serial, code, &parsed))
        return LicenseStatus::Invalid;

    if (expireDate)
        *expireDate = parsed;
    if (!today.isValid() || today > parsed)
        return LicenseStatus::Expired;
    return LicenseStatus::Valid;
}
