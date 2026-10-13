#pragma once

#include <QDate>
#include <QString>

// 与 frmRegister::GetKey / SoftKey::GetKey 相同：MD5 十六进制小写，
// 再拼成 5 + 前 10 位 + 1 + 其余 + 2。
QString makeAuthorizationCode(const QString& serial);

// 授权期限按自然日、自然月或自然年顺延。
// 到期日当天 23:59:59 前有效；目标月没有对应日时，取该月最后一天
// （例如 1 月 31 日加 1 个月落到 2 月 28 日或 29 日，2 月 29 日加 1 年落到 2 月 28 日）。
enum class LicenseTimeUnit
{
    Day,
    Month,
    Year
};

QString licenseTimeUnitText(LicenseTimeUnit unit);

// amount < 1 或起始日无效时返回无效日期。
QDate licenseExpireDate(const QDate& startDate, int amount, LicenseTimeUnit unit);

// 到期时刻固定显示为当天 23:59:59。
QString licenseExpireText(const QDate& expireDate);

// 例如：2026-10-13 起，有效期至 2027-10-13 23:59:59（1 年）
QString licensePeriodText(const QDate& startDate, int amount, LicenseTimeUnit unit);

// 带期限的授权码：散列原文为「机器序列号|yyyyMMdd」，
// 主体仍是 5 + 前 10 位 + 1 + 其余 + 2，末尾附上 8 位到期日，时间可以直接读出。
// 到期日无效时返回空串。
QString makeAuthorizationCode(const QString& serial, const QDate& expireDate);

// 校验授权码是否由该序列号和其中携带的到期日生成。匹配时写出到期日。
bool authorizationCodeMatches(const QString& serial, const QString& code, QDate* expireDate = nullptr);

// frmRegister 用同一规则判断：码必须对应该机器，且 today 不超过到期日（含当天）。
enum class LicenseStatus
{
    Valid,
    Invalid,
    Expired
};

LicenseStatus checkAuthorizationCode(const QString& serial, const QString& code, const QDate& today, QDate* expireDate = nullptr);
