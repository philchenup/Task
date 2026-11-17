/**
 * @file GlogDailyRollover.cpp
 * @brief GlogDailyRollover 实现：目录切换、定时过日与过期清理。
 */
#include "GlogDailyRollover.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QTime>

#include <glog/logging.h>

/** @brief 返回函数内静态单例引用。 */
GlogDailyRollover& GlogDailyRollover::instance()
{
    static GlogDailyRollover self;
    return self;
}

/** @brief 连接单次定时器，超时进入 onMidnight。 */
GlogDailyRollover::GlogDailyRollover(QObject* parent)
    : QObject(parent)
{
    connect(&timer_, &QTimer::timeout, this, &GlogDailyRollover::onMidnight);
    timer_.setSingleShot(true);
}

/**
 * @brief 创建当日目录并将 INFO/WARNING/ERROR/FATAL 分别指向 hsiweld.* 文件前缀。
 * @param day 日志所属自然日，同时写入 activeDate_。
 */
void GlogDailyRollover::applyDestinationsFor(const QDate& day)
{
    const QString dayDir =
        QStringLiteral("%1/%2").arg(logRoot_, day.toString(QStringLiteral("yyyy-MM-dd")));
    QDir().mkpath(dayDir);
    const std::string prefix = (dayDir + QLatin1Char('/')).toStdString();
    google::SetLogDestination(google::GLOG_INFO, (prefix + "hsiweld.info.").c_str());
    google::SetLogDestination(google::GLOG_WARNING, (prefix + "hsiweld.warn.").c_str());
    google::SetLogDestination(google::GLOG_ERROR, (prefix + "hsiweld.error.").c_str());
    google::SetLogDestination(google::GLOG_FATAL, (prefix + "hsiweld.fatal.").c_str());
    activeDate_ = day;
}

/**
 * @brief 扫描 logRoot_ 下符合 yyyy-MM-dd 命名的子目录并递归删除早于 cutoff 的目录。
 *
 * cutoff = 今天 - keepDays_（例如 keepDays=14 时删除 14 天以前那天的目录）。
 * 无效目录名或非日期格式会跳过。
 */
void GlogDailyRollover::purgeExpiredDateFolders()
{
    if (keepDays_ < 1)
        return;
    const QDate cutoff = QDate::currentDate().addDays(-keepDays_);
    QDir root(logRoot_);
    if (!root.exists())
        return;
    const QFileInfoList entries =
        root.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);
    for (const QFileInfo& fi : entries) {
        const QDate dirDay =
            QDate::fromString(fi.fileName(), QStringLiteral("yyyy-MM-dd"));
        if (!dirDay.isValid())
            continue;
        if (dirDay < cutoff) {
            const QString path = fi.absoluteFilePath();
            if (QDir(path).removeRecursively())
                LOG(INFO) << "Removed expired log directory: " << path.toStdString();
        }
    }
}

/** @brief 更新保留天数、应用当日目的地、清理并启动午夜定时器。 */
void GlogDailyRollover::start(int keepDays)
{
    keepDays_ = qMax(1, keepDays);
    applyDestinationsFor(QDate::currentDate());
    purgeExpiredDateFolders();
    scheduleNextMidnight();
}

/** @brief 计算到次日 0 点的毫秒间隔（用于 QTimer::start）。 */
qint64 GlogDailyRollover::msecsUntilNextMidnight() const
{
    const QDateTime now = QDateTime::currentDateTime();
    const QDateTime nextMidnight(now.date().addDays(1), QTime(0, 0, 0));
    return now.msecsTo(nextMidnight);
}

/** @brief 按 msecsUntilNextMidnight 重新 arm 单次定时器。 */
void GlogDailyRollover::scheduleNextMidnight()
{
    timer_.start(msecsUntilNextMidnight());
}

/**
 * @brief 跨越自然日时的处理：刷 INFO 日志、若日期变化则切换目录、清理、预约下一 0 点。
 */
void GlogDailyRollover::onMidnight()
{
    google::FlushLogFiles(google::GLOG_INFO);
    const QDate today = QDate::currentDate();
    if (today != activeDate_)
        applyDestinationsFor(today);
    purgeExpiredDateFolders();
    scheduleNextMidnight();
}