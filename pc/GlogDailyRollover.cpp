#include "GlogDailyRollover.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QTime>

#include <glog/logging.h>

GlogDailyRollover& GlogDailyRollover::instance()
{
    static GlogDailyRollover self;
    return self;
}

GlogDailyRollover::GlogDailyRollover(QObject* parent)
    : QObject(parent)
{
    connect(&timer_, &QTimer::timeout, this, &GlogDailyRollover::onMidnight);
    timer_.setSingleShot(true);
}

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

void GlogDailyRollover::purgeExpiredDateFolders()
{
    if (keepDays_ < 1)
        return;

    // 删除目录日期严格早于 (今天 - keepDays) 的文件夹，例如 keepDays=14 时删掉 14 天以前那天的目录。
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

void GlogDailyRollover::start(int keepDays)
{
    keepDays_ = qMax(1, keepDays);

    applyDestinationsFor(QDate::currentDate());
    purgeExpiredDateFolders();
    scheduleNextMidnight();
}

qint64 GlogDailyRollover::msecsUntilNextMidnight() const
{
    const QDateTime now = QDateTime::currentDateTime();
    const QDateTime nextMidnight(now.date().addDays(1), QTime(0, 0, 0));
    return now.msecsTo(nextMidnight);
}

void GlogDailyRollover::scheduleNextMidnight()
{
    timer_.start(msecsUntilNextMidnight());
}

void GlogDailyRollover::onMidnight()
{
    google::FlushLogFiles(google::GLOG_INFO);

    const QDate today = QDate::currentDate();
    if (today != activeDate_)
        applyDestinationsFor(today);

    purgeExpiredDateFolders();
    scheduleNextMidnight();
}

/*
 * ---------- main 使用示例 ----------
 *
 * #include <QCoreApplication>
 * #include <QDir>
 * #include <glog/logging.h>
 * #include "GlogDailyRollover.h"
 *
 * int main(int argc, char* argv[])
 * {
 *     QCoreApplication app(argc, argv);
 *
 *     QDir().mkpath(QStringLiteral("./logs"));
 *
 *     google::InitGoogleLogging(argv[0]);
 *     FLAGS_log_dir = "./logs";
 *     FLAGS_alsologtostderr = false;
 *     google::SetLogFilenameExtension(".txt");
 *
 *     // 按日分目录、午夜切换、logs 下删除 14 天前的 yyyy-MM-dd 目录
 *     GlogDailyRollover::instance().start(14);
 *
 *     LOG(INFO) << "application started";
 *
 *     return app.exec();
 * }
 *
 * 进程退出前可选：
 *     google::ShutdownGoogleLogging();
 */