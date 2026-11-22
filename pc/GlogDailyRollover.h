#ifndef GLOG_DAILY_ROLLOVER_H
#define GLOG_DAILY_ROLLOVER_H

#include <QObject>
#include <QDate>
#include <QTimer>
#include <QString>

// 按自然日分目录写 glog；每天 0 点切换目录；自动删除 logs 下早于保留期的 yyyy-MM-dd 子目录。
class GlogDailyRollover final : public QObject
{
public:
    static GlogDailyRollover& instance();

    // 在 google::InitGoogleLogging 之后调用；keepDays 默认保留 14 个自然日（含今天的滚动策略见实现注释）
    void start(int keepDays = 14);

    // 将当日日志 destination 设为 logs/<yyyy-MM-dd>/hsiweld.{info,warn,error,fatal}.
    void applyDestinationsFor(const QDate& day);

    QString logRoot() const { return logRoot_; }
    int keepDays() const { return keepDays_; }

private:
    explicit GlogDailyRollover(QObject* parent = nullptr);

    void scheduleNextMidnight();
    void onMidnight();
    void purgeExpiredDateFolders();
    qint64 msecsUntilNextMidnight() const;

    QTimer timer_;
    int keepDays_{14};
    QString logRoot_{QStringLiteral("./logs")};
    QDate activeDate_;
};

#endif // GLOG_DAILY_ROLLOVER_H