/**
 * @file GlogDailyRollover.h
 * @brief 按自然日将 glog 写入子目录，每日 0 点切换并清理过期日志文件夹。
 */
#ifndef GLOG_DAILY_ROLLOVER_H
#define GLOG_DAILY_ROLLOVER_H

#include <QObject>
#include <QDate>
#include <QTimer>
#include <QString>

/**
 * @brief 单例：管理 glog 日志目录按天滚动与保留策略。
 *
 * 行为概要：
 * - 日志根目录默认为 `./logs`，每日子目录名为 `yyyy-MM-dd`。
 * - 各级别文件前缀为 `hsiweld.{info,warn,error,fatal}.`。
 * - 在 `google::InitGoogleLogging` 之后调用 start() 生效。
 * - 保留 keepDays 个自然日（含当天策略见 start 说明），更早的日期目录会被删除。
 */
class GlogDailyRollover final : public QObject
{
public:
    /** @brief 进程内唯一实例（函数内静态局部变量）。 */
    static GlogDailyRollover& instance();

    /**
     * @brief 启动按日滚动：设置当日目录、清理过期文件夹、预约下次 0 点切换。
     * @param keepDays 至少保留最近多少个自然日的日志目录（内部下限为 1，默认 14）。
     * @note 须在 InitGoogleLogging 之后调用。
     */
    void start(int keepDays = 14);

    /**
     * @brief 将 glog 输出目录切换到指定自然日。
     * @param day 目标日期；会创建 `logRoot()/yyyy-MM-dd` 并调用 SetLogDestination。
     */
    void applyDestinationsFor(const QDate& day);

    /** @brief 日志根路径（默认 `./logs`）。 */
    QString logRoot() const { return logRoot_; }

    /** @brief 当前配置的保留天数。 */
    int keepDays() const { return keepDays_; }

private:
    explicit GlogDailyRollover(QObject* parent = nullptr);

    /** @brief 以单次触发方式启动定时器，超时时刻为下一本地 0 点。 */
    void scheduleNextMidnight();

    /** @brief 0 点回调：刷盘、按日切换目录、清理、再次预约。 */
    void onMidnight();

    /** @brief 删除 logRoot 下日期早于 cutoff 的 `yyyy-MM-dd` 子目录。 */
    void purgeExpiredDateFolders();

    /** @brief 当前时间到下一自然日 0:00:00 的毫秒数。 */
    qint64 msecsUntilNextMidnight() const;

    QTimer timer_;
    int keepDays_{ 14 };
    QString logRoot_{ QStringLiteral("./logs") };
    QDate activeDate_;
};

#endif // GLOG_DAILY_ROLLOVER_H