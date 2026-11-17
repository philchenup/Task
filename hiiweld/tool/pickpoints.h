/**
 * @file pickpoints.h
 * @brief 点云选点工具：两点或多边形选区，生成子点云。
 */
#ifndef CT_TOOL_PICKPOINTS_H
#define CT_TOOL_PICKPOINTS_H

#include "base/customdialog.h"
#include "base/common.h"

namespace Ui
{
    class PickPoints;
}

/**
 * @brief 在 CloudView 上交互拾取点或多边形区域，导出选中点云。
 */
class PickPoints : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit PickPoints(QWidget* parent = nullptr);
    ~PickPoints() override;

    virtual void init();
    /** @brief 进入/退出选点模式。 */
    void start();
    /** @brief 将拾取结果添加为子点云节点。 */
    void add();
    virtual void reset();
    virtual void deinit() { m_cloudview->clearInfo(); }

private:
    /** @brief 根据选点类型更新界面提示。 */
    void updateInfo(int index);

public slots:
    void mouseLeftPressed(const ct::PointXY& pt);
    void mouseLeftReleased(const ct::PointXY& pt);
    void mouseRightReleased(const ct::PointXY& pt);
    void mouseMoved(const ct::PointXY& pt);

signals:
    /** @brief 拾取完成时发出点云副本。 */
    void sendPointsPicked(const ct::Cloud::Ptr& pick_points);

private:
    Ui::PickPoints* ui;
    bool is_picking;
    bool pick_start;
    ct::Cloud::Ptr m_selected_cloud;
    ct::Cloud::Ptr m_pick_cloud;
    ct::PointXY m_pick_point;
};

#endif // CT_TOOL_PICKPOINTS_H