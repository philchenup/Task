#include "StepRegistry.h"

namespace {

PortDesc makePort(const QString &key, const QString &typeId,
                  const QString &label, const QString &name)
{
    PortDesc desc;
    desc.key = key;
    desc.typeId = typeId;
    desc.typeLabel = label;
    desc.name = name;
    return desc;
}

StepDesc makeStep(const QString &typeId, const QString &title, const QString &category,
                  const QString &summary, const QString &icon,
                  const QVector<PortDesc> &inputs, const QVector<PortDesc> &outputs)
{
    StepDesc desc;
    desc.typeId = typeId;
    desc.title = title;
    desc.category = category;
    desc.summary = summary;
    desc.icon = icon;
    desc.inputs = inputs;
    desc.outputs = outputs;
    return desc;
}

} // namespace

StepRegistry::StepRegistry()
{
    const QString cloud = QStringLiteral("CloudXYZ");
    const QString cloudN = QStringLiteral("CloudXYZNormal");
    const QString pose = QStringLiteral("Pose");
    const QString image = QStringLiteral("Image");
    const QString path = QStringLiteral("Path");
    const QString number = QStringLiteral("Number");
    const QString boolean = QStringLiteral("Bool");

    const QString cloudLabel = QStringLiteral("<Cloud(XYZ)>");
    const QString cloudNLabel = QStringLiteral("<Cloud(XYZ-Normal)>");
    const QString poseLabel = QStringLiteral("<Pose>");
    const QString imageLabel = QStringLiteral("<Image>");
    const QString pathLabel = QStringLiteral("<Path>");
    const QString numberLabel = QStringLiteral("<Number>");
    const QString boolLabel = QStringLiteral("<Bool>");

    addStep(makeStep("read_cloud_v2", QStringLiteral("读取点云V2"), QStringLiteral("采集"),
                     QStringLiteral("从文件读取 XYZ 点云。"), QStringLiteral("cloud"),
                     {},
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))}));

    addStep(makeStep("capture_image", QStringLiteral("从相机采集图像"), QStringLiteral("采集"),
                     QStringLiteral("从已连接的相机采集一帧图像。"), QStringLiteral("camera"),
                     {},
                     {makePort("image", image, imageLabel, QStringLiteral("图像"))}));

    addStep(makeStep("read_image", QStringLiteral("读取图像"), QStringLiteral("采集"),
                     QStringLiteral("从文件读取图像。"), QStringLiteral("image"),
                     {},
                     {makePort("image", image, imageLabel, QStringLiteral("图像"))}));

    addStep(makeStep("calc_normals", QStringLiteral("计算点云的法向量并滤波"), QStringLiteral("预处理"),
                     QStringLiteral("估计点云法向量，并按阈值滤除离群点。"), QStringLiteral("normal"),
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))},
                     {makePort("cloud", cloudN, cloudNLabel, QStringLiteral("带有法向量的点云"))}));

    addStep(makeStep("downsample", QStringLiteral("点云下采样"), QStringLiteral("预处理"),
                     QStringLiteral("按体素尺寸降低点云密度。"), QStringLiteral("filter"),
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))},
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))}));

    addStep(makeStep("cloud_filter", QStringLiteral("点云滤波"), QStringLiteral("预处理"),
                     QStringLiteral("去除离群点和噪声。"), QStringLiteral("filter"),
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))},
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))}));

    addStep(makeStep("extract_edge", QStringLiteral("提取点云边缘"), QStringLiteral("预处理"),
                     QStringLiteral("从带法向量的点云中提取边缘点。"), QStringLiteral("normal"),
                     {makePort("cloud", cloudN, cloudNLabel, QStringLiteral("带有法向量的点云"))},
                     {makePort("edge", cloudN, cloudNLabel, QStringLiteral("点云边缘"))}));

    addStep(makeStep("match_3d", QStringLiteral("3D匹配"), QStringLiteral("识别"),
                     QStringLiteral("在点云中匹配三维模板，输出位姿。"), QStringLiteral("match"),
                     {makePort("cloud", cloudN, cloudNLabel, QStringLiteral("带有法向量的点云"))},
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))}));

    addStep(makeStep("match_template", QStringLiteral("模板匹配"), QStringLiteral("识别"),
                     QStringLiteral("在图像中匹配模板。"), QStringLiteral("match"),
                     {makePort("image", image, imageLabel, QStringLiteral("图像"))},
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))}));

    addStep(makeStep("pose_transform", QStringLiteral("位姿变换"), QStringLiteral("位姿处理"),
                     QStringLiteral("对位姿做平移和旋转。"), QStringLiteral("pose"),
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))},
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))}));

    addStep(makeStep("pose_filter", QStringLiteral("位姿过滤"), QStringLiteral("位姿处理"),
                     QStringLiteral("按角度和距离过滤位姿。"), QStringLiteral("pose"),
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))},
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))}));

    addStep(makeStep("plan_path", QStringLiteral("规划路径"), QStringLiteral("路径规划"),
                     QStringLiteral("由目标位姿生成机器人路径。"), QStringLiteral("path"),
                     {makePort("pose", pose, poseLabel, QStringLiteral("目标位姿"))},
                     {makePort("path", path, pathLabel, QStringLiteral("路径"))}));

    addStep(makeStep("evaluate", QStringLiteral("评估结果"), QStringLiteral("评估"),
                     QStringLiteral("判断位姿是否落在允许范围内。"), QStringLiteral("check"),
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))},
                     {makePort("ok", boolean, boolLabel, QStringLiteral("是否合格"))}));

    addStep(makeStep("step_group", QStringLiteral("步骤组合"), QStringLiteral("数据处理"),
                     QStringLiteral("把一组步骤收成一个节点。示例里把点云整理成边缘点云。"),
                     QStringLiteral("group"),
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))},
                     {makePort("edge", cloudN, cloudNLabel, QStringLiteral("点云边缘"))}));

    addStep(makeStep("number_op", QStringLiteral("数值运算"), QStringLiteral("数据处理"),
                     QStringLiteral("对输入数值做简单运算。"), QStringLiteral("number"),
                     {makePort("value", number, numberLabel, QStringLiteral("数值"))},
                     {makePort("value", number, numberLabel, QStringLiteral("数值"))}));

    addStep(makeStep("save_cloud", QStringLiteral("保存点云"), QStringLiteral("文件"),
                     QStringLiteral("把点云写到文件。"), QStringLiteral("file"),
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))},
                     {}));

    addStep(makeStep("save_pose", QStringLiteral("保存结果"), QStringLiteral("文件"),
                     QStringLiteral("把位姿写到文件。"), QStringLiteral("file"),
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))},
                     {}));

    addStep(makeStep("send_robot", QStringLiteral("发送到机器人"), QStringLiteral("传输"),
                     QStringLiteral("把路径发送到机器人控制器。"), QStringLiteral("send"),
                     {makePort("path", path, pathLabel, QStringLiteral("路径"))},
                     {}));

    addStep(makeStep("delay", QStringLiteral("延时"), QStringLiteral("系统"),
                     QStringLiteral("等待一段时间。此步骤不传递数据。"), QStringLiteral("timer"),
                     {},
                     {}));

    addStep(makeStep("counter", QStringLiteral("计数"), QStringLiteral("系统"),
                     QStringLiteral("累计运行次数。"), QStringLiteral("number"),
                     {},
                     {makePort("count", number, numberLabel, QStringLiteral("次数"))}));

    addStep(makeStep("script", QStringLiteral("脚本"), QStringLiteral("高级步骤"),
                     QStringLiteral("用脚本处理点云。"), QStringLiteral("script"),
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))},
                     {makePort("cloud", cloud, cloudLabel, QStringLiteral("点云"))}));

    addStep(makeStep("pose_sort", QStringLiteral("位姿排序"), QStringLiteral("位姿"),
                     QStringLiteral("按距离对位姿排序。"), QStringLiteral("sort"),
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))},
                     {makePort("pose", pose, poseLabel, QStringLiteral("位姿"))}));
}

void StepRegistry::addStep(const StepDesc &desc)
{
    if (!m_categories.contains(desc.category))
        m_categories.append(desc.category);
    m_index.insert(desc.typeId, m_steps.size());
    m_steps.append(desc);
}

const StepDesc *StepRegistry::find(const QString &typeId) const
{
    const auto it = m_index.constFind(typeId);
    if (it == m_index.cend())
        return nullptr;
    return &m_steps.at(it.value());
}

QStringList StepRegistry::categories() const
{
    return m_categories;
}

QVector<StepDesc> StepRegistry::stepsInCategory(const QString &category) const
{
    QVector<StepDesc> result;
    for (const StepDesc &desc : m_steps) {
        if (desc.category == category)
            result.append(desc);
    }
    return result;
}
