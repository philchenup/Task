/**
 * @file expertprocess.h
 * @brief º¸½Ó×¨¼Ò¹¤ÒÕ¿â¶Ô»°¿ò¡£
 *
 * ½çÃæÒÀ¾Ý FSWeld2800 ÃâÊ¾½ÌÈí¼þµÄ×¨¼Ò¹¤ÒÕ¿â£¬²¢ÓÃ CypWeld ÓÃ»§ÊÖ²áÖÐ
 * ¹¤ÒÕ²ÎÊý¡¢¹¤ÒÕÆ¥Åä±í¡¢¶à²ã¶àµÀºÍÈ«¾Ö¹¤ÒÕµÄËµÃ÷²¹È«²úÆ·Ò³Î´Õ¹¿ªµÄÏî¡£
 * ¼Ì³Ð ct::CustomDialog£¬ÓÉ createDialog µ¯³ö£º
 * ct::createDialog<ExpertProcess>(parent, "ExpertProcess", cloudview, cloudtree, console, true);
 */
#ifndef CT_TOOL_EXPERTPROCESS_H
#define CT_TOOL_EXPERTPROCESS_H

#include "base/customdialog.h"

#include <QString>
#include <QVector>

class ExpertProcess : public ct::CustomDialog
{
    Q_OBJECT

public:
    /** @brief 坡口/焊缝截面类型。 */
    enum GrooveType
    {
        GrooveNone = 0,
        GrooveNatural,
        GrooveI,
        GrooveSingleV,
        GrooveBoxV,
        GrooveAsymmetricV
    };

    /** @brief 焊接位置（平焊、立焊等）。 */
    enum WeldPosition
    {
        PosFillet = 0,
        PosFlat,
        PosVerticalUp,
        PosVerticalDown,
        PosBoat,
        PosMultiFillet,
        PosMultiFlat,
        PosMultiVertical
    };

    /** @brief 摆弧波形。 */
    enum WeaveType
    {
        WeaveNone = 0,
        WeaveSine,
        WeavePendulum,
        WeaveL,
        WeaveTriangle
    };

    enum LocateFailPolicy
    {
        FailPrompt = 0,
        FailSkip,
        FailStop
    };

    struct WrapEnd
    {
        QString layerName;
        double pushAngle = 10.0;
    };

    struct WrapCorner
    {
        WrapEnd upFront;
        WrapEnd upBack;
        WrapEnd downFront;
        WrapEnd downBack;
    };

    struct MatchRule
    {
        int weldPosition = PosFillet;
        double thickMin = 4.0;
        double thickMax = 8.0;
        double tiltMin = 0.0;
        double tiltMax = 30.0;
        QString layerName;
        double startDistance = 0.0;
        double endDistance = 0.0;
        bool continuous = false;
        WrapCorner wrap;
    };

    /** @brief 单焊道电弧、摆动、包角等参数。 */
    struct PassParams
    {
        QString name = QStringLiteral("º¸µÀ1£¨¸ùº¸µÀ£©");
        int weldPosition = PosFillet;
        double stickout = 15.0;
        double pushAngle = 0.0;
        double entryAngle = 0.0;
        double exitAngle = 0.0;
        double pitchAngle = 0.0;
        double offsetY = 0.0;
        double offsetZ = 0.0;
        double startOffset = 0.0;

        QString gas = QStringLiteral("80%Ar+20%CO2");
        QString wire = QStringLiteral("ER50-6");
        double wireDiameter = 1.2;
        QString polarity = QStringLiteral("DCEP");

        double arcCurrent = 180.0;
        double arcVoltage = 22.0;
        int arcTime = 200;
        int arcTimeout = 500;
        int preflow = 200;
        double slowWire = 2.0;

        double weldCurrent = 180.0;
        double weldVoltage = 22.0;
        double weldSpeed = 6.0;
        double wireFeed = 8.0;
        double arcLengthTrim = 0.0;
        double inductance = 0.0;

        double endCurrent = 140.0;
        double endVoltage = 18.0;
        int endTime = 400;
        int postflow = 500;
        int burnback = 50;

        int weave = WeaveSine;
        double weaveAmp = 2.0;
        double weaveFreq = 2.0;
        int dwellLeft = 80;
        int dwellRight = 80;
        int dwellCenter = 0;
        double weaveAngle = 0.0;
        bool trackEnable = true;

        double cornerRadius = 5.0;
        int smoothLevel = 3;
        double approachDist = 0.0;
        double departDist = 0.0;

        bool craterEnable = false;
        double craterDistance = 8.0;
        double craterSpeed = 4.0;
        double craterCurrent = 150.0;
        double craterVoltage = 18.0;

        bool rearcEnable = false;
        int retractTime = 200;
        int rearcTime = 500;
        int rearcCount = 2;
        bool scratchEnable = false;
        double scratchDistance = 15.0;
        double scratchSpeed = 10.0;
        bool scratchWeave = false;

        bool cornerEnable = false;
        double cornerBefore = 10.0;
        double cornerAfter = 10.0;
        double cornerRpm = 10.0;
        double cornerCurrent = 170.0;
        double cornerVoltage = 21.0;
        double cornerSpeed = 4.0;
    };

    /** @brief 多层工艺：坡口几何与焊道列表。 */
    struct LayerParams
    {
        QString name;
        QString category;
        QString color = QStringLiteral("#2563EB");
        bool debug = false;
        int groove = GrooveNone;
        double plateThickness = 6.0;
        double grooveAngle = 0.0;
        double grooveAngle2 = 0.0;
        double rootGap = 0.0;
        double rootFace = 0.0;
        int basePlate = 0;
        bool intermittent = false;
        double beadOn = 100.0;
        double beadOff = 50.0;
        bool adaptiveEnable = false;
        bool variableGroove = false;
        double widthComp = 0.0;
        QVector<PassParams> passes;
    };

    /** @brief 全局断弧、寻位、速度规划等开关。 */
    struct GlobalParams
    {
        bool checkBreakpoint = true;
        double breakpointMaxDev = 30.0;
        bool arcTrackIntegral = false;
        bool arcTrackLog = true;
        double samplePhaseDelay = 0.0;
        bool useCoarseDirection = false;
        bool warnVectorDev = true;
        double vectorDevDeg = 5.0;
        bool circleUseModelRadius = true;
        bool warnRadiusDev = true;
        double radiusDev = 2.0;
        bool warnCenterDev = true;
        double centerDev = 2.0;
        int locateFailPolicy = FailPrompt;
        bool axisFilter = true;
        bool speedPlanWeave = true;
    };

    struct Visibility
    {
        bool grooveDetail = false;
        bool poseExtra = false;
        bool offsets = false;
        bool intermittent = false;
        bool motion = false;
        bool crater = false;
        bool rearc = false;
        bool corner = false;
        bool adaptive = false;
        bool globalPage = false;
        bool matchTable = false;
    };

    /** @brief 完整专家工艺库 JSON 根结构。 */
    struct Library
    {
        int version = 1;
        bool keepExpert = false;
        GlobalParams global;
        Visibility visibility;
        QVector<LayerParams> layers;
        QVector<MatchRule> rules;
    };

    explicit ExpertProcess(QWidget* parent = nullptr);
    ~ExpertProcess();

    virtual void init();
    virtual void reset();
    virtual void deinit();

    /** @brief 新增工艺层。 */
    void addLayer();
    void removeLayer();
    void addPass();
    void removePass();
    /** @brief 应用当前编辑到库并写状态。 */
    void apply();
    /** @brief 导入工艺包文件。 */
    void importPackage();
    /** @brief 导出工艺包。 */
    void exportPackage();
    void recommendOffsets();

    /** @brief 只读访问当前 Library。 */
    const Library& library() const;

protected:
    virtual bool eventFilter(QObject* watched, QEvent* event);

private:
    void buildUi();
    void connectSignals();
    void seedLibrary();
    void reloadUi();
    void rebuildTree(int selectIndex);
    void rebuildPasses(int selectIndex);
    void rebuildMatch();
    void refreshLayerCombos();
    void loadLayerFields();
    void loadPass();
    bool commitPass();
    void readGlobal();
    void writeGlobal();
    void readVisibility();
    void writeVisibility();
    void readMatch();
    void applyPermission();
    void updateGrooveWidgets();
    void updateWeaveWidgets();
    void updateAdaptiveHint();
    void refreshStatus();
    void onLayerSelected();
    void onPassSelected(int row);
    void onPassCountChanged(int count);
    void onExpertToggled(bool on);
    void onRenameFinished();
    void changePassword();
    void showHelp();
    void importFromFile(const QString& path, const QString& forcedType);
    void exportToFile(const QString& type);
    bool currentIsDebug() const;
    int nonDebugCount() const;
    QStringList layerNames() const;

    struct Impl;
    Impl* d;
};

#endif // CT_TOOL_EXPERTPROCESS_H