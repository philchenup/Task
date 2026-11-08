/**
 * @file expertprocess.h
 * @brief 焊接专家工艺库对话框。
 *
 * 界面依据 FSWeld2800 免示教软件的专家工艺库，并用 CypWeld 用户手册中
 * 工艺参数、工艺匹配表、多层多道和全局工艺的说明补全产品页未展开的项。
 * 继承 ct::CustomDialog，由 createDialog 弹出：
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
    enum GrooveType
    {
        GrooveNone = 0,
        GrooveNatural,
        GrooveI,
        GrooveSingleV,
        GrooveBoxV,
        GrooveAsymmetricV
    };

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

    struct PassParams
    {
        QString name = QStringLiteral("焊道1（根焊道）");
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

    void addLayer();
    void removeLayer();
    void addPass();
    void removePass();
    void apply();
    void importPackage();
    void exportPackage();
    void recommendOffsets();

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
