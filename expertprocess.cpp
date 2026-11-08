/**
 * @file expertprocess.cpp
 * @brief 焊接专家工艺库对话框。
 *
 * 产品页只给出工艺库能力：工艺包导入、多坡口、多层多道、单道独立参数、
 * 50mm 以下全熔透。本对话框按 CypWeld 手册把未展开的页面补全：
 * 图层/焊道、焊缝参数、常规工艺、弧坑回填、再起弧、拐角、坡口自适应、
 * 全局工艺和工艺匹配表。
 */
#include "expertprocess.h"

#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHash>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QIcon>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPixmap>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QtMath>

namespace
{

QStringList categoryNames()
{
    return QStringList()
        << QStringLiteral("调试工艺")
        << QStringLiteral("平角焊")
        << QStringLiteral("平焊")
        << QStringLiteral("立向上焊")
        << QStringLiteral("立向下焊")
        << QStringLiteral("多层平角焊")
        << QStringLiteral("多层平焊")
        << QStringLiteral("多层立焊")
        << QStringLiteral("其他");
}

QString passTitle(int index)
{
    if (index <= 0)
        return QStringLiteral("焊道1（根焊道）");
    return QStringLiteral("焊道%1").arg(index + 1);
}

void renamePasses(QVector<ExpertProcess::PassParams>& passes)
{
    for (int i = 0; i < passes.size(); ++i)
        passes[i].name = passTitle(i);
}

QDoubleSpinBox* dspin(double min, double max, double step, int decimals, const QString& suffix)
{
    QDoubleSpinBox* box = new QDoubleSpinBox;
    box->setRange(min, max);
    box->setSingleStep(step);
    box->setDecimals(decimals);
    if (!suffix.isEmpty())
        box->setSuffix(suffix);
    box->setMinimumWidth(118);
    return box;
}

QSpinBox* ispin(int min, int max, int step, const QString& suffix)
{
    QSpinBox* box = new QSpinBox;
    box->setRange(min, max);
    box->setSingleStep(step);
    if (!suffix.isEmpty())
        box->setSuffix(suffix);
    box->setMinimumWidth(118);
    return box;
}

void tip(QWidget* widget, const QString& text)
{
    if (widget)
        widget->setToolTip(text);
}

struct Grid
{
    QGridLayout* g;
    int r;
    int c;

    Grid() : g(new QGridLayout), r(0), c(0)
    {
        g->setHorizontalSpacing(12);
        g->setVerticalSpacing(6);
        g->setColumnStretch(1, 1);
        g->setColumnStretch(3, 1);
    }

    void add(const QString& text, QWidget* widget, const QString& hint)
    {
        QLabel* lab = new QLabel(text);
        tip(lab, hint);
        tip(widget, hint);
        g->addWidget(lab, r, c * 2);
        g->addWidget(widget, r, c * 2 + 1);
        ++c;
        if (c >= 2) {
            c = 0;
            ++r;
        }
    }

    void span(QWidget* widget)
    {
        if (c != 0) {
            c = 0;
            ++r;
        }
        g->addWidget(widget, r, 0, 1, 4);
        ++r;
    }
};

QCheckBox* visibilityBox()
{
    QCheckBox* box = new QCheckBox(QStringLiteral("非专家模式可见"));
    box->setToolTip(QStringLiteral("专家模式下勾选后，操作员可以查看并修改这一组参数。调试工艺不受此限制。"));
    return box;
}

QGroupBox* makeGroup(const QString& title, QCheckBox** vis, QVBoxLayout** layout)
{
    QGroupBox* box = new QGroupBox(title);
    QVBoxLayout* lay = new QVBoxLayout(box);
    lay->setContentsMargins(8, 14, 8, 8);
    if (vis) {
        *vis = visibilityBox();
        lay->addWidget(*vis);
    }
    *layout = lay;
    return box;
}

QScrollArea* scrollOf(QWidget* page)
{
    QScrollArea* area = new QScrollArea;
    area->setWidgetResizable(true);
    area->setFrameShape(QFrame::NoFrame);
    area->setWidget(page);
    return area;
}

void fillGroove(QComboBox* box)
{
    box->addItem(QStringLiteral("无坡口"), ExpertProcess::GrooveNone);
    box->addItem(QStringLiteral("自然坡口"), ExpertProcess::GrooveNatural);
    box->addItem(QStringLiteral("I型坡口"), ExpertProcess::GrooveI);
    box->addItem(QStringLiteral("单V坡口"), ExpertProcess::GrooveSingleV);
    box->addItem(QStringLiteral("箱型柱主焊缝单V坡口"), ExpertProcess::GrooveBoxV);
    box->addItem(QStringLiteral("非对称双边V型坡口"), ExpertProcess::GrooveAsymmetricV);
}

void fillPosition(QComboBox* box)
{
    box->addItem(QStringLiteral("平角焊"), ExpertProcess::PosFillet);
    box->addItem(QStringLiteral("平焊"), ExpertProcess::PosFlat);
    box->addItem(QStringLiteral("立向上焊"), ExpertProcess::PosVerticalUp);
    box->addItem(QStringLiteral("立向下焊"), ExpertProcess::PosVerticalDown);
    box->addItem(QStringLiteral("船形焊"), ExpertProcess::PosBoat);
    box->addItem(QStringLiteral("多层平角焊"), ExpertProcess::PosMultiFillet);
    box->addItem(QStringLiteral("多层平焊"), ExpertProcess::PosMultiFlat);
    box->addItem(QStringLiteral("多层立焊"), ExpertProcess::PosMultiVertical);
}

void fillWeave(QComboBox* box)
{
    box->addItem(QStringLiteral("无"), ExpertProcess::WeaveNone);
    box->addItem(QStringLiteral("正弦摆"), ExpertProcess::WeaveSine);
    box->addItem(QStringLiteral("钟摆"), ExpertProcess::WeavePendulum);
    box->addItem(QStringLiteral("L型摆"), ExpertProcess::WeaveL);
    box->addItem(QStringLiteral("三角摆"), ExpertProcess::WeaveTriangle);
}

void setComboData(QComboBox* box, int data)
{
    int index = box->findData(data);
    box->setCurrentIndex(index < 0 ? 0 : index);
}

int comboData(const QComboBox* box)
{
    return box->currentData().toInt();
}

void setComboText(QComboBox* box, const QString& text)
{
    int index = box->findText(text);
    if (index < 0) {
        box->addItem(text);
        index = box->count() - 1;
    }
    box->setCurrentIndex(index);
}

void put(QJsonObject& obj, const char* key, double value) { obj.insert(QString::fromLatin1(key), value); }
void put(QJsonObject& obj, const char* key, int value) { obj.insert(QString::fromLatin1(key), value); }
void put(QJsonObject& obj, const char* key, bool value) { obj.insert(QString::fromLatin1(key), value); }
void put(QJsonObject& obj, const char* key, const QString& value) { obj.insert(QString::fromLatin1(key), value); }

double jd(const QJsonObject& obj, const char* key, double fallback)
{
    const QJsonValue value = obj.value(QString::fromLatin1(key));
    return value.isDouble() ? value.toDouble() : fallback;
}

int ji(const QJsonObject& obj, const char* key, int fallback)
{
    const QJsonValue value = obj.value(QString::fromLatin1(key));
    return value.isDouble() ? value.toInt() : fallback;
}

bool jb(const QJsonObject& obj, const char* key, bool fallback)
{
    const QJsonValue value = obj.value(QString::fromLatin1(key));
    return value.isBool() ? value.toBool() : fallback;
}

QString js(const QJsonObject& obj, const char* key, const QString& fallback)
{
    const QJsonValue value = obj.value(QString::fromLatin1(key));
    return value.isString() ? value.toString() : fallback;
}

QJsonObject wrapEndToJson(const ExpertProcess::WrapEnd& end)
{
    QJsonObject obj;
    put(obj, "layer", end.layerName);
    put(obj, "pushAngle", end.pushAngle);
    return obj;
}

ExpertProcess::WrapEnd wrapEndFromJson(const QJsonObject& obj, const ExpertProcess::WrapEnd& fallback)
{
    ExpertProcess::WrapEnd end = fallback;
    end.layerName = js(obj, "layer", end.layerName);
    end.pushAngle = jd(obj, "pushAngle", end.pushAngle);
    return end;
}

QJsonObject wrapToJson(const ExpertProcess::WrapCorner& wrap)
{
    QJsonObject obj;
    obj.insert(QStringLiteral("upFront"), wrapEndToJson(wrap.upFront));
    obj.insert(QStringLiteral("upBack"), wrapEndToJson(wrap.upBack));
    obj.insert(QStringLiteral("downFront"), wrapEndToJson(wrap.downFront));
    obj.insert(QStringLiteral("downBack"), wrapEndToJson(wrap.downBack));
    return obj;
}

ExpertProcess::WrapCorner wrapFromJson(const QJsonObject& obj, const ExpertProcess::WrapCorner& fallback)
{
    ExpertProcess::WrapCorner wrap = fallback;
    wrap.upFront = wrapEndFromJson(obj.value(QStringLiteral("upFront")).toObject(), wrap.upFront);
    wrap.upBack = wrapEndFromJson(obj.value(QStringLiteral("upBack")).toObject(), wrap.upBack);
    wrap.downFront = wrapEndFromJson(obj.value(QStringLiteral("downFront")).toObject(), wrap.downFront);
    wrap.downBack = wrapEndFromJson(obj.value(QStringLiteral("downBack")).toObject(), wrap.downBack);
    return wrap;
}

QJsonObject passToJson(const ExpertProcess::PassParams& pass)
{
    QJsonObject obj;
    put(obj, "name", pass.name);
    put(obj, "weldPosition", pass.weldPosition);
    put(obj, "stickout", pass.stickout);
    put(obj, "pushAngle", pass.pushAngle);
    put(obj, "entryAngle", pass.entryAngle);
    put(obj, "exitAngle", pass.exitAngle);
    put(obj, "pitchAngle", pass.pitchAngle);
    put(obj, "offsetY", pass.offsetY);
    put(obj, "offsetZ", pass.offsetZ);
    put(obj, "startOffset", pass.startOffset);
    put(obj, "gas", pass.gas);
    put(obj, "wire", pass.wire);
    put(obj, "wireDiameter", pass.wireDiameter);
    put(obj, "polarity", pass.polarity);
    put(obj, "arcCurrent", pass.arcCurrent);
    put(obj, "arcVoltage", pass.arcVoltage);
    put(obj, "arcTime", pass.arcTime);
    put(obj, "arcTimeout", pass.arcTimeout);
    put(obj, "preflow", pass.preflow);
    put(obj, "slowWire", pass.slowWire);
    put(obj, "weldCurrent", pass.weldCurrent);
    put(obj, "weldVoltage", pass.weldVoltage);
    put(obj, "weldSpeed", pass.weldSpeed);
    put(obj, "wireFeed", pass.wireFeed);
    put(obj, "arcLengthTrim", pass.arcLengthTrim);
    put(obj, "inductance", pass.inductance);
    put(obj, "endCurrent", pass.endCurrent);
    put(obj, "endVoltage", pass.endVoltage);
    put(obj, "endTime", pass.endTime);
    put(obj, "postflow", pass.postflow);
    put(obj, "burnback", pass.burnback);
    put(obj, "weave", pass.weave);
    put(obj, "weaveAmp", pass.weaveAmp);
    put(obj, "weaveFreq", pass.weaveFreq);
    put(obj, "dwellLeft", pass.dwellLeft);
    put(obj, "dwellRight", pass.dwellRight);
    put(obj, "dwellCenter", pass.dwellCenter);
    put(obj, "weaveAngle", pass.weaveAngle);
    put(obj, "trackEnable", pass.trackEnable);
    put(obj, "cornerRadius", pass.cornerRadius);
    put(obj, "smoothLevel", pass.smoothLevel);
    put(obj, "approachDist", pass.approachDist);
    put(obj, "departDist", pass.departDist);
    put(obj, "craterEnable", pass.craterEnable);
    put(obj, "craterDistance", pass.craterDistance);
    put(obj, "craterSpeed", pass.craterSpeed);
    put(obj, "craterCurrent", pass.craterCurrent);
    put(obj, "craterVoltage", pass.craterVoltage);
    put(obj, "rearcEnable", pass.rearcEnable);
    put(obj, "retractTime", pass.retractTime);
    put(obj, "rearcTime", pass.rearcTime);
    put(obj, "rearcCount", pass.rearcCount);
    put(obj, "scratchEnable", pass.scratchEnable);
    put(obj, "scratchDistance", pass.scratchDistance);
    put(obj, "scratchSpeed", pass.scratchSpeed);
    put(obj, "scratchWeave", pass.scratchWeave);
    put(obj, "cornerEnable", pass.cornerEnable);
    put(obj, "cornerBefore", pass.cornerBefore);
    put(obj, "cornerAfter", pass.cornerAfter);
    put(obj, "cornerRpm", pass.cornerRpm);
    put(obj, "cornerCurrent", pass.cornerCurrent);
    put(obj, "cornerVoltage", pass.cornerVoltage);
    put(obj, "cornerSpeed", pass.cornerSpeed);
    return obj;
}

ExpertProcess::PassParams passFromJson(const QJsonObject& obj)
{
    ExpertProcess::PassParams pass;
    pass.name = js(obj, "name", pass.name);
    pass.weldPosition = ji(obj, "weldPosition", pass.weldPosition);
    pass.stickout = jd(obj, "stickout", pass.stickout);
    pass.pushAngle = jd(obj, "pushAngle", pass.pushAngle);
    pass.entryAngle = jd(obj, "entryAngle", pass.entryAngle);
    pass.exitAngle = jd(obj, "exitAngle", pass.exitAngle);
    pass.pitchAngle = jd(obj, "pitchAngle", pass.pitchAngle);
    pass.offsetY = jd(obj, "offsetY", pass.offsetY);
    pass.offsetZ = jd(obj, "offsetZ", pass.offsetZ);
    pass.startOffset = jd(obj, "startOffset", pass.startOffset);
    pass.gas = js(obj, "gas", pass.gas);
    pass.wire = js(obj, "wire", pass.wire);
    pass.wireDiameter = jd(obj, "wireDiameter", pass.wireDiameter);
    pass.polarity = js(obj, "polarity", pass.polarity);
    pass.arcCurrent = jd(obj, "arcCurrent", pass.arcCurrent);
    pass.arcVoltage = jd(obj, "arcVoltage", pass.arcVoltage);
    pass.arcTime = ji(obj, "arcTime", pass.arcTime);
    pass.arcTimeout = ji(obj, "arcTimeout", pass.arcTimeout);
    pass.preflow = ji(obj, "preflow", pass.preflow);
    pass.slowWire = jd(obj, "slowWire", pass.slowWire);
    pass.weldCurrent = jd(obj, "weldCurrent", pass.weldCurrent);
    pass.weldVoltage = jd(obj, "weldVoltage", pass.weldVoltage);
    pass.weldSpeed = jd(obj, "weldSpeed", pass.weldSpeed);
    pass.wireFeed = jd(obj, "wireFeed", pass.wireFeed);
    pass.arcLengthTrim = jd(obj, "arcLengthTrim", pass.arcLengthTrim);
    pass.inductance = jd(obj, "inductance", pass.inductance);
    pass.endCurrent = jd(obj, "endCurrent", pass.endCurrent);
    pass.endVoltage = jd(obj, "endVoltage", pass.endVoltage);
    pass.endTime = ji(obj, "endTime", pass.endTime);
    pass.postflow = ji(obj, "postflow", pass.postflow);
    pass.burnback = ji(obj, "burnback", pass.burnback);
    pass.weave = ji(obj, "weave", pass.weave);
    pass.weaveAmp = jd(obj, "weaveAmp", pass.weaveAmp);
    pass.weaveFreq = jd(obj, "weaveFreq", pass.weaveFreq);
    pass.dwellLeft = ji(obj, "dwellLeft", pass.dwellLeft);
    pass.dwellRight = ji(obj, "dwellRight", pass.dwellRight);
    pass.dwellCenter = ji(obj, "dwellCenter", pass.dwellCenter);
    pass.weaveAngle = jd(obj, "weaveAngle", pass.weaveAngle);
    pass.trackEnable = jb(obj, "trackEnable", pass.trackEnable);
    pass.cornerRadius = jd(obj, "cornerRadius", pass.cornerRadius);
    pass.smoothLevel = ji(obj, "smoothLevel", pass.smoothLevel);
    pass.approachDist = jd(obj, "approachDist", pass.approachDist);
    pass.departDist = jd(obj, "departDist", pass.departDist);
    pass.craterEnable = jb(obj, "craterEnable", pass.craterEnable);
    pass.craterDistance = jd(obj, "craterDistance", pass.craterDistance);
    pass.craterSpeed = jd(obj, "craterSpeed", pass.craterSpeed);
    pass.craterCurrent = jd(obj, "craterCurrent", pass.craterCurrent);
    pass.craterVoltage = jd(obj, "craterVoltage", pass.craterVoltage);
    pass.rearcEnable = jb(obj, "rearcEnable", pass.rearcEnable);
    pass.retractTime = ji(obj, "retractTime", pass.retractTime);
    pass.rearcTime = ji(obj, "rearcTime", pass.rearcTime);
    pass.rearcCount = ji(obj, "rearcCount", pass.rearcCount);
    pass.scratchEnable = jb(obj, "scratchEnable", pass.scratchEnable);
    pass.scratchDistance = jd(obj, "scratchDistance", pass.scratchDistance);
    pass.scratchSpeed = jd(obj, "scratchSpeed", pass.scratchSpeed);
    pass.scratchWeave = jb(obj, "scratchWeave", pass.scratchWeave);
    pass.cornerEnable = jb(obj, "cornerEnable", pass.cornerEnable);
    pass.cornerBefore = jd(obj, "cornerBefore", pass.cornerBefore);
    pass.cornerAfter = jd(obj, "cornerAfter", pass.cornerAfter);
    pass.cornerRpm = jd(obj, "cornerRpm", pass.cornerRpm);
    pass.cornerCurrent = jd(obj, "cornerCurrent", pass.cornerCurrent);
    pass.cornerVoltage = jd(obj, "cornerVoltage", pass.cornerVoltage);
    pass.cornerSpeed = jd(obj, "cornerSpeed", pass.cornerSpeed);
    return pass;
}

QJsonObject layerToJson(const ExpertProcess::LayerParams& layer)
{
    QJsonObject obj;
    put(obj, "name", layer.name);
    put(obj, "category", layer.category);
    put(obj, "color", layer.color);
    put(obj, "debug", layer.debug);
    put(obj, "groove", layer.groove);
    put(obj, "plateThickness", layer.plateThickness);
    put(obj, "grooveAngle", layer.grooveAngle);
    put(obj, "grooveAngle2", layer.grooveAngle2);
    put(obj, "rootGap", layer.rootGap);
    put(obj, "rootFace", layer.rootFace);
    put(obj, "basePlate", layer.basePlate);
    put(obj, "intermittent", layer.intermittent);
    put(obj, "beadOn", layer.beadOn);
    put(obj, "beadOff", layer.beadOff);
    put(obj, "adaptiveEnable", layer.adaptiveEnable);
    put(obj, "variableGroove", layer.variableGroove);
    put(obj, "widthComp", layer.widthComp);
    QJsonArray passes;
    for (int i = 0; i < layer.passes.size(); ++i)
        passes.append(passToJson(layer.passes.at(i)));
    obj.insert(QStringLiteral("passes"), passes);
    return obj;
}

ExpertProcess::LayerParams layerFromJson(const QJsonObject& obj)
{
    ExpertProcess::LayerParams layer;
    layer.name = js(obj, "name", QStringLiteral("未命名工艺"));
    layer.category = js(obj, "category", QStringLiteral("其他"));
    layer.color = js(obj, "color", layer.color);
    layer.debug = jb(obj, "debug", false);
    layer.groove = ji(obj, "groove", layer.groove);
    layer.plateThickness = jd(obj, "plateThickness", layer.plateThickness);
    layer.grooveAngle = jd(obj, "grooveAngle", layer.grooveAngle);
    layer.grooveAngle2 = jd(obj, "grooveAngle2", layer.grooveAngle2);
    layer.rootGap = jd(obj, "rootGap", layer.rootGap);
    layer.rootFace = jd(obj, "rootFace", layer.rootFace);
    layer.basePlate = ji(obj, "basePlate", layer.basePlate);
    layer.intermittent = jb(obj, "intermittent", layer.intermittent);
    layer.beadOn = jd(obj, "beadOn", layer.beadOn);
    layer.beadOff = jd(obj, "beadOff", layer.beadOff);
    layer.adaptiveEnable = jb(obj, "adaptiveEnable", layer.adaptiveEnable);
    layer.variableGroove = jb(obj, "variableGroove", layer.variableGroove);
    layer.widthComp = jd(obj, "widthComp", layer.widthComp);
    const QJsonArray passes = obj.value(QStringLiteral("passes")).toArray();
    for (int i = 0; i < passes.size(); ++i)
        layer.passes.append(passFromJson(passes.at(i).toObject()));
    if (layer.passes.isEmpty())
        layer.passes.append(ExpertProcess::PassParams());
    renamePasses(layer.passes);
    if (!categoryNames().contains(layer.category))
        layer.category = QStringLiteral("其他");
    return layer;
}

QJsonObject ruleToJson(const ExpertProcess::MatchRule& rule)
{
    QJsonObject obj;
    put(obj, "weldPosition", rule.weldPosition);
    put(obj, "thickMin", rule.thickMin);
    put(obj, "thickMax", rule.thickMax);
    put(obj, "tiltMin", rule.tiltMin);
    put(obj, "tiltMax", rule.tiltMax);
    put(obj, "layer", rule.layerName);
    put(obj, "startDistance", rule.startDistance);
    put(obj, "endDistance", rule.endDistance);
    put(obj, "continuous", rule.continuous);
    obj.insert(QStringLiteral("wrap"), wrapToJson(rule.wrap));
    return obj;
}

ExpertProcess::MatchRule ruleFromJson(const QJsonObject& obj)
{
    ExpertProcess::MatchRule rule;
    rule.weldPosition = ji(obj, "weldPosition", rule.weldPosition);
    rule.thickMin = jd(obj, "thickMin", rule.thickMin);
    rule.thickMax = jd(obj, "thickMax", rule.thickMax);
    rule.tiltMin = jd(obj, "tiltMin", rule.tiltMin);
    rule.tiltMax = jd(obj, "tiltMax", rule.tiltMax);
    rule.layerName = js(obj, "layer", rule.layerName);
    rule.startDistance = jd(obj, "startDistance", rule.startDistance);
    rule.endDistance = jd(obj, "endDistance", rule.endDistance);
    rule.continuous = jb(obj, "continuous", rule.continuous);
    rule.wrap = wrapFromJson(obj.value(QStringLiteral("wrap")).toObject(), rule.wrap);
    return rule;
}

QJsonObject globalToJson(const ExpertProcess::GlobalParams& global)
{
    QJsonObject obj;
    put(obj, "checkBreakpoint", global.checkBreakpoint);
    put(obj, "breakpointMaxDev", global.breakpointMaxDev);
    put(obj, "arcTrackIntegral", global.arcTrackIntegral);
    put(obj, "arcTrackLog", global.arcTrackLog);
    put(obj, "samplePhaseDelay", global.samplePhaseDelay);
    put(obj, "useCoarseDirection", global.useCoarseDirection);
    put(obj, "warnVectorDev", global.warnVectorDev);
    put(obj, "vectorDevDeg", global.vectorDevDeg);
    put(obj, "circleUseModelRadius", global.circleUseModelRadius);
    put(obj, "warnRadiusDev", global.warnRadiusDev);
    put(obj, "radiusDev", global.radiusDev);
    put(obj, "warnCenterDev", global.warnCenterDev);
    put(obj, "centerDev", global.centerDev);
    put(obj, "locateFailPolicy", global.locateFailPolicy);
    put(obj, "axisFilter", global.axisFilter);
    put(obj, "speedPlanWeave", global.speedPlanWeave);
    return obj;
}

ExpertProcess::GlobalParams globalFromJson(const QJsonObject& obj)
{
    ExpertProcess::GlobalParams global;
    global.checkBreakpoint = jb(obj, "checkBreakpoint", global.checkBreakpoint);
    global.breakpointMaxDev = jd(obj, "breakpointMaxDev", global.breakpointMaxDev);
    global.arcTrackIntegral = jb(obj, "arcTrackIntegral", global.arcTrackIntegral);
    global.arcTrackLog = jb(obj, "arcTrackLog", global.arcTrackLog);
    global.samplePhaseDelay = jd(obj, "samplePhaseDelay", global.samplePhaseDelay);
    global.useCoarseDirection = jb(obj, "useCoarseDirection", global.useCoarseDirection);
    global.warnVectorDev = jb(obj, "warnVectorDev", global.warnVectorDev);
    global.vectorDevDeg = jd(obj, "vectorDevDeg", global.vectorDevDeg);
    global.circleUseModelRadius = jb(obj, "circleUseModelRadius", global.circleUseModelRadius);
    global.warnRadiusDev = jb(obj, "warnRadiusDev", global.warnRadiusDev);
    global.radiusDev = jd(obj, "radiusDev", global.radiusDev);
    global.warnCenterDev = jb(obj, "warnCenterDev", global.warnCenterDev);
    global.centerDev = jd(obj, "centerDev", global.centerDev);
    global.locateFailPolicy = ji(obj, "locateFailPolicy", global.locateFailPolicy);
    global.axisFilter = jb(obj, "axisFilter", global.axisFilter);
    global.speedPlanWeave = jb(obj, "speedPlanWeave", global.speedPlanWeave);
    return global;
}

QJsonObject visibilityToJson(const ExpertProcess::Visibility& vis)
{
    QJsonObject obj;
    put(obj, "grooveDetail", vis.grooveDetail);
    put(obj, "poseExtra", vis.poseExtra);
    put(obj, "offsets", vis.offsets);
    put(obj, "intermittent", vis.intermittent);
    put(obj, "motion", vis.motion);
    put(obj, "crater", vis.crater);
    put(obj, "rearc", vis.rearc);
    put(obj, "corner", vis.corner);
    put(obj, "adaptive", vis.adaptive);
    put(obj, "globalPage", vis.globalPage);
    put(obj, "matchTable", vis.matchTable);
    return obj;
}

ExpertProcess::Visibility visibilityFromJson(const QJsonObject& obj)
{
    ExpertProcess::Visibility vis;
    vis.grooveDetail = jb(obj, "grooveDetail", vis.grooveDetail);
    vis.poseExtra = jb(obj, "poseExtra", vis.poseExtra);
    vis.offsets = jb(obj, "offsets", vis.offsets);
    vis.intermittent = jb(obj, "intermittent", vis.intermittent);
    vis.motion = jb(obj, "motion", vis.motion);
    vis.crater = jb(obj, "crater", vis.crater);
    vis.rearc = jb(obj, "rearc", vis.rearc);
    vis.corner = jb(obj, "corner", vis.corner);
    vis.adaptive = jb(obj, "adaptive", vis.adaptive);
    vis.globalPage = jb(obj, "globalPage", vis.globalPage);
    vis.matchTable = jb(obj, "matchTable", vis.matchTable);
    return vis;
}

QString fillRecommendedOffsets(ExpertProcess::LayerParams& layer)
{
    QString warn;
    QVector<ExpertProcess::PassParams>& passes = layer.passes;
    const int count = passes.size();
    if (count <= 0)
        return QStringLiteral("当前图层没有焊道。");

    const double sign = (layer.basePlate == 2) ? -1.0 : 1.0;
    if (layer.plateThickness > 50.0)
        warn = QStringLiteral("板厚超过 50mm，已超出全熔透工艺库声明的范围，仍按截面几何写入偏移。");

    if (layer.groove == ExpertProcess::GrooveNone || layer.groove == ExpertProcess::GrooveNatural) {
        const double leg = qBound(2.0, layer.plateThickness * 0.5, 6.0);
        int layers = 1;
        while (layers * (layers + 1) / 2 < count)
            ++layers;
        int index = 0;
        for (int li = 0; li < layers && index < count; ++li) {
            int beads = li + 1;
            if (index + beads > count)
                beads = count - index;
            for (int b = 0; b < beads; ++b, ++index) {
                passes[index].offsetY = sign * (beads - 1 - b) * leg;
                passes[index].offsetZ = b * leg;
            }
        }
        return warn;
    }

    const double depth = qMax(0.1, layer.plateThickness - layer.rootFace);
    double leftDeg = 0.0;
    double rightDeg = 0.0;
    bool openGroove = true;
    if (layer.groove == ExpertProcess::GrooveAsymmetricV) {
        leftDeg = layer.grooveAngle;
        rightDeg = layer.grooveAngle2;
        if (leftDeg <= 0.0 || rightDeg <= 0.0) {
            openGroove = false;
            warn = QStringLiteral("非对称坡口两侧角度需要大于 0，已按 I 型直通焊道分层。");
        }
    } else if (layer.groove == ExpertProcess::GrooveI || layer.grooveAngle <= 0.0) {
        openGroove = false;
        if (layer.groove != ExpertProcess::GrooveI && layer.grooveAngle <= 0.0)
            warn = QStringLiteral("坡口角度为 0，已按 I 型直通焊道分层。");
    } else {
        leftDeg = layer.grooveAngle * 0.5;
        rightDeg = leftDeg;
    }

    if (!openGroove) {
        const double height = depth / count;
        for (int i = 0; i < count; ++i) {
            passes[i].offsetY = 0.0;
            passes[i].offsetZ = (i + 0.5) * height;
        }
        return warn;
    }

    leftDeg = qBound(1.0, leftDeg, 80.0);
    rightDeg = qBound(1.0, rightDeg, 80.0);
    int layers = 1;
    while (layers * (layers + 1) / 2 < count)
        ++layers;
    const double beadH = depth / layers;
    int index = 0;
    for (int li = 0; li < layers && index < count; ++li) {
        int beads = li + 1;
        if (index + beads > count)
            beads = count - index;
        const double height = (li + 0.5) * beadH;
        const double left = height * qTan(qDegreesToRadians(leftDeg));
        const double right = height * qTan(qDegreesToRadians(rightDeg));
        const double width = layer.rootGap + left + right;
        const double step = width / beads;
        for (int b = 0; b < beads; ++b, ++index) {
            const double across = -left + (b + 0.5) * step;
            passes[index].offsetY = sign * across;
            passes[index].offsetZ = height;
        }
    }
    return warn;
}

bool editWrap(QWidget* parent, ExpertProcess::WrapCorner& wrap, const QStringList& layers)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("连续焊包角工艺"));
    dialog.resize(460, 360);
    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QLabel* note = new QLabel(QStringLiteral(
        "包角前后两段可以分别指定工艺图层和最佳推拉角。起终点姿态受限时，"
        "推拉角会由极限姿态过渡到该最佳角度。包角段不启用电弧跟踪。"));
    note->setWordWrap(true);
    layout->addWidget(note);

    struct Row { QComboBox* layer; QDoubleSpinBox* angle; };
    auto addEnd = [&](const QString& title, const ExpertProcess::WrapEnd& end) -> Row {
        QGroupBox* box = new QGroupBox(title);
        QGridLayout* grid = new QGridLayout(box);
        QComboBox* layer = new QComboBox;
        layer->addItems(layers);
        setComboText(layer, end.layerName);
        QDoubleSpinBox* angle = dspin(-90.0, 90.0, 1.0, 1, QStringLiteral(" °"));
        angle->setValue(end.pushAngle);
        grid->addWidget(new QLabel(QStringLiteral("工艺图层")), 0, 0);
        grid->addWidget(layer, 0, 1);
        grid->addWidget(new QLabel(QStringLiteral("最佳推拉角")), 1, 0);
        grid->addWidget(angle, 1, 1);
        layout->addWidget(box);
        Row row;
        row.layer = layer;
        row.angle = angle;
        return row;
    };

    const Row upFront = addEnd(QStringLiteral("立向上包角 · 前段"), wrap.upFront);
    const Row upBack = addEnd(QStringLiteral("立向上包角 · 后段"), wrap.upBack);
    const Row downFront = addEnd(QStringLiteral("包角立向下 · 前段"), wrap.downFront);
    const Row downBack = addEnd(QStringLiteral("包角立向下 · 后段"), wrap.downBack);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted)
        return false;

    wrap.upFront.layerName = upFront.layer->currentText();
    wrap.upFront.pushAngle = upFront.angle->value();
    wrap.upBack.layerName = upBack.layer->currentText();
    wrap.upBack.pushAngle = upBack.angle->value();
    wrap.downFront.layerName = downFront.layer->currentText();
    wrap.downFront.pushAngle = downFront.angle->value();
    wrap.downBack.layerName = downBack.layer->currentText();
    wrap.downBack.pushAngle = downBack.angle->value();
    return true;
}

ExpertProcess::PassParams basePass(int position)
{
    ExpertProcess::PassParams pass;
    pass.weldPosition = position;
    pass.gas = QStringLiteral("80%Ar+20%CO2");
    pass.wire = QStringLiteral("ER50-6");
    pass.wireDiameter = 1.2;
    pass.polarity = QStringLiteral("DCEP");
    pass.stickout = 15.0;
    pass.trackEnable = true;
    pass.smoothLevel = 3;
    return pass;
}

void tuneRoot(ExpertProcess::PassParams& pass)
{
    pass.arcCurrent = 170;
    pass.arcVoltage = 20;
    pass.weldCurrent = 160;
    pass.weldVoltage = 20;
    pass.weldSpeed = 4.0;
    pass.wireFeed = 5.5;
    pass.endCurrent = 130;
    pass.endVoltage = 17;
    pass.weave = ExpertProcess::WeaveNone;
    pass.weaveAmp = 0.0;
    pass.pushAngle = 0.0;
    pass.pitchAngle = 0.0;
}

void tuneFill(ExpertProcess::PassParams& pass)
{
    pass.arcCurrent = 220;
    pass.arcVoltage = 25;
    pass.weldCurrent = 230;
    pass.weldVoltage = 26;
    pass.weldSpeed = 5.5;
    pass.wireFeed = 9.0;
    pass.endCurrent = 180;
    pass.endVoltage = 22;
    pass.weave = ExpertProcess::WeaveSine;
    pass.weaveAmp = 3.0;
    pass.weaveFreq = 2.0;
    pass.dwellLeft = 80;
    pass.dwellRight = 80;
    pass.pushAngle = 5.0;
}

void tuneCap(ExpertProcess::PassParams& pass)
{
    tuneFill(pass);
    pass.weldCurrent = 200;
    pass.weldVoltage = 23.5;
    pass.weldSpeed = 4.5;
    pass.wireFeed = 8.0;
    pass.weaveAmp = 4.5;
    pass.dwellLeft = 120;
    pass.dwellRight = 120;
    pass.pushAngle = 10.0;
}

ExpertProcess::LayerParams makeLayer(const QString& name, const QString& category,
                                      const QString& color, bool debug)
{
    ExpertProcess::LayerParams layer;
    layer.name = name;
    layer.category = category;
    layer.color = color;
    layer.debug = debug;
    return layer;
}

} // namespace

struct ExpertProcess::Impl
{
    bool loading;
    bool ready;
    bool expert;
    int layerIndex;
    int passIndex;
    QString password;
    Library lib;
    Library committed;

    QLabel* title;
    QLabel* status;
    QLabel* categoryLabel;
    QLabel* adaptiveHint;
    QCheckBox* expertCheck;
    QCheckBox* keepCheck;
    QPushButton* btnPassword;
    QPushButton* btnImport;
    QPushButton* btnExport;
    QPushButton* btnHelp;
    QPushButton* btnClose;
    QPushButton* btnAddLayer;
    QPushButton* btnDelLayer;
    QPushButton* btnAddPass;
    QPushButton* btnDelPass;
    QPushButton* btnRecommend;
    QPushButton* btnApply;
    QPushButton* btnReset;
    QTreeWidget* tree;
    QListWidget* passList;
    QSpinBox* passCount;
    QLineEdit* nameEdit;
    QTabWidget* tabs;

    QComboBox* groove;
    QDoubleSpinBox* thickness;
    QDoubleSpinBox* angle;
    QDoubleSpinBox* angle2;
    QDoubleSpinBox* gap;
    QDoubleSpinBox* rootFace;
    QComboBox* basePlate;
    QCheckBox* visGroove;
    QWidget* grooveDetail;

    QComboBox* position;
    QDoubleSpinBox* stickout;
    QDoubleSpinBox* push;
    QDoubleSpinBox* entry;
    QDoubleSpinBox* exitAngle;
    QDoubleSpinBox* pitch;
    QCheckBox* visPose;
    QWidget* poseExtra;

    QDoubleSpinBox* offsetY;
    QDoubleSpinBox* offsetZ;
    QDoubleSpinBox* startOffset;
    QCheckBox* visOffset;
    QWidget* offsetGroup;

    QCheckBox* intermittent;
    QDoubleSpinBox* beadOn;
    QDoubleSpinBox* beadOff;
    QCheckBox* visInter;
    QWidget* interGroup;

    QComboBox* gas;
    QComboBox* wire;
    QComboBox* diameter;
    QComboBox* polarity;

    QDoubleSpinBox* arcCurrent;
    QDoubleSpinBox* arcVoltage;
    QSpinBox* arcTime;
    QSpinBox* arcTimeout;
    QSpinBox* preflow;
    QDoubleSpinBox* slowWire;

    QDoubleSpinBox* weldCurrent;
    QDoubleSpinBox* weldVoltage;
    QDoubleSpinBox* weldSpeed;
    QDoubleSpinBox* wireFeed;
    QDoubleSpinBox* arcTrim;
    QDoubleSpinBox* inductance;

    QDoubleSpinBox* endCurrent;
    QDoubleSpinBox* endVoltage;
    QSpinBox* endTime;
    QSpinBox* postflow;
    QSpinBox* burnback;

    QComboBox* weave;
    QDoubleSpinBox* weaveAmp;
    QDoubleSpinBox* weaveFreq;
    QSpinBox* dwellLeft;
    QSpinBox* dwellRight;
    QSpinBox* dwellCenter;
    QDoubleSpinBox* weaveAngle;
    QCheckBox* trackEnable;

    QDoubleSpinBox* cornerRadius;
    QSpinBox* smoothLevel;
    QDoubleSpinBox* approach;
    QDoubleSpinBox* depart;
    QCheckBox* visMotion;
    QWidget* motionGroup;

    QCheckBox* craterEnable;
    QDoubleSpinBox* craterDistance;
    QDoubleSpinBox* craterSpeed;
    QDoubleSpinBox* craterCurrent;
    QDoubleSpinBox* craterVoltage;
    QCheckBox* visCrater;

    QCheckBox* rearcEnable;
    QSpinBox* retractTime;
    QSpinBox* rearcTime;
    QSpinBox* rearcCount;
    QCheckBox* scratchEnable;
    QDoubleSpinBox* scratchDistance;
    QDoubleSpinBox* scratchSpeed;
    QCheckBox* scratchWeave;
    QCheckBox* visRearc;

    QCheckBox* cornerEnable;
    QDoubleSpinBox* cornerBefore;
    QDoubleSpinBox* cornerAfter;
    QDoubleSpinBox* cornerRpm;
    QDoubleSpinBox* cornerCurrent;
    QDoubleSpinBox* cornerVoltage;
    QDoubleSpinBox* cornerSpeed;
    QCheckBox* visCorner;

    QCheckBox* adaptiveEnable;
    QCheckBox* variableGroove;
    QDoubleSpinBox* widthComp;
    QCheckBox* visAdaptive;

    QCheckBox* checkBp;
    QDoubleSpinBox* bpDev;
    QCheckBox* trackIntegral;
    QCheckBox* trackLog;
    QDoubleSpinBox* phaseDelay;
    QCheckBox* useCoarse;
    QCheckBox* warnVector;
    QDoubleSpinBox* vectorDev;
    QCheckBox* circleModel;
    QCheckBox* warnRadius;
    QDoubleSpinBox* radiusDev;
    QCheckBox* warnCenter;
    QDoubleSpinBox* centerDev;
    QComboBox* failPolicy;
    QCheckBox* axisFilter;
    QCheckBox* speedPlan;
    QCheckBox* visGlobal;

    QTableWidget* matchTable;
    QCheckBox* visMatch;

    int tabCrater;
    int tabRearc;
    int tabCorner;
    int tabAdaptive;
    int tabGlobal;
    int tabMatch;

    Impl()
        : loading(false), ready(false), expert(false), layerIndex(-1), passIndex(-1),
          password(QStringLiteral("123456")),
          title(0), status(0), categoryLabel(0), adaptiveHint(0),
          expertCheck(0), keepCheck(0), btnPassword(0), btnImport(0), btnExport(0),
          btnHelp(0), btnClose(0), btnAddLayer(0), btnDelLayer(0), btnAddPass(0),
          btnDelPass(0), btnRecommend(0), btnApply(0), btnReset(0),
          tree(0), passList(0), passCount(0), nameEdit(0), tabs(0),
          groove(0), thickness(0), angle(0), angle2(0), gap(0), rootFace(0), basePlate(0),
          visGroove(0), grooveDetail(0),
          position(0), stickout(0), push(0), entry(0), exitAngle(0), pitch(0),
          visPose(0), poseExtra(0),
          offsetY(0), offsetZ(0), startOffset(0), visOffset(0), offsetGroup(0),
          intermittent(0), beadOn(0), beadOff(0), visInter(0), interGroup(0),
          gas(0), wire(0), diameter(0), polarity(0),
          arcCurrent(0), arcVoltage(0), arcTime(0), arcTimeout(0), preflow(0), slowWire(0),
          weldCurrent(0), weldVoltage(0), weldSpeed(0), wireFeed(0), arcTrim(0), inductance(0),
          endCurrent(0), endVoltage(0), endTime(0), postflow(0), burnback(0),
          weave(0), weaveAmp(0), weaveFreq(0), dwellLeft(0), dwellRight(0), dwellCenter(0),
          weaveAngle(0), trackEnable(0),
          cornerRadius(0), smoothLevel(0), approach(0), depart(0), visMotion(0), motionGroup(0),
          craterEnable(0), craterDistance(0), craterSpeed(0), craterCurrent(0), craterVoltage(0), visCrater(0),
          rearcEnable(0), retractTime(0), rearcTime(0), rearcCount(0),
          scratchEnable(0), scratchDistance(0), scratchSpeed(0), scratchWeave(0), visRearc(0),
          cornerEnable(0), cornerBefore(0), cornerAfter(0), cornerRpm(0),
          cornerCurrent(0), cornerVoltage(0), cornerSpeed(0), visCorner(0),
          adaptiveEnable(0), variableGroove(0), widthComp(0), visAdaptive(0),
          checkBp(0), bpDev(0), trackIntegral(0), trackLog(0), phaseDelay(0),
          useCoarse(0), warnVector(0), vectorDev(0), circleModel(0),
          warnRadius(0), radiusDev(0), warnCenter(0), centerDev(0),
          failPolicy(0), axisFilter(0), speedPlan(0), visGlobal(0),
          matchTable(0), visMatch(0),
          tabCrater(2), tabRearc(3), tabCorner(4), tabAdaptive(5), tabGlobal(6), tabMatch(7)
    {
    }
};

namespace
{

void setTabShown(QTabWidget* tabs, int index, bool shown)
{
#if QT_VERSION >= QT_VERSION_CHECK(5, 15, 0)
    tabs->setTabVisible(index, shown);
#else
    tabs->setTabEnabled(index, shown);
#endif
}

} // namespace

ExpertProcess::ExpertProcess(QWidget* parent)
    : ct::CustomDialog(parent), d(new Impl)
{
    m_cloudview = nullptr;
    m_cloudtree = nullptr;
    m_console = nullptr;
    center = false;
    setWindowTitle(QStringLiteral("专家工艺库"));
    resize(1120, 760);
    setMinimumSize(980, 640);
    buildUi();
    connectSignals();
    seedLibrary();
    d->committed = d->lib;
    d->ready = true;
    reloadUi();
}

ExpertProcess::~ExpertProcess()
{
    delete d;
}

void ExpertProcess::init()
{
    if (!d->ready) {
        seedLibrary();
        d->committed = d->lib;
        d->ready = true;
        reloadUi();
    }
    if (m_cloudview) {
        m_cloudview->showInfo(QStringLiteral("专家工艺库"), 1);
        m_cloudview->showInfo(QStringLiteral("图层工艺 / 多层多道 / 工艺匹配 / 参数包"), 2);
    }
    if (m_console)
        printI(QStringLiteral("专家工艺库已打开。非专家模式只显示已开放参数，调试工艺始终全部开放。"));
}

void ExpertProcess::reset()
{
    if (!d || !d->ready)
        return;
    const bool keep = d->committed.keepExpert;
    d->lib = d->committed;
    d->lib.keepExpert = keep;
    d->committed.keepExpert = keep;
    reloadUi();
}

void ExpertProcess::deinit()
{
    if (m_cloudview)
        m_cloudview->clearInfo();
}

const ExpertProcess::Library& ExpertProcess::library() const
{
    return d->lib;
}

void ExpertProcess::buildUi()
{
    setStyleSheet(QStringLiteral(
        "QDialog,QWidget{background:#000000;color:#e8e8e8;font-size:12px;}"
        "QGroupBox{background:#000000;border:1px solid #3a3a3a;border-radius:4px;margin-top:12px;padding-top:4px;font-weight:600;color:#f2f2f2;}"
        "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;color:#f2f2f2;}"
        "QLabel{background:transparent;color:#e8e8e8;}"
        "QCheckBox{background:transparent;color:#e8e8e8;}"
        "QTreeWidget,QListWidget,QTableWidget,QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox{"
        "background:#111111;color:#f0f0f0;border:1px solid #3a3a3a;border-radius:2px;padding:1px 4px;}"
        "QComboBox QAbstractItemView{background:#111111;color:#f0f0f0;selection-background-color:#2f6fed;}"
        "QHeaderView::section{background:#1a1a1a;color:#f0f0f0;border:1px solid #3a3a3a;padding:4px;}"
        "QTreeWidget::item:selected,QListWidget::item:selected,QTableWidget::item:selected{background:#2f6fed;color:#ffffff;}"
        "QScrollArea,QScrollArea>QWidget{background:#000000;border:none;}"
        "QTabWidget::pane{border:1px solid #3a3a3a;background:#000000;}"
        "QTabBar::tab{background:#1a1a1a;color:#c8c8c8;padding:6px 12px;margin-right:2px;}"
        "QTabBar::tab:selected{background:#000000;color:#ffffff;font-weight:600;}"
        "QPushButton{background:#1a1a1a;color:#f0f0f0;border:1px solid #4a4a4a;border-radius:3px;padding:4px 10px;}"
        "QPushButton:hover{border-color:#6aa1ff;}"
        "QPushButton#applyBtn{background:#2f6fed;color:#ffffff;border:none;padding:5px 16px;}"
        "QPushButton#closeBtn{color:#ff8a80;}"
        "QMenu{background:#111111;color:#f0f0f0;border:1px solid #3a3a3a;}"
        "QMenu::item:selected{background:#2f6fed;}"
        "QLabel#titleLabel{font-size:16px;font-weight:700;color:#ffffff;}"
        "QLabel#hint{color:#a0a0a0;}"));

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(10, 8, 10, 8);
    root->setSpacing(8);

    QHBoxLayout* header = new QHBoxLayout;
    d->title = new QLabel(QStringLiteral("专家工艺库"));
    d->title->setObjectName(QStringLiteral("titleLabel"));
    d->expertCheck = new QCheckBox(QStringLiteral("专家模式"));
    d->expertCheck->setToolTip(QStringLiteral("初始密码 123456。专家模式可编辑全部参数，并决定操作员能看到哪些项。"));
    d->keepCheck = new QCheckBox(QStringLiteral("保持专家模式打开"));
    d->keepCheck->setToolTip(QStringLiteral("勾选后，关闭本页再次打开将直接进入专家模式。"));
    d->btnPassword = new QPushButton(QStringLiteral("修改密码"));
    d->btnImport = new QPushButton(QStringLiteral("导入"));
    d->btnExport = new QPushButton(QStringLiteral("导出"));
    d->btnHelp = new QPushButton(QStringLiteral("在线帮助"));
    d->btnClose = new QPushButton(QStringLiteral("关闭"));
    d->btnClose->setObjectName(QStringLiteral("closeBtn"));
    header->addWidget(d->title);
    header->addSpacing(12);
    header->addWidget(d->expertCheck);
    header->addWidget(d->keepCheck);
    header->addWidget(d->btnPassword);
    header->addStretch(1);
    header->addWidget(d->btnImport);
    header->addWidget(d->btnExport);
    header->addWidget(d->btnHelp);
    header->addWidget(d->btnClose);
    root->addLayout(header);

    QWidget* left = new QWidget;
    QVBoxLayout* leftLay = new QVBoxLayout(left);
    leftLay->setContentsMargins(0, 0, 0, 0);
    QLabel* layerTitle = new QLabel(QStringLiteral("工艺图层"));
    d->tree = new QTreeWidget;
    d->tree->setHeaderHidden(true);
    d->tree->setMinimumWidth(230);
    QHBoxLayout* layerBtns = new QHBoxLayout;
    d->btnAddLayer = new QPushButton(QStringLiteral("新增工艺"));
    d->btnDelLayer = new QPushButton(QStringLiteral("删除工艺"));
    layerBtns->addWidget(d->btnAddLayer);
    layerBtns->addWidget(d->btnDelLayer);
    QLabel* layerHint = new QLabel(QStringLiteral("调试工艺不可删除。除调试工艺外至少保留一个图层。"));
    layerHint->setObjectName(QStringLiteral("hint"));
    layerHint->setWordWrap(true);
    leftLay->addWidget(layerTitle);
    leftLay->addWidget(d->tree, 1);
    leftLay->addLayout(layerBtns);
    leftLay->addWidget(layerHint);

    QWidget* right = new QWidget;
    QVBoxLayout* rightLay = new QVBoxLayout(right);
    rightLay->setContentsMargins(0, 0, 0, 0);
    QHBoxLayout* nameRow = new QHBoxLayout;
    nameRow->addWidget(new QLabel(QStringLiteral("工艺名称")));
    d->nameEdit = new QLineEdit;
    d->categoryLabel = new QLabel;
    d->categoryLabel->setObjectName(QStringLiteral("hint"));
    nameRow->addWidget(d->nameEdit, 1);
    nameRow->addWidget(d->categoryLabel);
    rightLay->addLayout(nameRow);

    QHBoxLayout* passRow = new QHBoxLayout;
    d->passList = new QListWidget;
    d->passList->setFixedHeight(118);
    QVBoxLayout* passBtns = new QVBoxLayout;
    d->passCount = ispin(1, 40, 1, QString());
    d->btnAddPass = new QPushButton(QStringLiteral("增加焊道"));
    d->btnDelPass = new QPushButton(QStringLiteral("删除焊道"));
    d->btnRecommend = new QPushButton(QStringLiteral("推荐偏移"));
    d->btnRecommend->setToolTip(QStringLiteral(
        "按坡口截面为多层多道写入 Y/Z 偏移。Y 沿坡口宽度，Z 自坡口根部向上；"
        "主板面选板面 B 时 Y 取反。箱型柱主焊缝使用这一组推荐值。"));
    passBtns->addWidget(new QLabel(QStringLiteral("焊道数量")));
    passBtns->addWidget(d->passCount);
    passBtns->addWidget(d->btnAddPass);
    passBtns->addWidget(d->btnDelPass);
    passBtns->addWidget(d->btnRecommend);
    passBtns->addStretch(1);
    passRow->addWidget(d->passList, 1);
    passRow->addLayout(passBtns);
    rightLay->addLayout(passRow);

    d->tabs = new QTabWidget;
    rightLay->addWidget(d->tabs, 1);

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        QVBoxLayout* basicLay = 0;
        QGroupBox* basic = makeGroup(QStringLiteral("坡口与板厚"), 0, &basicLay);
        Grid grid;
        d->groove = new QComboBox;
        fillGroove(d->groove);
        d->thickness = dspin(0.0, 100.0, 0.5, 1, QStringLiteral(" mm"));
        grid.add(QStringLiteral("坡口类型"), d->groove,
                 QStringLiteral("无坡口时角度、间隙和钝边按 0 处理。多层多道支持自然坡口、I 型、单 V、箱型柱主焊缝单 V 和非对称双边 V。"));
        grid.add(QStringLiteral("板厚"), d->thickness,
                 QStringLiteral("工艺匹配按板厚范围选型。全熔透工艺库覆盖 50mm 以下。"));
        basicLay->addLayout(grid.g);
        pageLay->addWidget(basic);

        QVBoxLayout* detailLay = 0;
        d->grooveDetail = makeGroup(QStringLiteral("坡口细节"), &d->visGroove, &detailLay);
        Grid detail;
        d->angle = dspin(0.0, 180.0, 1.0, 1, QStringLiteral(" °"));
        d->angle2 = dspin(0.0, 90.0, 1.0, 1, QStringLiteral(" °"));
        d->gap = dspin(0.0, 20.0, 0.5, 1, QStringLiteral(" mm"));
        d->rootFace = dspin(0.0, 20.0, 0.5, 1, QStringLiteral(" mm"));
        d->basePlate = new QComboBox;
        d->basePlate->addItem(QStringLiteral("自动"), 0);
        d->basePlate->addItem(QStringLiteral("板面A"), 1);
        d->basePlate->addItem(QStringLiteral("板面B"), 2);
        detail.add(QStringLiteral("坡口角度"), d->angle,
                   QStringLiteral("单 V 和箱型柱单 V 填整坡口角度。非对称双边 V 填一侧角度。"));
        detail.add(QStringLiteral("另一侧角度"), d->angle2,
                   QStringLiteral("仅非对称双边 V 使用，填另一侧坡口角度。"));
        detail.add(QStringLiteral("根部间隙"), d->gap, QStringLiteral("坡口根部间隙。无坡口时为 0。"));
        detail.add(QStringLiteral("钝边"), d->rootFace, QStringLiteral("钝边高度。推荐偏移时从钝边以上的坡口深度分层。"));
        detail.add(QStringLiteral("主板面"), d->basePlate,
                   QStringLiteral("子焊道偏移的 Y 正方向。自动与板面 A 同向，板面 B 将 Y 取反。"));
        detailLay->addLayout(detail.g);
        pageLay->addWidget(d->grooveDetail);

        QVBoxLayout* poseLay = 0;
        QGroupBox* pose = makeGroup(QStringLiteral("本焊道姿态"), 0, &poseLay);
        Grid poseGrid;
        d->position = new QComboBox;
        fillPosition(d->position);
        d->stickout = dspin(5.0, 40.0, 0.5, 1, QStringLiteral(" mm"));
        d->push = dspin(-45.0, 45.0, 1.0, 1, QStringLiteral(" °"));
        poseGrid.add(QStringLiteral("焊接位置"), d->position,
                     QStringLiteral("平角焊、平焊、立向上、立向下、船形焊，以及多层位置。子焊道改用不同位置仅在变位机场景生效。"));
        poseGrid.add(QStringLiteral("焊丝长度"), d->stickout,
                     QStringLiteral("干伸长，导电嘴端部到熔池的距离。精度取决于 TCP 和标定用的标准干伸长。"));
        poseGrid.add(QStringLiteral("推拉角"), d->push,
                     QStringLiteral("焊丝与焊缝法向的夹角。0° 表示焊丝与焊缝垂直，作为中间段的最佳推拉角。"));
        poseLay->addLayout(poseGrid.g);
        pageLay->addWidget(pose);

        QVBoxLayout* extraLay = 0;
        d->poseExtra = makeGroup(QStringLiteral("起终点姿态"), &d->visPose, &extraLay);
        Grid extra;
        d->entry = dspin(-45.0, 45.0, 1.0, 1, QStringLiteral(" °"));
        d->exitAngle = dspin(-45.0, 45.0, 1.0, 1, QStringLiteral(" °"));
        d->pitch = dspin(-90.0, 90.0, 1.0, 1, QStringLiteral(" °"));
        extra.add(QStringLiteral("入刀角度"), d->entry,
                  QStringLiteral("焊缝起点处的推拉角，随后过渡到最佳推拉角。"));
        extra.add(QStringLiteral("出刀角度"), d->exitAngle,
                  QStringLiteral("焊缝终点处的推拉角。"));
        extra.add(QStringLiteral("俯仰角"), d->pitch,
                  QStringLiteral("角焊缝常用 45°。根焊道若重新计算姿态，自动俯仰角会覆盖手工值。"));
        extraLay->addLayout(extra.g);
        pageLay->addWidget(d->poseExtra);

        QVBoxLayout* offLay = 0;
        d->offsetGroup = makeGroup(QStringLiteral("焊道偏移"), &d->visOffset, &offLay);
        Grid off;
        d->offsetY = dspin(-80.0, 80.0, 0.1, 2, QStringLiteral(" mm"));
        d->offsetZ = dspin(-20.0, 80.0, 0.1, 2, QStringLiteral(" mm"));
        d->startOffset = dspin(-50.0, 50.0, 0.5, 1, QStringLiteral(" mm"));
        off.add(QStringLiteral("Y 偏移"), d->offsetY, QStringLiteral("沿坡口宽度方向的焊道偏移，方向与示意图和主板面一致。"));
        off.add(QStringLiteral("Z 偏移"), d->offsetZ, QStringLiteral("自坡口根部向上的焊道偏移。"));
        off.add(QStringLiteral("焊接起点"), d->startOffset, QStringLiteral("本焊道起点相对焊缝起点的偏移。"));
        offLay->addLayout(off.g);
        pageLay->addWidget(d->offsetGroup);

        QVBoxLayout* interLay = 0;
        d->interGroup = makeGroup(QStringLiteral("断续焊"), &d->visInter, &interLay);
        Grid inter;
        d->intermittent = new QCheckBox(QStringLiteral("启用断续焊"));
        d->beadOn = dspin(5.0, 2000.0, 5.0, 1, QStringLiteral(" mm"));
        d->beadOff = dspin(5.0, 2000.0, 5.0, 1, QStringLiteral(" mm"));
        inter.span(d->intermittent);
        inter.add(QStringLiteral("满焊长度"), d->beadOn,
                  QStringLiteral("断续焊满焊段长度。曲线和连续焊会在拐角处自动连接满焊。"));
        inter.add(QStringLiteral("间隔长度"), d->beadOff, QStringLiteral("断续焊间隔段长度。"));
        tip(d->intermittent, QStringLiteral("写入本图层后，匹配该工艺时可带出断续焊节距。"));
        interLay->addLayout(inter.g);
        pageLay->addWidget(d->interGroup);
        pageLay->addStretch(1);
        d->tabs->addTab(scrollOf(page), QStringLiteral("焊缝参数"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);

        QVBoxLayout* consLay = 0;
        QGroupBox* cons = makeGroup(QStringLiteral("保护气与焊丝"), 0, &consLay);
        Grid consGrid;
        d->gas = new QComboBox;
        d->gas->setEditable(true);
        d->gas->addItems(QStringList()
                         << QStringLiteral("100%CO2")
                         << QStringLiteral("80%Ar+20%CO2")
                         << QStringLiteral("82%Ar+18%CO2")
                         << QStringLiteral("98%Ar+2%CO2")
                         << QStringLiteral("98%Ar+2%O2"));
        d->wire = new QComboBox;
        d->wire->setEditable(true);
        d->wire->addItems(QStringList()
                          << QStringLiteral("ER50-6")
                          << QStringLiteral("ER70S-6")
                          << QStringLiteral("ER70S-G"));
        d->diameter = new QComboBox;
        d->diameter->addItems(QStringList()
                              << QStringLiteral("0.8")
                              << QStringLiteral("1.0")
                              << QStringLiteral("1.2")
                              << QStringLiteral("1.6"));
        d->polarity = new QComboBox;
        d->polarity->setEditable(true);
        d->polarity->addItems(QStringList() << QStringLiteral("DCEP") << QStringLiteral("DCEN"));
        consGrid.add(QStringLiteral("保护气"), d->gas, QStringLiteral("随工艺包保存，便于工位之间复制同一套工艺。"));
        consGrid.add(QStringLiteral("焊丝"), d->wire, QStringLiteral("焊丝牌号。"));
        consGrid.add(QStringLiteral("焊丝直径"), d->diameter, QStringLiteral("单位 mm。"));
        consGrid.add(QStringLiteral("极性"), d->polarity, QStringLiteral("气体保护焊常用 DCEP。"));
        consLay->addLayout(consGrid.g);
        pageLay->addWidget(cons);

        QVBoxLayout* arcLay = 0;
        QGroupBox* arc = makeGroup(QStringLiteral("引弧"), 0, &arcLay);
        Grid arcGrid;
        d->arcCurrent = dspin(0.0, 600.0, 1.0, 1, QStringLiteral(" A"));
        d->arcVoltage = dspin(0.0, 50.0, 0.1, 1, QStringLiteral(" V"));
        d->arcTime = ispin(0, 5000, 10, QStringLiteral(" ms"));
        d->arcTimeout = ispin(0, 5000, 10, QStringLiteral(" ms"));
        d->preflow = ispin(0, 5000, 10, QStringLiteral(" ms"));
        d->slowWire = dspin(0.0, 22.0, 0.1, 1, QStringLiteral(" m/min"));
        arcGrid.add(QStringLiteral("引弧电流"), d->arcCurrent, QStringLiteral("起弧阶段电流。"));
        arcGrid.add(QStringLiteral("引弧电压"), d->arcVoltage, QStringLiteral("起弧阶段电压。"));
        arcGrid.add(QStringLiteral("引弧时间"), d->arcTime, QStringLiteral("起弧参数保持时间。"));
        arcGrid.add(QStringLiteral("起弧超时"), d->arcTimeout, QStringLiteral("超时未起弧则失败。再起弧时间一般与此一致。"));
        arcGrid.add(QStringLiteral("预送气"), d->preflow, QStringLiteral("引弧前保护气提前送出的时间。"));
        arcGrid.add(QStringLiteral("慢送丝"), d->slowWire, QStringLiteral("起弧阶段的送丝速度。"));
        arcLay->addLayout(arcGrid.g);
        pageLay->addWidget(arc);

        QVBoxLayout* weldLay = 0;
        QGroupBox* weld = makeGroup(QStringLiteral("焊接"), 0, &weldLay);
        Grid weldGrid;
        d->weldCurrent = dspin(0.0, 600.0, 1.0, 1, QStringLiteral(" A"));
        d->weldVoltage = dspin(0.0, 50.0, 0.1, 1, QStringLiteral(" V"));
        d->weldSpeed = dspin(0.1, 50.0, 0.1, 2, QStringLiteral(" mm/s"));
        d->wireFeed = dspin(0.0, 30.0, 0.1, 1, QStringLiteral(" m/min"));
        d->arcTrim = dspin(-10.0, 10.0, 0.1, 1, QString());
        d->inductance = dspin(0.0, 100.0, 1.0, 0, QStringLiteral(" %"));
        weldGrid.add(QStringLiteral("焊接电流"), d->weldCurrent, QStringLiteral("稳态焊接电流。"));
        weldGrid.add(QStringLiteral("焊接电压"), d->weldVoltage, QStringLiteral("稳态焊接电压。"));
        weldGrid.add(QStringLiteral("焊接速度"), d->weldSpeed, QStringLiteral("沿焊缝的焊接速度。"));
        weldGrid.add(QStringLiteral("送丝速度"), d->wireFeed, QStringLiteral("一元化焊机可与电流联动，这里按工艺包单独保存。"));
        weldGrid.add(QStringLiteral("弧长修正"), d->arcTrim, QStringLiteral("正值弧长变长。范围以焊机协议为准。"));
        weldGrid.add(QStringLiteral("电感"), d->inductance, QStringLiteral("影响熔滴过渡和飞溅，范围以焊机协议为准。"));
        weldLay->addLayout(weldGrid.g);
        pageLay->addWidget(weld);

        QVBoxLayout* endLay = 0;
        QGroupBox* end = makeGroup(QStringLiteral("收弧"), 0, &endLay);
        Grid endGrid;
        d->endCurrent = dspin(0.0, 600.0, 1.0, 1, QStringLiteral(" A"));
        d->endVoltage = dspin(0.0, 50.0, 0.1, 1, QStringLiteral(" V"));
        d->endTime = ispin(0, 5000, 10, QStringLiteral(" ms"));
        d->postflow = ispin(0, 8000, 10, QStringLiteral(" ms"));
        d->burnback = ispin(0, 1000, 5, QStringLiteral(" ms"));
        endGrid.add(QStringLiteral("收弧电流"), d->endCurrent, QStringLiteral("收弧阶段电流。弧坑回填完成后再执行这组收弧。"));
        endGrid.add(QStringLiteral("收弧电压"), d->endVoltage, QStringLiteral("收弧阶段电压。"));
        endGrid.add(QStringLiteral("收弧时间"), d->endTime, QStringLiteral("收弧参数保持时间。"));
        endGrid.add(QStringLiteral("滞后送气"), d->postflow, QStringLiteral("熄弧后保护气保持时间。"));
        endGrid.add(QStringLiteral("回烧时间"), d->burnback, QStringLiteral("熄弧后回抽焊丝，避免粘丝。"));
        endLay->addLayout(endGrid.g);
        pageLay->addWidget(end);

        QVBoxLayout* weaveLay = 0;
        QGroupBox* weave = makeGroup(QStringLiteral("摆动与电弧跟踪"), 0, &weaveLay);
        Grid weaveGrid;
        d->weave = new QComboBox;
        fillWeave(d->weave);
        d->weaveAmp = dspin(0.0, 20.0, 0.1, 2, QStringLiteral(" mm"));
        d->weaveFreq = dspin(0.0, 10.0, 0.1, 2, QStringLiteral(" Hz"));
        d->dwellLeft = ispin(0, 2000, 10, QStringLiteral(" ms"));
        d->dwellRight = ispin(0, 2000, 10, QStringLiteral(" ms"));
        d->dwellCenter = ispin(0, 2000, 10, QStringLiteral(" ms"));
        d->weaveAngle = dspin(0.0, 180.0, 1.0, 1, QStringLiteral(" °"));
        d->trackEnable = new QCheckBox(QStringLiteral("启用电弧跟踪"));
        weaveGrid.add(QStringLiteral("摆动方式"), d->weave,
                      QStringLiteral("正弦摆、钟摆、L 型摆、三角摆。薄板和非直线焊缝可把摆幅收到 1mm。"));
        weaveGrid.add(QStringLiteral("摆幅"), d->weaveAmp,
                      QStringLiteral("单侧摆动幅度。低摆幅 1mm 时仍可配合电弧跟踪，薄板热变形精度目标在 1mm 以内。"));
        weaveGrid.add(QStringLiteral("频率"), d->weaveFreq, QStringLiteral("每秒摆动次数。"));
        weaveGrid.add(QStringLiteral("左停留"), d->dwellLeft, QStringLiteral("摆到左侧的停留时间，用于保证侧壁熔合。"));
        weaveGrid.add(QStringLiteral("右停留"), d->dwellRight, QStringLiteral("摆到右侧的停留时间。"));
        weaveGrid.add(QStringLiteral("中心停留"), d->dwellCenter, QStringLiteral("经过中心时的停留时间。"));
        weaveGrid.add(QStringLiteral("摆动角"), d->weaveAngle, QStringLiteral("L 型摆和三角摆使用的摆动张角。"));
        weaveGrid.span(d->trackEnable);
        tip(d->trackEnable, QStringLiteral("包角段不启用电弧跟踪。全局采样延时和日志在“全局工艺”中设置。"));
        weaveLay->addLayout(weaveGrid.g);
        pageLay->addWidget(weave);

        QVBoxLayout* motionLay = 0;
        d->motionGroup = makeGroup(QStringLiteral("运动"), &d->visMotion, &motionLay);
        Grid motion;
        d->cornerRadius = dspin(0.0, 50.0, 0.5, 1, QStringLiteral(" mm"));
        d->smoothLevel = ispin(1, 5, 1, QString());
        d->approach = dspin(0.0, 200.0, 1.0, 1, QStringLiteral(" mm"));
        d->depart = dspin(0.0, 200.0, 1.0, 1, QStringLiteral(" mm"));
        motion.add(QStringLiteral("拐角半径"), d->cornerRadius, QStringLiteral("连续轨迹拐角处的过渡半径。"));
        motion.add(QStringLiteral("平顺等级"), d->smoothLevel,
                   QStringLiteral("1 到 5。等级越高姿态变化越平缓，并参与喷嘴冗余下的防碰撞。"));
        motion.add(QStringLiteral("前变刀距离"), d->approach, QStringLiteral("进入本焊道前开始改变焊枪姿态的距离。"));
        motion.add(QStringLiteral("后变刀距离"), d->depart, QStringLiteral("离开本焊道前开始改变焊枪姿态的距离。"));
        motionLay->addLayout(motion.g);
        pageLay->addWidget(d->motionGroup);
        pageLay->addStretch(1);
        d->tabs->addTab(scrollOf(page), QStringLiteral("常规工艺"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        QVBoxLayout* lay = 0;
        QGroupBox* box = makeGroup(QStringLiteral("弧坑回填"), &d->visCrater, &lay);
        QLabel* note = new QLabel(QStringLiteral(
            "焊接过快时在起终点回焊一段距离，把弧坑填满，然后自动改回常规收弧。"
            "只用于走图纸，不用于示教焊接。"));
        note->setWordWrap(true);
        lay->addWidget(note);
        Grid grid;
        d->craterEnable = new QCheckBox(QStringLiteral("启用弧坑回填"));
        d->craterDistance = dspin(0.0, 80.0, 0.5, 1, QStringLiteral(" mm"));
        d->craterSpeed = dspin(0.1, 30.0, 0.1, 2, QStringLiteral(" mm/s"));
        d->craterCurrent = dspin(0.0, 600.0, 1.0, 1, QStringLiteral(" A"));
        d->craterVoltage = dspin(0.0, 50.0, 0.1, 1, QStringLiteral(" V"));
        grid.span(d->craterEnable);
        grid.add(QStringLiteral("回焊距离"), d->craterDistance, QStringLiteral("沿主焊缝反向回焊的长度。"));
        grid.add(QStringLiteral("回填速度"), d->craterSpeed, QStringLiteral("回焊时的速度。"));
        grid.add(QStringLiteral("回填电流"), d->craterCurrent, QStringLiteral("回填阶段电流。"));
        grid.add(QStringLiteral("回填电压"), d->craterVoltage, QStringLiteral("回填阶段电压。"));
        lay->addLayout(grid.g);
        pageLay->addWidget(box);
        pageLay->addStretch(1);
        d->tabCrater = d->tabs->addTab(scrollOf(page), QStringLiteral("弧坑回填"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        QVBoxLayout* lay = 0;
        QGroupBox* box = makeGroup(QStringLiteral("再起弧与刮擦起弧"), &d->visRearc, &lay);
        QLabel* note = new QLabel(QStringLiteral(
            "起点有锈蚀、间隙或识别偏差导致起弧失败时使用。两者都开启时先原地再起弧，"
            "失败后再沿焊缝刮擦起弧。只用于走图纸，不用于示教焊接。"));
        note->setWordWrap(true);
        lay->addWidget(note);
        Grid grid;
        d->rearcEnable = new QCheckBox(QStringLiteral("启用再起弧"));
        d->retractTime = ispin(0, 2000, 10, QStringLiteral(" ms"));
        d->rearcTime = ispin(0, 5000, 10, QStringLiteral(" ms"));
        d->rearcCount = ispin(1, 5, 1, QString());
        d->scratchEnable = new QCheckBox(QStringLiteral("启用刮擦起弧"));
        d->scratchDistance = dspin(0.0, 80.0, 1.0, 1, QStringLiteral(" mm"));
        d->scratchSpeed = dspin(1.0, 50.0, 0.5, 1, QStringLiteral(" mm/s"));
        d->scratchWeave = new QCheckBox(QStringLiteral("刮擦时摆动"));
        grid.span(d->rearcEnable);
        grid.add(QStringLiteral("抽丝时间"), d->retractTime,
                 QStringLiteral("起弧失败后回抽焊丝的时间。过长时熔球可能堵住导电嘴。"));
        grid.add(QStringLiteral("再起弧时间"), d->rearcTime,
                 QStringLiteral("抽丝后再次送丝的时间，一般与常规工艺的起弧超时一致。"));
        grid.add(QStringLiteral("再起弧次数"), d->rearcCount, QStringLiteral("原地伸缩焊丝、敲掉氧化皮的重复次数。"));
        grid.span(d->scratchEnable);
        grid.add(QStringLiteral("刮擦距离"), d->scratchDistance,
                 QStringLiteral("沿焊缝空移到无药皮位置起弧，再反向回到原起弧点。"));
        grid.add(QStringLiteral("刮擦速度"), d->scratchSpeed, QStringLiteral("刮擦返回起弧点的速度。"));
        grid.span(d->scratchWeave);
        tip(d->scratchWeave, QStringLiteral("刮擦起弧过程中允许使用本焊道的摆动参数。"));
        lay->addLayout(grid.g);
        pageLay->addWidget(box);
        pageLay->addStretch(1);
        d->tabRearc = d->tabs->addTab(scrollOf(page), QStringLiteral("再起弧"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        QVBoxLayout* lay = 0;
        QGroupBox* box = makeGroup(QStringLiteral("拐角工艺"), &d->visCorner, &lay);
        QLabel* note = new QLabel(QStringLiteral(
            "连续焊从本焊缝拐到下一条时，拐角段调用本焊缝的拐角工艺。"
            "最大转速用来压住姿态突变，建议 5–20 RPM，过低会把焊接速度一起拉下来。"));
        note->setWordWrap(true);
        lay->addWidget(note);
        Grid grid;
        d->cornerEnable = new QCheckBox(QStringLiteral("启用拐角工艺"));
        d->cornerBefore = dspin(0.0, 100.0, 1.0, 1, QStringLiteral(" mm"));
        d->cornerAfter = dspin(0.0, 100.0, 1.0, 1, QStringLiteral(" mm"));
        d->cornerRpm = dspin(0.0, 100.0, 1.0, 1, QStringLiteral(" RPM"));
        d->cornerCurrent = dspin(0.0, 600.0, 1.0, 1, QStringLiteral(" A"));
        d->cornerVoltage = dspin(0.0, 50.0, 0.1, 1, QStringLiteral(" V"));
        d->cornerSpeed = dspin(0.1, 30.0, 0.1, 2, QStringLiteral(" mm/s"));
        grid.span(d->cornerEnable);
        grid.add(QStringLiteral("拐角前距离"), d->cornerBefore, QStringLiteral("到达拐角前，拐角工艺开始生效的距离。"));
        grid.add(QStringLiteral("拐角后距离"), d->cornerAfter, QStringLiteral("离开拐角后，拐角工艺继续生效的距离。"));
        grid.add(QStringLiteral("最大转速"), d->cornerRpm, QStringLiteral("限制拐角处焊枪姿态变化，建议 5–20 RPM。"));
        grid.add(QStringLiteral("拐角电流"), d->cornerCurrent, QStringLiteral("拐角段电流。摆动仍沿用本焊道常规工艺。"));
        grid.add(QStringLiteral("拐角电压"), d->cornerVoltage, QStringLiteral("拐角段电压。"));
        grid.add(QStringLiteral("拐角速度"), d->cornerSpeed, QStringLiteral("拐角段焊接速度。"));
        lay->addLayout(grid.g);
        pageLay->addWidget(box);
        pageLay->addStretch(1);
        d->tabCorner = d->tabs->addTab(scrollOf(page), QStringLiteral("拐角工艺"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        QVBoxLayout* lay = 0;
        QGroupBox* box = makeGroup(QStringLiteral("坡口自适应"), &d->visAdaptive, &lay);
        d->adaptiveHint = new QLabel;
        d->adaptiveHint->setWordWrap(true);
        lay->addWidget(d->adaptiveHint);
        Grid grid;
        d->adaptiveEnable = new QCheckBox(QStringLiteral("启用坡口自适应"));
        d->variableGroove = new QCheckBox(QStringLiteral("变坡口"));
        d->widthComp = dspin(-10.0, 10.0, 0.1, 2, QStringLiteral(" mm"));
        grid.span(d->adaptiveEnable);
        grid.span(d->variableGroove);
        grid.add(QStringLiteral("宽度补偿"), d->widthComp,
                 QStringLiteral("在视觉测得的坡口宽度上叠加补偿，用来适应拼装造成的宽窄不一。"));
        tip(d->adaptiveEnable, QStringLiteral("用于朝上的箱型柱主焊缝。视觉扫描变坡口后按实测宽度焊接。"));
        tip(d->variableGroove, QStringLiteral("坡口沿焊缝方向宽度变化时打开。"));
        lay->addLayout(grid.g);
        pageLay->addWidget(box);
        pageLay->addStretch(1);
        d->tabAdaptive = d->tabs->addTab(scrollOf(page), QStringLiteral("坡口自适应"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        QVBoxLayout* lay = 0;
        QGroupBox* box = makeGroup(QStringLiteral("全局工艺"), &d->visGlobal, &lay);
        QLabel* note = new QLabel(QStringLiteral("这组参数属于整个工艺库，不随单个焊道变化。"));
        note->setWordWrap(true);
        lay->addWidget(note);
        Grid grid;
        d->checkBp = new QCheckBox(QStringLiteral("加工前检查断点"));
        d->bpDev = dspin(0.0, 200.0, 1.0, 1, QStringLiteral(" mm"));
        d->trackIntegral = new QCheckBox(QStringLiteral("电弧跟踪计入停留时间"));
        d->trackLog = new QCheckBox(QStringLiteral("记录电弧跟踪日志"));
        d->phaseDelay = dspin(0.0, 500.0, 1.0, 1, QStringLiteral(" ms"));
        d->useCoarse = new QCheckBox(QStringLiteral("使用初定位的工件方向"));
        d->warnVector = new QCheckBox(QStringLiteral("寻位方向偏差警告"));
        d->vectorDev = dspin(0.0, 45.0, 0.5, 1, QStringLiteral(" °"));
        d->circleModel = new QCheckBox(QStringLiteral("整圆寻位采用图纸半径"));
        d->warnRadius = new QCheckBox(QStringLiteral("圆半径偏差警告"));
        d->radiusDev = dspin(0.0, 20.0, 0.1, 1, QStringLiteral(" mm"));
        d->warnCenter = new QCheckBox(QStringLiteral("圆心偏差警告"));
        d->centerDev = dspin(0.0, 20.0, 0.1, 1, QStringLiteral(" mm"));
        d->failPolicy = new QComboBox;
        d->failPolicy->addItem(QStringLiteral("弹窗提示"), FailPrompt);
        d->failPolicy->addItem(QStringLiteral("直接跳过"), FailSkip);
        d->failPolicy->addItem(QStringLiteral("停止加工"), FailStop);
        d->axisFilter = new QCheckBox(QStringLiteral("外部轴启停滤波"));
        d->speedPlan = new QCheckBox(QStringLiteral("速度规划方式摆动"));
        grid.span(d->checkBp);
        grid.add(QStringLiteral("断点最大偏差"), d->bpDev,
                 QStringLiteral("把当前位置设为再加工断点时允许的 TCP 偏移上限。"));
        grid.span(d->trackIntegral);
        tip(d->trackIntegral, QStringLiteral("停留时间参与电弧跟踪积分。默认关闭。"));
        grid.span(d->trackLog);
        tip(d->trackLog, QStringLiteral("记录寻位类型、起点、方向夹角和电弧跟踪累计偏移。建议打开。"));
        grid.add(QStringLiteral("采样相位延时"), d->phaseDelay,
                 QStringLiteral("系统延时测定后写入，用于修正电弧跟踪采样。"));
        grid.span(d->useCoarse);
        tip(d->useCoarse, QStringLiteral("焊接方向使用初定位结果。不勾选时使用每条焊缝的寻位方向。"));
        grid.span(d->warnVector);
        grid.add(QStringLiteral("方向偏差阈值"), d->vectorDev, QStringLiteral("寻位方向与初定位方向超过该角度时给出警告。"));
        grid.span(d->circleModel);
        tip(d->circleModel, QStringLiteral("圆弧半径使用图纸值。模型和实物偏差大时再取消。"));
        grid.span(d->warnRadius);
        grid.add(QStringLiteral("半径偏差阈值"), d->radiusDev, QStringLiteral("扫描圆与模型圆的半径差超过该值时警告。"));
        grid.span(d->warnCenter);
        grid.add(QStringLiteral("圆心偏差阈值"), d->centerDev, QStringLiteral("扫描圆与模型圆的圆心差超过该值时警告。"));
        grid.add(QStringLiteral("寻位失败"), d->failPolicy,
                 QStringLiteral("弹窗确认、直接跳过本条焊缝，或停止加工。"));
        grid.span(d->axisFilter);
        tip(d->axisFilter, QStringLiteral("地轨、悬臂、龙门启停时减小抖动。"));
        grid.span(d->speedPlan);
        tip(d->speedPlan, QStringLiteral("用速度规划加滤波提高摆动稳定性。建议打开。"));
        lay->addLayout(grid.g);
        pageLay->addWidget(box);
        pageLay->addStretch(1);
        d->tabGlobal = d->tabs->addTab(scrollOf(page), QStringLiteral("全局工艺"));
    }

    {
        QWidget* page = new QWidget;
        QVBoxLayout* pageLay = new QVBoxLayout(page);
        d->visMatch = visibilityBox();
        pageLay->addWidget(d->visMatch);
        QLabel* note = new QLabel(QStringLiteral(
            "自动出图和手动生成焊缝时，按焊接位置、板厚和倾角匹配工艺图层，并可带出起终点距离。"
            "勾选连续焊后可设置立向上包角和包角立向下的前后段工艺。"));
        note->setWordWrap(true);
        pageLay->addWidget(note);
        d->matchTable = new QTableWidget(0, 10);
        d->matchTable->setHorizontalHeaderLabels(QStringList()
                                                 << QStringLiteral("焊接位置")
                                                 << QStringLiteral("板厚下限")
                                                 << QStringLiteral("板厚上限")
                                                 << QStringLiteral("倾角下限")
                                                 << QStringLiteral("倾角上限")
                                                 << QStringLiteral("工艺图层")
                                                 << QStringLiteral("起点距离")
                                                 << QStringLiteral("终点距离")
                                                 << QStringLiteral("连续焊")
                                                 << QStringLiteral("包角"));
        d->matchTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        d->matchTable->verticalHeader()->setVisible(false);
        d->matchTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        d->matchTable->setMinimumHeight(220);
        pageLay->addWidget(d->matchTable, 1);
        QHBoxLayout* btns = new QHBoxLayout;
        QPushButton* addRule = new QPushButton(QStringLiteral("添加规则"));
        QPushButton* delRule = new QPushButton(QStringLiteral("删除规则"));
        btns->addWidget(addRule);
        btns->addWidget(delRule);
        btns->addStretch(1);
        pageLay->addLayout(btns);
        connect(addRule, &QPushButton::clicked, this, [this]() {
            if (d->loading)
                return;
            readMatch();
            MatchRule rule;
            if (d->layerIndex >= 0 && d->layerIndex < d->lib.layers.size())
                rule.layerName = d->lib.layers.at(d->layerIndex).name;
            rule.wrap.upFront.layerName = rule.layerName;
            rule.wrap.upBack.layerName = rule.layerName;
            rule.wrap.downFront.layerName = rule.layerName;
            rule.wrap.downBack.layerName = rule.layerName;
            d->lib.rules.append(rule);
            rebuildMatch();
        });
        connect(delRule, &QPushButton::clicked, this, [this]() {
            if (d->loading)
                return;
            const int row = d->matchTable->currentRow();
            if (row < 0)
                return;
            readMatch();
            if (row < d->lib.rules.size())
                d->lib.rules.removeAt(row);
            rebuildMatch();
        });
        d->tabMatch = d->tabs->addTab(page, QStringLiteral("工艺匹配表"));
    }

    QSplitter* split = new QSplitter;
    split->addWidget(left);
    split->addWidget(right);
    split->setStretchFactor(0, 0);
    split->setStretchFactor(1, 1);
    split->setSizes(QList<int>() << 250 << 860);
    root->addWidget(split, 1);

    QHBoxLayout* footer = new QHBoxLayout;
    d->status = new QLabel;
    d->status->setObjectName(QStringLiteral("hint"));
    d->btnApply = new QPushButton(QStringLiteral("应用"));
    d->btnApply->setObjectName(QStringLiteral("applyBtn"));
    d->btnReset = new QPushButton(QStringLiteral("重置"));
    footer->addWidget(d->status, 1);
    footer->addWidget(d->btnReset);
    footer->addWidget(d->btnApply);
    root->addLayout(footer);

    QMenu* inMenu = new QMenu(d->btnImport);
    inMenu->addAction(QStringLiteral("工艺包"), this, [this]() { importFromFile(QString(), QStringLiteral("package")); });
    inMenu->addAction(QStringLiteral("焊缝工艺到当前图层"), this, [this]() { importFromFile(QString(), QStringLiteral("layer")); });
    inMenu->addAction(QStringLiteral("焊道工艺到当前焊道"), this, [this]() { importFromFile(QString(), QStringLiteral("pass")); });
    d->btnImport->setMenu(inMenu);
    QMenu* outMenu = new QMenu(d->btnExport);
    outMenu->addAction(QStringLiteral("工艺包"), this, [this]() { exportToFile(QStringLiteral("package")); });
    outMenu->addAction(QStringLiteral("当前焊缝工艺"), this, [this]() { exportToFile(QStringLiteral("layer")); });
    outMenu->addAction(QStringLiteral("当前焊道工艺"), this, [this]() { exportToFile(QStringLiteral("pass")); });
    d->btnExport->setMenu(outMenu);
}

void ExpertProcess::connectSignals()
{
    connect(d->btnClose, &QPushButton::clicked, this, &QDialog::close);
    connect(d->btnApply, &QPushButton::clicked, this, &ExpertProcess::apply);
    connect(d->btnReset, &QPushButton::clicked, this, [this]() {
        reset();
        if (m_console)
            printI(QStringLiteral("已恢复到上次应用的工艺。"));
    });
    connect(d->btnHelp, &QPushButton::clicked, this, &ExpertProcess::showHelp);
    connect(d->btnPassword, &QPushButton::clicked, this, &ExpertProcess::changePassword);
    connect(d->btnAddLayer, &QPushButton::clicked, this, &ExpertProcess::addLayer);
    connect(d->btnDelLayer, &QPushButton::clicked, this, &ExpertProcess::removeLayer);
    connect(d->btnAddPass, &QPushButton::clicked, this, &ExpertProcess::addPass);
    connect(d->btnDelPass, &QPushButton::clicked, this, &ExpertProcess::removePass);
    connect(d->btnRecommend, &QPushButton::clicked, this, &ExpertProcess::recommendOffsets);
    connect(d->expertCheck, &QCheckBox::toggled, this, &ExpertProcess::onExpertToggled);
    connect(d->keepCheck, &QCheckBox::toggled, this, [this](bool on) {
        if (d->loading)
            return;
        if (on && !d->expert) {
            d->loading = true;
            d->keepCheck->setChecked(false);
            d->loading = false;
            return;
        }
        d->lib.keepExpert = on;
        d->committed.keepExpert = on;
    });
    connect(d->tree, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem*, QTreeWidgetItem*) {
        onLayerSelected();
    });
    connect(d->passList, &QListWidget::currentRowChanged, this, &ExpertProcess::onPassSelected);
    connect(d->passCount, QOverload<int>::of(&QSpinBox::valueChanged), this, &ExpertProcess::onPassCountChanged);
    connect(d->nameEdit, &QLineEdit::editingFinished, this, &ExpertProcess::onRenameFinished);
    connect(d->groove, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        updateGrooveWidgets();
        updateAdaptiveHint();
        if (d->loading)
            return;
        const int groove = comboData(d->groove);
        if (groove == GrooveNone) {
            d->angle->setValue(0);
            d->angle2->setValue(0);
            d->gap->setValue(0);
            d->rootFace->setValue(0);
        } else if (groove == GrooveAsymmetricV) {
            if (d->angle->value() <= 0.0)
                d->angle->setValue(30);
            if (d->angle2->value() <= 0.0)
                d->angle2->setValue(30);
        } else if ((groove == GrooveSingleV || groove == GrooveBoxV) && d->angle->value() <= 0.0) {
            d->angle->setValue(60);
        }
    });
    connect(d->weave, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        updateWeaveWidgets();
    });
    connect(d->intermittent, &QCheckBox::toggled, this, [this](bool on) {
        d->beadOn->setEnabled(on);
        d->beadOff->setEnabled(on);
    });
    connect(d->craterEnable, &QCheckBox::toggled, this, [this](bool on) {
        d->craterDistance->setEnabled(on);
        d->craterSpeed->setEnabled(on);
        d->craterCurrent->setEnabled(on);
        d->craterVoltage->setEnabled(on);
    });
    connect(d->rearcEnable, &QCheckBox::toggled, this, [this](bool on) {
        d->retractTime->setEnabled(on);
        d->rearcTime->setEnabled(on);
        d->rearcCount->setEnabled(on);
    });
    connect(d->scratchEnable, &QCheckBox::toggled, this, [this](bool on) {
        d->scratchDistance->setEnabled(on);
        d->scratchSpeed->setEnabled(on);
        d->scratchWeave->setEnabled(on);
    });
    connect(d->cornerEnable, &QCheckBox::toggled, this, [this](bool on) {
        d->cornerBefore->setEnabled(on);
        d->cornerAfter->setEnabled(on);
        d->cornerRpm->setEnabled(on);
        d->cornerCurrent->setEnabled(on);
        d->cornerVoltage->setEnabled(on);
        d->cornerSpeed->setEnabled(on);
    });
}

void ExpertProcess::seedLibrary()
{
    d->lib = Library();
    d->lib.keepExpert = false;

    LayerParams debug = makeLayer(QStringLiteral("调试工艺"), QStringLiteral("调试工艺"),
                                  QStringLiteral("#64748B"), true);
    debug.plateThickness = 6;
    debug.passes.append(basePass(PosFillet));
    debug.passes[0].weldCurrent = 180;
    debug.passes[0].weldVoltage = 22;
    debug.passes[0].weldSpeed = 5;
    debug.passes[0].wireFeed = 7;
    debug.passes[0].weave = WeaveSine;
    debug.passes[0].weaveAmp = 2;
    debug.passes[0].pushAngle = 10;
    debug.passes[0].pitchAngle = 45;
    debug.passes[0].cornerEnable = true;

    LayerParams fillet = makeLayer(QStringLiteral("平角焊-6mm"), QStringLiteral("平角焊"),
                                   QStringLiteral("#2563EB"), false);
    fillet.plateThickness = 6;
    fillet.passes.append(basePass(PosFillet));
    fillet.passes[0].weldCurrent = 180;
    fillet.passes[0].arcCurrent = 190;
    fillet.passes[0].weldVoltage = 22;
    fillet.passes[0].weldSpeed = 6;
    fillet.passes[0].wireFeed = 7.5;
    fillet.passes[0].weave = WeaveSine;
    fillet.passes[0].weaveAmp = 2;
    fillet.passes[0].weaveFreq = 2;
    fillet.passes[0].pushAngle = 10;
    fillet.passes[0].entryAngle = 15;
    fillet.passes[0].exitAngle = 15;
    fillet.passes[0].pitchAngle = 45;
    fillet.passes[0].cornerEnable = true;
    fillet.passes[0].cornerRpm = 12;

    LayerParams flat = makeLayer(QStringLiteral("平焊-8mm"), QStringLiteral("平焊"),
                                 QStringLiteral("#059669"), false);
    flat.groove = GrooveI;
    flat.plateThickness = 8;
    flat.rootGap = 1;
    flat.passes.append(basePass(PosFlat));
    flat.passes[0].weldCurrent = 210;
    flat.passes[0].weldVoltage = 24;
    flat.passes[0].weldSpeed = 7;
    flat.passes[0].wireFeed = 8.5;
    flat.passes[0].weave = WeaveNone;
    flat.passes[0].weaveAmp = 0;
    flat.passes[0].pushAngle = 8;

    LayerParams vertical = makeLayer(QStringLiteral("立向上-10mm"), QStringLiteral("立向上焊"),
                                     QStringLiteral("#D97706"), false);
    vertical.plateThickness = 10;
    vertical.passes.append(basePass(PosVerticalUp));
    vertical.passes[0].weldCurrent = 140;
    vertical.passes[0].weldVoltage = 19;
    vertical.passes[0].weldSpeed = 3.2;
    vertical.passes[0].wireFeed = 5.0;
    vertical.passes[0].weave = WeaveTriangle;
    vertical.passes[0].weaveAmp = 3;
    vertical.passes[0].weaveFreq = 1.2;
    vertical.passes[0].dwellLeft = 200;
    vertical.passes[0].dwellRight = 200;
    vertical.passes[0].weaveAngle = 60;
    vertical.passes[0].pushAngle = 5;
    vertical.passes[0].entryAngle = 15;
    vertical.passes[0].exitAngle = 10;
    vertical.passes[0].pitchAngle = 45;
    vertical.passes[0].cornerEnable = true;
    vertical.passes[0].cornerRpm = 8;
    vertical.passes[0].cornerSpeed = 2.5;

    LayerParams box = makeLayer(QStringLiteral("箱型柱主焊缝-32mm"), QStringLiteral("多层平焊"),
                                QStringLiteral("#7C3AED"), false);
    box.groove = GrooveBoxV;
    box.plateThickness = 32;
    box.grooveAngle = 45;
    box.rootGap = 2;
    box.rootFace = 2;
    box.adaptiveEnable = true;
    for (int i = 0; i < 6; ++i) {
        PassParams pass = basePass(PosMultiFlat);
        if (i == 0)
            tuneRoot(pass);
        else if (i == 5)
            tuneCap(pass);
        else
            tuneFill(pass);
        box.passes.append(pass);
    }
    fillRecommendedOffsets(box);

    LayerParams full = makeLayer(QStringLiteral("单V全熔透-50mm"), QStringLiteral("多层平焊"),
                                 QStringLiteral("#DC2626"), false);
    full.groove = GrooveSingleV;
    full.plateThickness = 50;
    full.grooveAngle = 60;
    full.rootGap = 2;
    full.rootFace = 2;
    for (int i = 0; i < 10; ++i) {
        PassParams pass = basePass(PosMultiFlat);
        if (i == 0)
            tuneRoot(pass);
        else if (i == 9)
            tuneCap(pass);
        else
            tuneFill(pass);
        pass.trackEnable = true;
        full.passes.append(pass);
    }
    fillRecommendedOffsets(full);

    renamePasses(debug.passes);
    renamePasses(fillet.passes);
    renamePasses(flat.passes);
    renamePasses(vertical.passes);
    renamePasses(box.passes);
    renamePasses(full.passes);
    d->lib.layers << debug << fillet << flat << vertical << box << full;

    MatchRule filletRule;
    filletRule.weldPosition = PosFillet;
    filletRule.thickMin = 4;
    filletRule.thickMax = 8;
    filletRule.tiltMin = -15;
    filletRule.tiltMax = 20;
    filletRule.layerName = fillet.name;
    filletRule.wrap.upFront.layerName = vertical.name;
    filletRule.wrap.upFront.pushAngle = 10;
    filletRule.wrap.upBack.layerName = fillet.name;
    filletRule.wrap.upBack.pushAngle = 10;
    filletRule.wrap.downFront.layerName = fillet.name;
    filletRule.wrap.downBack.layerName = vertical.name;

    MatchRule flatRule;
    flatRule.weldPosition = PosFlat;
    flatRule.thickMin = 6;
    flatRule.thickMax = 12;
    flatRule.tiltMin = -10;
    flatRule.tiltMax = 10;
    flatRule.layerName = flat.name;

    MatchRule upRule;
    upRule.weldPosition = PosVerticalUp;
    upRule.thickMin = 6;
    upRule.thickMax = 14;
    upRule.tiltMin = 60;
    upRule.tiltMax = 100;
    upRule.layerName = vertical.name;

    MatchRule boxRule;
    boxRule.weldPosition = PosMultiFlat;
    boxRule.thickMin = 20;
    boxRule.thickMax = 40;
    boxRule.tiltMin = -10;
    boxRule.tiltMax = 10;
    boxRule.layerName = box.name;

    MatchRule fullRule;
    fullRule.weldPosition = PosMultiFlat;
    fullRule.thickMin = 40;
    fullRule.thickMax = 50;
    fullRule.tiltMin = -10;
    fullRule.tiltMax = 10;
    fullRule.layerName = full.name;

    d->lib.rules << filletRule << flatRule << upRule << boxRule << fullRule;
    d->layerIndex = 1;
    d->passIndex = 0;
}

QStringList ExpertProcess::layerNames() const
{
    QStringList names;
    for (int i = 0; i < d->lib.layers.size(); ++i)
        names << d->lib.layers.at(i).name;
    return names;
}

bool ExpertProcess::currentIsDebug() const
{
    return d->layerIndex >= 0 && d->layerIndex < d->lib.layers.size()
        && d->lib.layers.at(d->layerIndex).debug;
}

int ExpertProcess::nonDebugCount() const
{
    int count = 0;
    for (int i = 0; i < d->lib.layers.size(); ++i) {
        if (!d->lib.layers.at(i).debug)
            ++count;
    }
    return count;
}

void ExpertProcess::refreshStatus()
{
    if (!d->status || d->layerIndex < 0 || d->layerIndex >= d->lib.layers.size())
        return;
    const LayerParams& layer = d->lib.layers.at(d->layerIndex);
    QString passName;
    if (d->passIndex >= 0 && d->passIndex < layer.passes.size())
        passName = layer.passes.at(d->passIndex).name;
    const QString mode = d->expert ? QStringLiteral("专家模式") : QStringLiteral("操作员模式");
    d->status->setText(QStringLiteral("%1  |  %2 / %3 / %4")
                           .arg(mode, layer.category, layer.name, passName));
}

void ExpertProcess::reloadUi()
{
    d->loading = true;
    if (d->lib.layers.isEmpty())
        seedLibrary();
    if (d->layerIndex < 0 || d->layerIndex >= d->lib.layers.size())
        d->layerIndex = 0;
    if (d->lib.layers.at(d->layerIndex).passes.isEmpty())
        d->lib.layers[d->layerIndex].passes.append(PassParams());
    if (d->passIndex < 0 || d->passIndex >= d->lib.layers.at(d->layerIndex).passes.size())
        d->passIndex = 0;
    d->expert = d->expert || d->lib.keepExpert;
    d->expertCheck->setChecked(d->expert);
    d->keepCheck->setChecked(d->lib.keepExpert);
    rebuildTree(d->layerIndex);
    rebuildPasses(d->passIndex);
    loadLayerFields();
    loadPass();
    writeGlobal();
    writeVisibility();
    rebuildMatch();
    d->loading = false;
    updateGrooveWidgets();
    updateWeaveWidgets();
    updateAdaptiveHint();
    applyPermission();
    refreshStatus();
}

void ExpertProcess::rebuildTree(int selectIndex)
{
    QSignalBlocker blocker(d->tree);
    d->tree->clear();
    QHash<QString, QTreeWidgetItem*> nodes;
    const QStringList cats = categoryNames();
    for (int i = 0; i < cats.size(); ++i) {
        QTreeWidgetItem* node = new QTreeWidgetItem(d->tree, QStringList(cats.at(i)));
        node->setData(0, Qt::UserRole, -1);
        node->setFlags(node->flags() & ~Qt::ItemIsSelectable);
        QFont font = node->font(0);
        font.setBold(true);
        node->setFont(0, font);
        nodes.insert(cats.at(i), node);
    }
    QTreeWidgetItem* selected = 0;
    for (int i = 0; i < d->lib.layers.size(); ++i) {
        const LayerParams& layer = d->lib.layers.at(i);
        QTreeWidgetItem* parent = nodes.value(layer.category, nodes.value(QStringLiteral("其他")));
        QTreeWidgetItem* item = new QTreeWidgetItem(parent, QStringList(layer.name));
        item->setData(0, Qt::UserRole, i);
        QPixmap pix(12, 12);
        pix.fill(QColor(layer.color));
        item->setIcon(0, QIcon(pix));
        if (i == selectIndex)
            selected = item;
    }
    d->tree->expandAll();
    if (selected)
        d->tree->setCurrentItem(selected);
}

void ExpertProcess::rebuildPasses(int selectIndex)
{
    QSignalBlocker blocker(d->passList);
    d->passList->clear();
    if (d->layerIndex < 0 || d->layerIndex >= d->lib.layers.size())
        return;
    const LayerParams& layer = d->lib.layers.at(d->layerIndex);
    for (int i = 0; i < layer.passes.size(); ++i)
        d->passList->addItem(layer.passes.at(i).name);
    if (selectIndex >= 0 && selectIndex < d->passList->count())
        d->passList->setCurrentRow(selectIndex);
}

void ExpertProcess::loadLayerFields()
{
    const LayerParams& layer = d->lib.layers.at(d->layerIndex);
    d->nameEdit->setText(layer.name);
    d->categoryLabel->setText(layer.category);
    setComboData(d->groove, layer.groove);
    d->thickness->setValue(layer.plateThickness);
    d->angle->setValue(layer.grooveAngle);
    d->angle2->setValue(layer.grooveAngle2);
    d->gap->setValue(layer.rootGap);
    d->rootFace->setValue(layer.rootFace);
    setComboData(d->basePlate, layer.basePlate);
    d->intermittent->setChecked(layer.intermittent);
    d->beadOn->setValue(layer.beadOn);
    d->beadOff->setValue(layer.beadOff);
    d->beadOn->setEnabled(layer.intermittent);
    d->beadOff->setEnabled(layer.intermittent);
    d->adaptiveEnable->setChecked(layer.adaptiveEnable);
    d->variableGroove->setChecked(layer.variableGroove);
    d->widthComp->setValue(layer.widthComp);
    QSignalBlocker blocker(d->passCount);
    d->passCount->setValue(layer.passes.size());
}

void ExpertProcess::loadPass()
{
    const PassParams& pass = d->lib.layers.at(d->layerIndex).passes.at(d->passIndex);
    setComboData(d->position, pass.weldPosition);
    d->stickout->setValue(pass.stickout);
    d->push->setValue(pass.pushAngle);
    d->entry->setValue(pass.entryAngle);
    d->exitAngle->setValue(pass.exitAngle);
    d->pitch->setValue(pass.pitchAngle);
    d->offsetY->setValue(pass.offsetY);
    d->offsetZ->setValue(pass.offsetZ);
    d->startOffset->setValue(pass.startOffset);
    setComboText(d->gas, pass.gas);
    setComboText(d->wire, pass.wire);
    setComboText(d->diameter, QString::number(pass.wireDiameter, 'f', 1));
    setComboText(d->polarity, pass.polarity);
    d->arcCurrent->setValue(pass.arcCurrent);
    d->arcVoltage->setValue(pass.arcVoltage);
    d->arcTime->setValue(pass.arcTime);
    d->arcTimeout->setValue(pass.arcTimeout);
    d->preflow->setValue(pass.preflow);
    d->slowWire->setValue(pass.slowWire);
    d->weldCurrent->setValue(pass.weldCurrent);
    d->weldVoltage->setValue(pass.weldVoltage);
    d->weldSpeed->setValue(pass.weldSpeed);
    d->wireFeed->setValue(pass.wireFeed);
    d->arcTrim->setValue(pass.arcLengthTrim);
    d->inductance->setValue(pass.inductance);
    d->endCurrent->setValue(pass.endCurrent);
    d->endVoltage->setValue(pass.endVoltage);
    d->endTime->setValue(pass.endTime);
    d->postflow->setValue(pass.postflow);
    d->burnback->setValue(pass.burnback);
    setComboData(d->weave, pass.weave);
    d->weaveAmp->setValue(pass.weaveAmp);
    d->weaveFreq->setValue(pass.weaveFreq);
    d->dwellLeft->setValue(pass.dwellLeft);
    d->dwellRight->setValue(pass.dwellRight);
    d->dwellCenter->setValue(pass.dwellCenter);
    d->weaveAngle->setValue(pass.weaveAngle);
    d->trackEnable->setChecked(pass.trackEnable);
    d->cornerRadius->setValue(pass.cornerRadius);
    d->smoothLevel->setValue(pass.smoothLevel);
    d->approach->setValue(pass.approachDist);
    d->depart->setValue(pass.departDist);
    d->craterEnable->setChecked(pass.craterEnable);
    d->craterDistance->setValue(pass.craterDistance);
    d->craterSpeed->setValue(pass.craterSpeed);
    d->craterCurrent->setValue(pass.craterCurrent);
    d->craterVoltage->setValue(pass.craterVoltage);
    d->craterDistance->setEnabled(pass.craterEnable);
    d->craterSpeed->setEnabled(pass.craterEnable);
    d->craterCurrent->setEnabled(pass.craterEnable);
    d->craterVoltage->setEnabled(pass.craterEnable);
    d->rearcEnable->setChecked(pass.rearcEnable);
    d->retractTime->setValue(pass.retractTime);
    d->rearcTime->setValue(pass.rearcTime);
    d->rearcCount->setValue(pass.rearcCount);
    d->retractTime->setEnabled(pass.rearcEnable);
    d->rearcTime->setEnabled(pass.rearcEnable);
    d->rearcCount->setEnabled(pass.rearcEnable);
    d->scratchEnable->setChecked(pass.scratchEnable);
    d->scratchDistance->setValue(pass.scratchDistance);
    d->scratchSpeed->setValue(pass.scratchSpeed);
    d->scratchWeave->setChecked(pass.scratchWeave);
    d->scratchDistance->setEnabled(pass.scratchEnable);
    d->scratchSpeed->setEnabled(pass.scratchEnable);
    d->scratchWeave->setEnabled(pass.scratchEnable);
    d->cornerEnable->setChecked(pass.cornerEnable);
    d->cornerBefore->setValue(pass.cornerBefore);
    d->cornerAfter->setValue(pass.cornerAfter);
    d->cornerRpm->setValue(pass.cornerRpm);
    d->cornerCurrent->setValue(pass.cornerCurrent);
    d->cornerVoltage->setValue(pass.cornerVoltage);
    d->cornerSpeed->setValue(pass.cornerSpeed);
    const bool corner = pass.cornerEnable;
    d->cornerBefore->setEnabled(corner);
    d->cornerAfter->setEnabled(corner);
    d->cornerRpm->setEnabled(corner);
    d->cornerCurrent->setEnabled(corner);
    d->cornerVoltage->setEnabled(corner);
    d->cornerSpeed->setEnabled(corner);
    updateWeaveWidgets();
}

bool ExpertProcess::commitPass()
{
    if (d->loading)
        return true;
    if (d->layerIndex < 0 || d->layerIndex >= d->lib.layers.size())
        return false;
    LayerParams& layer = d->lib.layers[d->layerIndex];
    layer.groove = comboData(d->groove);
    layer.plateThickness = d->thickness->value();
    layer.grooveAngle = d->angle->value();
    layer.grooveAngle2 = d->angle2->value();
    layer.rootGap = d->gap->value();
    layer.rootFace = d->rootFace->value();
    layer.basePlate = comboData(d->basePlate);
    layer.intermittent = d->intermittent->isChecked();
    layer.beadOn = d->beadOn->value();
    layer.beadOff = d->beadOff->value();
    layer.adaptiveEnable = d->adaptiveEnable->isChecked();
    layer.variableGroove = d->variableGroove->isChecked();
    layer.widthComp = d->widthComp->value();
    if (d->passIndex < 0 || d->passIndex >= layer.passes.size())
        return false;
    PassParams& pass = layer.passes[d->passIndex];
    pass.weldPosition = comboData(d->position);
    pass.stickout = d->stickout->value();
    pass.pushAngle = d->push->value();
    pass.entryAngle = d->entry->value();
    pass.exitAngle = d->exitAngle->value();
    pass.pitchAngle = d->pitch->value();
    pass.offsetY = d->offsetY->value();
    pass.offsetZ = d->offsetZ->value();
    pass.startOffset = d->startOffset->value();
    pass.gas = d->gas->currentText().trimmed();
    pass.wire = d->wire->currentText().trimmed();
    pass.wireDiameter = d->diameter->currentText().toDouble();
    pass.polarity = d->polarity->currentText().trimmed();
    pass.arcCurrent = d->arcCurrent->value();
    pass.arcVoltage = d->arcVoltage->value();
    pass.arcTime = d->arcTime->value();
    pass.arcTimeout = d->arcTimeout->value();
    pass.preflow = d->preflow->value();
    pass.slowWire = d->slowWire->value();
    pass.weldCurrent = d->weldCurrent->value();
    pass.weldVoltage = d->weldVoltage->value();
    pass.weldSpeed = d->weldSpeed->value();
    pass.wireFeed = d->wireFeed->value();
    pass.arcLengthTrim = d->arcTrim->value();
    pass.inductance = d->inductance->value();
    pass.endCurrent = d->endCurrent->value();
    pass.endVoltage = d->endVoltage->value();
    pass.endTime = d->endTime->value();
    pass.postflow = d->postflow->value();
    pass.burnback = d->burnback->value();
    pass.weave = comboData(d->weave);
    pass.weaveAmp = d->weaveAmp->value();
    pass.weaveFreq = d->weaveFreq->value();
    pass.dwellLeft = d->dwellLeft->value();
    pass.dwellRight = d->dwellRight->value();
    pass.dwellCenter = d->dwellCenter->value();
    pass.weaveAngle = d->weaveAngle->value();
    pass.trackEnable = d->trackEnable->isChecked();
    pass.cornerRadius = d->cornerRadius->value();
    pass.smoothLevel = d->smoothLevel->value();
    pass.approachDist = d->approach->value();
    pass.departDist = d->depart->value();
    pass.craterEnable = d->craterEnable->isChecked();
    pass.craterDistance = d->craterDistance->value();
    pass.craterSpeed = d->craterSpeed->value();
    pass.craterCurrent = d->craterCurrent->value();
    pass.craterVoltage = d->craterVoltage->value();
    pass.rearcEnable = d->rearcEnable->isChecked();
    pass.retractTime = d->retractTime->value();
    pass.rearcTime = d->rearcTime->value();
    pass.rearcCount = d->rearcCount->value();
    pass.scratchEnable = d->scratchEnable->isChecked();
    pass.scratchDistance = d->scratchDistance->value();
    pass.scratchSpeed = d->scratchSpeed->value();
    pass.scratchWeave = d->scratchWeave->isChecked();
    pass.cornerEnable = d->cornerEnable->isChecked();
    pass.cornerBefore = d->cornerBefore->value();
    pass.cornerAfter = d->cornerAfter->value();
    pass.cornerRpm = d->cornerRpm->value();
    pass.cornerCurrent = d->cornerCurrent->value();
    pass.cornerVoltage = d->cornerVoltage->value();
    pass.cornerSpeed = d->cornerSpeed->value();
    return true;
}

void ExpertProcess::readGlobal()
{
    GlobalParams& global = d->lib.global;
    global.checkBreakpoint = d->checkBp->isChecked();
    global.breakpointMaxDev = d->bpDev->value();
    global.arcTrackIntegral = d->trackIntegral->isChecked();
    global.arcTrackLog = d->trackLog->isChecked();
    global.samplePhaseDelay = d->phaseDelay->value();
    global.useCoarseDirection = d->useCoarse->isChecked();
    global.warnVectorDev = d->warnVector->isChecked();
    global.vectorDevDeg = d->vectorDev->value();
    global.circleUseModelRadius = d->circleModel->isChecked();
    global.warnRadiusDev = d->warnRadius->isChecked();
    global.radiusDev = d->radiusDev->value();
    global.warnCenterDev = d->warnCenter->isChecked();
    global.centerDev = d->centerDev->value();
    global.locateFailPolicy = comboData(d->failPolicy);
    global.axisFilter = d->axisFilter->isChecked();
    global.speedPlanWeave = d->speedPlan->isChecked();
}

void ExpertProcess::writeGlobal()
{
    const GlobalParams& global = d->lib.global;
    d->checkBp->setChecked(global.checkBreakpoint);
    d->bpDev->setValue(global.breakpointMaxDev);
    d->trackIntegral->setChecked(global.arcTrackIntegral);
    d->trackLog->setChecked(global.arcTrackLog);
    d->phaseDelay->setValue(global.samplePhaseDelay);
    d->useCoarse->setChecked(global.useCoarseDirection);
    d->warnVector->setChecked(global.warnVectorDev);
    d->vectorDev->setValue(global.vectorDevDeg);
    d->circleModel->setChecked(global.circleUseModelRadius);
    d->warnRadius->setChecked(global.warnRadiusDev);
    d->radiusDev->setValue(global.radiusDev);
    d->warnCenter->setChecked(global.warnCenterDev);
    d->centerDev->setValue(global.centerDev);
    setComboData(d->failPolicy, global.locateFailPolicy);
    d->axisFilter->setChecked(global.axisFilter);
    d->speedPlan->setChecked(global.speedPlanWeave);
}

void ExpertProcess::readVisibility()
{
    Visibility& vis = d->lib.visibility;
    vis.grooveDetail = d->visGroove->isChecked();
    vis.poseExtra = d->visPose->isChecked();
    vis.offsets = d->visOffset->isChecked();
    vis.intermittent = d->visInter->isChecked();
    vis.motion = d->visMotion->isChecked();
    vis.crater = d->visCrater->isChecked();
    vis.rearc = d->visRearc->isChecked();
    vis.corner = d->visCorner->isChecked();
    vis.adaptive = d->visAdaptive->isChecked();
    vis.globalPage = d->visGlobal->isChecked();
    vis.matchTable = d->visMatch->isChecked();
}

void ExpertProcess::writeVisibility()
{
    const Visibility& vis = d->lib.visibility;
    d->visGroove->setChecked(vis.grooveDetail);
    d->visPose->setChecked(vis.poseExtra);
    d->visOffset->setChecked(vis.offsets);
    d->visInter->setChecked(vis.intermittent);
    d->visMotion->setChecked(vis.motion);
    d->visCrater->setChecked(vis.crater);
    d->visRearc->setChecked(vis.rearc);
    d->visCorner->setChecked(vis.corner);
    d->visAdaptive->setChecked(vis.adaptive);
    d->visGlobal->setChecked(vis.globalPage);
    d->visMatch->setChecked(vis.matchTable);
}

void ExpertProcess::readMatch()
{
    QVector<MatchRule> rules;
    for (int row = 0; row < d->matchTable->rowCount(); ++row) {
        MatchRule rule;
        if (row < d->lib.rules.size())
            rule.wrap = d->lib.rules.at(row).wrap;
        QComboBox* position = qobject_cast<QComboBox*>(d->matchTable->cellWidget(row, 0));
        QDoubleSpinBox* thickMin = qobject_cast<QDoubleSpinBox*>(d->matchTable->cellWidget(row, 1));
        QDoubleSpinBox* thickMax = qobject_cast<QDoubleSpinBox*>(d->matchTable->cellWidget(row, 2));
        QDoubleSpinBox* tiltMin = qobject_cast<QDoubleSpinBox*>(d->matchTable->cellWidget(row, 3));
        QDoubleSpinBox* tiltMax = qobject_cast<QDoubleSpinBox*>(d->matchTable->cellWidget(row, 4));
        QComboBox* layer = qobject_cast<QComboBox*>(d->matchTable->cellWidget(row, 5));
        QDoubleSpinBox* start = qobject_cast<QDoubleSpinBox*>(d->matchTable->cellWidget(row, 6));
        QDoubleSpinBox* end = qobject_cast<QDoubleSpinBox*>(d->matchTable->cellWidget(row, 7));
        QCheckBox* continuous = qobject_cast<QCheckBox*>(d->matchTable->cellWidget(row, 8));
        if (!position || !thickMin || !thickMax || !tiltMin || !tiltMax || !layer || !start || !end || !continuous)
            continue;
        rule.weldPosition = comboData(position);
        rule.thickMin = thickMin->value();
        rule.thickMax = thickMax->value();
        rule.tiltMin = tiltMin->value();
        rule.tiltMax = tiltMax->value();
        rule.layerName = layer->currentText();
        rule.startDistance = start->value();
        rule.endDistance = end->value();
        rule.continuous = continuous->isChecked();
        rules.append(rule);
    }
    d->lib.rules = rules;
}

void ExpertProcess::rebuildMatch()
{
    QSignalBlocker blocker(d->matchTable);
    d->matchTable->setRowCount(0);
    const QStringList names = layerNames();
    for (int row = 0; row < d->lib.rules.size(); ++row) {
        const MatchRule& rule = d->lib.rules.at(row);
        d->matchTable->insertRow(row);
        QComboBox* position = new QComboBox;
        fillPosition(position);
        setComboData(position, rule.weldPosition);
        QDoubleSpinBox* thickMin = dspin(0, 100, 0.5, 1, QString());
        QDoubleSpinBox* thickMax = dspin(0, 100, 0.5, 1, QString());
        QDoubleSpinBox* tiltMin = dspin(-180, 180, 1, 1, QString());
        QDoubleSpinBox* tiltMax = dspin(-180, 180, 1, 1, QString());
        thickMin->setValue(rule.thickMin);
        thickMax->setValue(rule.thickMax);
        tiltMin->setValue(rule.tiltMin);
        tiltMax->setValue(rule.tiltMax);
        QComboBox* layer = new QComboBox;
        layer->addItems(names);
        setComboText(layer, rule.layerName);
        QDoubleSpinBox* start = dspin(0, 200, 1, 1, QString());
        QDoubleSpinBox* end = dspin(0, 200, 1, 1, QString());
        start->setValue(rule.startDistance);
        end->setValue(rule.endDistance);
        QCheckBox* continuous = new QCheckBox;
        continuous->setChecked(rule.continuous);
        QPushButton* wrapBtn = new QPushButton(QStringLiteral("包角设置"));
        wrapBtn->setEnabled(rule.continuous);
        connect(continuous, &QCheckBox::toggled, wrapBtn, &QPushButton::setEnabled);
        connect(wrapBtn, &QPushButton::clicked, this, [this, row]() {
            if (row < 0 || row >= d->matchTable->rowCount())
                return;
            readMatch();
            if (row >= d->lib.rules.size())
                return;
            editWrap(this, d->lib.rules[row].wrap, layerNames());
        });
        d->matchTable->setCellWidget(row, 0, position);
        d->matchTable->setCellWidget(row, 1, thickMin);
        d->matchTable->setCellWidget(row, 2, thickMax);
        d->matchTable->setCellWidget(row, 3, tiltMin);
        d->matchTable->setCellWidget(row, 4, tiltMax);
        d->matchTable->setCellWidget(row, 5, layer);
        d->matchTable->setCellWidget(row, 6, start);
        d->matchTable->setCellWidget(row, 7, end);
        d->matchTable->setCellWidget(row, 8, continuous);
        d->matchTable->setCellWidget(row, 9, wrapBtn);
        d->matchTable->setRowHeight(row, 32);
    }
}

void ExpertProcess::refreshLayerCombos()
{
    const QStringList names = layerNames();
    for (int row = 0; row < d->matchTable->rowCount(); ++row) {
        QComboBox* layer = qobject_cast<QComboBox*>(d->matchTable->cellWidget(row, 5));
        if (!layer)
            continue;
        const QString current = (row < d->lib.rules.size())
            ? d->lib.rules.at(row).layerName
            : layer->currentText();
        QSignalBlocker blocker(layer);
        layer->clear();
        layer->addItems(names);
        setComboText(layer, current);
    }
}

void ExpertProcess::applyPermission()
{
    const bool debugLayer = currentIsDebug();
    const bool full = d->expert || debugLayer;
    d->grooveDetail->setVisible(full || d->visGroove->isChecked());
    d->poseExtra->setVisible(full || d->visPose->isChecked());
    d->offsetGroup->setVisible(full || d->visOffset->isChecked());
    d->interGroup->setVisible(full || d->visInter->isChecked());
    d->motionGroup->setVisible(full || d->visMotion->isChecked());
    d->visGroove->setVisible(d->expert);
    d->visPose->setVisible(d->expert);
    d->visOffset->setVisible(d->expert);
    d->visInter->setVisible(d->expert);
    d->visMotion->setVisible(d->expert);
    d->visCrater->setVisible(d->expert);
    d->visRearc->setVisible(d->expert);
    d->visCorner->setVisible(d->expert);
    d->visAdaptive->setVisible(d->expert);
    d->visGlobal->setVisible(d->expert);
    d->visMatch->setVisible(d->expert);
    const struct { int index; bool shown; } gates[] = {
        {d->tabCrater, full || d->visCrater->isChecked()},
        {d->tabRearc, full || d->visRearc->isChecked()},
        {d->tabCorner, full || d->visCorner->isChecked()},
        {d->tabAdaptive, full || d->visAdaptive->isChecked()},
        {d->tabGlobal, full || d->visGlobal->isChecked()},
        {d->tabMatch, full || d->visMatch->isChecked()}
    };
    for (unsigned i = 0; i < sizeof(gates) / sizeof(gates[0]); ++i) {
        setTabShown(d->tabs, gates[i].index, gates[i].shown);
        if (!gates[i].shown && d->tabs->currentIndex() == gates[i].index)
            d->tabs->setCurrentIndex(0);
    }

    const bool structure = d->expert || debugLayer;
    d->btnAddPass->setEnabled(structure);
    d->btnDelPass->setEnabled(structure && d->passCount->value() > 1);
    d->passCount->setEnabled(structure);
    d->btnAddLayer->setEnabled(d->expert);
    d->btnDelLayer->setEnabled(d->expert && !debugLayer && nonDebugCount() > 1);
    d->btnImport->setEnabled(d->expert);
    d->btnExport->setEnabled(d->expert);
    d->btnPassword->setEnabled(d->expert);
    d->keepCheck->setEnabled(d->expert);
    d->nameEdit->setReadOnly(!d->expert || debugLayer);
    d->btnRecommend->setEnabled(full || d->visOffset->isChecked());
    refreshStatus();
}

void ExpertProcess::updateGrooveWidgets()
{
    const int groove = comboData(d->groove);
    const bool anyAngle = groove == GrooveSingleV || groove == GrooveBoxV || groove == GrooveAsymmetricV;
    const bool second = groove == GrooveAsymmetricV;
    const bool gap = groove != GrooveNone;
    d->angle->setEnabled(anyAngle);
    d->angle2->setEnabled(second);
    d->gap->setEnabled(gap);
    d->rootFace->setEnabled(groove == GrooveI || anyAngle);
}

void ExpertProcess::updateWeaveWidgets()
{
    const bool on = comboData(d->weave) != WeaveNone;
    d->weaveAmp->setEnabled(on);
    d->weaveFreq->setEnabled(on);
    d->dwellLeft->setEnabled(on);
    d->dwellRight->setEnabled(on);
    d->dwellCenter->setEnabled(on);
    d->weaveAngle->setEnabled(on && (comboData(d->weave) == WeaveL || comboData(d->weave) == WeaveTriangle));
}

void ExpertProcess::updateAdaptiveHint()
{
    if (!d->adaptiveHint)
        return;
    if (comboData(d->groove) == GrooveBoxV) {
        d->adaptiveHint->setText(QStringLiteral(
            "当前坡口是箱型柱主焊缝单 V。朝上的变坡口可以按视觉实测宽度焊接，"
            "用来补偿拼装造成的宽窄不一。"));
    } else {
        d->adaptiveHint->setText(QStringLiteral(
            "坡口自适应只对朝上的箱型柱主焊缝单 V 生效。其他坡口可以先把参数存进工艺库，匹配时不会按变坡口扫描。"));
    }
}

void ExpertProcess::onLayerSelected()
{
    if (d->loading)
        return;
    QTreeWidgetItem* item = d->tree->currentItem();
    if (!item)
        return;
    const int index = item->data(0, Qt::UserRole).toInt();
    if (index < 0) {
        if (item->childCount() > 0)
            d->tree->setCurrentItem(item->child(0));
        return;
    }
    if (index == d->layerIndex)
        return;
    commitPass();
    d->layerIndex = index;
    d->passIndex = 0;
    rebuildPasses(0);
    loadLayerFields();
    loadPass();
    updateGrooveWidgets();
    updateAdaptiveHint();
    applyPermission();
}

void ExpertProcess::onPassSelected(int row)
{
    if (d->loading || row < 0 || row == d->passIndex)
        return;
    commitPass();
    d->passIndex = row;
    loadPass();
    refreshStatus();
}

void ExpertProcess::onPassCountChanged(int count)
{
    if (d->loading)
        return;
    if (!commitPass())
        return;
    LayerParams& layer = d->lib.layers[d->layerIndex];
    PassParams proto = layer.passes.isEmpty() ? PassParams() : layer.passes.last();
    while (layer.passes.size() < count) {
        PassParams pass = proto;
        pass.offsetY = 0;
        pass.offsetZ = 0;
        pass.startOffset = 0;
        layer.passes.append(pass);
    }
    while (layer.passes.size() > count)
        layer.passes.removeLast();
    renamePasses(layer.passes);
    if (d->passIndex >= layer.passes.size())
        d->passIndex = layer.passes.size() - 1;
    rebuildPasses(d->passIndex);
    loadPass();
    applyPermission();
}

void ExpertProcess::onExpertToggled(bool on)
{
    if (d->loading)
        return;
    if (on) {
        bool ok = false;
        const QString password = QInputDialog::getText(
            this, QStringLiteral("专家模式"), QStringLiteral("请输入专家模式密码："),
            QLineEdit::Password, QString(), &ok);
        if (!ok || password != d->password) {
            d->loading = true;
            d->expertCheck->setChecked(false);
            d->loading = false;
            if (ok && m_console)
                printW(QStringLiteral("专家模式密码错误。"));
            return;
        }
        d->expert = true;
        if (m_console)
            printI(QStringLiteral("已进入专家模式，可以编辑全部工艺参数，并设置非专家可见项。"));
    } else {
        d->expert = false;
        d->lib.keepExpert = false;
        d->committed.keepExpert = false;
        d->loading = true;
        d->keepCheck->setChecked(false);
        d->loading = false;
        if (m_console)
            printI(QStringLiteral("已退出专家模式。"));
    }
    applyPermission();
}

void ExpertProcess::onRenameFinished()
{
    if (d->loading || d->layerIndex < 0)
        return;
    readMatch();
    const QString name = d->nameEdit->text().trimmed();
    LayerParams& layer = d->lib.layers[d->layerIndex];
    if (name.isEmpty() || name == layer.name) {
        d->nameEdit->setText(layer.name);
        return;
    }
    for (int i = 0; i < d->lib.layers.size(); ++i) {
        if (i != d->layerIndex && d->lib.layers.at(i).name == name) {
            if (m_console)
                printW(QStringLiteral("工艺名称已存在。"));
            d->nameEdit->setText(layer.name);
            return;
        }
    }
    const QString previous = layer.name;
    layer.name = name;
    for (int i = 0; i < d->lib.rules.size(); ++i) {
        MatchRule& rule = d->lib.rules[i];
        if (rule.layerName == previous)
            rule.layerName = name;
        if (rule.wrap.upFront.layerName == previous)
            rule.wrap.upFront.layerName = name;
        if (rule.wrap.upBack.layerName == previous)
            rule.wrap.upBack.layerName = name;
        if (rule.wrap.downFront.layerName == previous)
            rule.wrap.downFront.layerName = name;
        if (rule.wrap.downBack.layerName == previous)
            rule.wrap.downBack.layerName = name;
    }
    rebuildTree(d->layerIndex);
    refreshLayerCombos();
    refreshStatus();
}

void ExpertProcess::changePassword()
{
    if (!d->expert)
        return;
    bool ok = false;
    const QString again = QInputDialog::getText(
        this, QStringLiteral("修改密码"), QStringLiteral("新的专家模式密码："),
        QLineEdit::Password, QString(), &ok);
    if (!ok)
        return;
    if (again.trimmed().isEmpty()) {
        if (m_console)
            printW(QStringLiteral("密码不能为空。"));
        return;
    }
    bool confirmOk = false;
    const QString confirm = QInputDialog::getText(
        this, QStringLiteral("修改密码"), QStringLiteral("再次输入密码："),
        QLineEdit::Password, QString(), &confirmOk);
    if (!confirmOk)
        return;
    if (again != confirm) {
        if (m_console)
            printW(QStringLiteral("两次输入的密码不一致。"));
        return;
    }
    d->password = again;
    if (m_console)
        printI(QStringLiteral("专家模式密码已更新。密码只保存在本机对话框中，不写入工艺包。"));
}

void ExpertProcess::showHelp()
{
    QMessageBox::information(
        this,
        QStringLiteral("工艺库说明"),
        QStringLiteral(
            "专家模式初始密码是 123456。调试工艺不能删除，并且在操作员模式下也全部开放。\n\n"
            "除调试工艺外至少保留一个图层。导入导出和增删图层需要专家模式。\n"
            "每个图层可以有多条焊道，根焊道和子焊道分别保存电流、电压、摆动和偏移。\n"
            "箱型柱主焊缝可使用“推荐偏移”生成 Y/Z。50mm 以下单 V 作为全熔透工艺维护。\n\n"
            "弧坑回填会沿主焊缝回焊后再走常规收弧。再起弧先于刮擦起弧，两者都只用于走图纸。\n"
            "拐角工艺由拐角前/后距离决定生效范围，最大转速建议 5–20 RPM。\n"
            "摆动支持正弦摆、钟摆、L 型摆和三角摆，摆幅可以收到 1mm，并配合电弧跟踪。\n\n"
            "工艺匹配表按焊接位置、板厚和倾角选择图层。连续焊可分别设置立向上包角和包角立向下前后段的工艺与最佳推拉角。"));
    if (m_console)
        printI(QStringLiteral("已打开工艺库说明。现场排故仍以机床侧教程为准。"));
}

void ExpertProcess::addLayer()
{
    if (!d->expert) {
        if (m_console)
            printW(QStringLiteral("新增工艺需要专家模式。"));
        return;
    }
    commitPass();
    QDialog dialog(this);
    dialog.setWindowTitle(QStringLiteral("新增工艺图层"));
    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QComboBox* category = new QComboBox;
    const QStringList cats = categoryNames();
    for (int i = 0; i < cats.size(); ++i) {
        if (cats.at(i) != QStringLiteral("调试工艺"))
            category->addItem(cats.at(i));
    }
    if (d->layerIndex >= 0 && !currentIsDebug())
        category->setCurrentText(d->lib.layers.at(d->layerIndex).category);
    QLineEdit* name = new QLineEdit(QStringLiteral("新工艺"));
    layout->addWidget(new QLabel(QStringLiteral("分类")));
    layout->addWidget(category);
    layout->addWidget(new QLabel(QStringLiteral("名称")));
    layout->addWidget(name);
    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    layout->addWidget(buttons);
    if (dialog.exec() != QDialog::Accepted)
        return;
    const QString layerName = name->text().trimmed();
    if (layerName.isEmpty()) {
        if (m_console)
            printW(QStringLiteral("工艺名称不能为空。"));
        return;
    }
    if (layerNames().contains(layerName)) {
        if (m_console)
            printW(QStringLiteral("工艺名称已存在。"));
        return;
    }
    static const char* colors[] = {"#2563EB", "#059669", "#D97706", "#7C3AED", "#DC2626", "#0891B2", "#4F46E5"};
    LayerParams layer = makeLayer(layerName, category->currentText(),
                                  QString::fromLatin1(colors[d->lib.layers.size() % 7]), false);
    layer.plateThickness = 6;
    layer.passes.append(basePass(PosFillet));
    if (layer.category == QStringLiteral("平焊") || layer.category == QStringLiteral("多层平焊")) {
        layer.groove = layer.category == QStringLiteral("多层平焊") ? GrooveSingleV : GrooveI;
        layer.grooveAngle = layer.groove == GrooveSingleV ? 60 : 0;
        layer.plateThickness = layer.groove == GrooveSingleV ? 20 : 8;
        layer.passes[0].weldPosition = layer.groove == GrooveSingleV ? PosMultiFlat : PosFlat;
        layer.passes[0].pitchAngle = 0;
    } else if (layer.category == QStringLiteral("立向上焊")) {
        layer.passes[0].weldPosition = PosVerticalUp;
        layer.passes[0].pitchAngle = 45;
    } else if (layer.category == QStringLiteral("立向下焊")) {
        layer.passes[0].weldPosition = PosVerticalDown;
    } else if (layer.category == QStringLiteral("多层平角焊")) {
        layer.passes[0].weldPosition = PosMultiFillet;
        layer.passes[0].pitchAngle = 45;
    } else if (layer.category == QStringLiteral("多层立焊")) {
        layer.passes[0].weldPosition = PosMultiVertical;
    } else {
        layer.passes[0].pitchAngle = 45;
    }
    renamePasses(layer.passes);
    d->lib.layers.append(layer);
    d->layerIndex = d->lib.layers.size() - 1;
    d->passIndex = 0;
    reloadUi();
    if (m_console)
        printI(QStringLiteral("已添加工艺图层 %1。").arg(layerName));
}

void ExpertProcess::removeLayer()
{
    if (!d->expert) {
        if (m_console)
            printW(QStringLiteral("删除工艺需要专家模式。"));
        return;
    }
    if (d->layerIndex < 0 || d->layerIndex >= d->lib.layers.size())
        return;
    if (currentIsDebug()) {
        if (m_console)
            printW(QStringLiteral("调试工艺不可删除。"));
        return;
    }
    if (nonDebugCount() <= 1) {
        if (m_console)
            printW(QStringLiteral("至少保留一个图层工艺。"));
        return;
    }
    const QString name = d->lib.layers.at(d->layerIndex).name;
    if (QMessageBox::question(this, QStringLiteral("删除工艺"),
                              QStringLiteral("删除工艺图层“%1”？").arg(name))
        != QMessageBox::Yes)
        return;
    d->lib.layers.removeAt(d->layerIndex);
    d->layerIndex = 0;
    d->passIndex = 0;
    reloadUi();
    if (m_console)
        printW(QStringLiteral("已删除工艺图层 %1。匹配表里仍引用该名称的规则需要另行修改。").arg(name));
}

void ExpertProcess::addPass()
{
    if (!d->expert && !currentIsDebug()) {
        if (m_console)
            printW(QStringLiteral("增加焊道需要专家模式。调试工艺除外。"));
        return;
    }
    d->passCount->setValue(d->passCount->value() + 1);
}

void ExpertProcess::removePass()
{
    if (!d->expert && !currentIsDebug()) {
        if (m_console)
            printW(QStringLiteral("删除焊道需要专家模式。调试工艺除外。"));
        return;
    }
    if (!commitPass())
        return;
    LayerParams& layer = d->lib.layers[d->layerIndex];
    if (layer.passes.size() <= 1) {
        if (m_console)
            printW(QStringLiteral("至少保留一条焊道。"));
        return;
    }
    int removeAt = d->passIndex;
    if (removeAt < 0 || removeAt >= layer.passes.size())
        removeAt = layer.passes.size() - 1;
    layer.passes.removeAt(removeAt);
    renamePasses(layer.passes);
    if (d->passIndex >= layer.passes.size())
        d->passIndex = layer.passes.size() - 1;
    d->loading = true;
    d->passCount->setValue(layer.passes.size());
    d->loading = false;
    rebuildPasses(d->passIndex);
    loadPass();
    applyPermission();
}

void ExpertProcess::apply()
{
    if (!commitPass()) {
        if (m_console)
            printW(QStringLiteral("没有可应用的工艺图层。"));
        return;
    }
    readGlobal();
    readVisibility();
    readMatch();
    const LayerParams& layer = d->lib.layers.at(d->layerIndex);
    const PassParams& pass = layer.passes.at(d->passIndex);
    if (pass.cornerEnable && (pass.cornerRpm < 5.0 || pass.cornerRpm > 20.0) && m_console)
        printW(QStringLiteral("拐角最大转速建议在 5–20 RPM。"));
    if (pass.rearcEnable && pass.rearcTime != pass.arcTimeout && m_console)
        printW(QStringLiteral("再起弧时间建议与起弧超时保持一致。"));
    if (pass.weave != WeaveNone && pass.weaveFreq <= 0.0 && m_console)
        printW(QStringLiteral("已选择摆动，但频率为 0。"));
    if (layer.plateThickness > 50.0 && m_console)
        printW(QStringLiteral("板厚超过 50mm，超出本工艺库声明的全熔透范围。"));
    for (int i = 0; i < d->lib.rules.size(); ++i) {
        if (!layerNames().contains(d->lib.rules.at(i).layerName) && m_console)
            printW(QStringLiteral("匹配规则 %1 指向了不存在的工艺图层。").arg(i + 1));
    }
    d->committed = d->lib;
    if (m_console)
        printI(QStringLiteral("已应用工艺 %1 / %2。").arg(layer.name, pass.name));
    refreshStatus();
}

void ExpertProcess::recommendOffsets()
{
    if (!commitPass())
        return;
    if (!(d->expert || currentIsDebug() || d->visOffset->isChecked())) {
        if (m_console)
            printW(QStringLiteral("推荐偏移需要专家模式，或将该组参数设为操作员可见。"));
        return;
    }
    LayerParams& layer = d->lib.layers[d->layerIndex];
    const QString warn = fillRecommendedOffsets(layer);
    loadPass();
    if (!warn.isEmpty() && m_console)
        printW(warn);
    if (m_console)
        printI(QStringLiteral("已按坡口截面写入 %1 条焊道的 Y/Z 偏移。").arg(layer.passes.size()));
}

void ExpertProcess::importPackage()
{
    importFromFile(QString(), QStringLiteral("package"));
}

void ExpertProcess::exportPackage()
{
    exportToFile(QStringLiteral("package"));
}

void ExpertProcess::importFromFile(const QString& path, const QString& forcedType)
{
    if (!d->expert) {
        if (m_console)
            printW(QStringLiteral("导入工艺需要专家模式。"));
        return;
    }
    QString filePath = path;
    if (filePath.isEmpty()) {
        filePath = QFileDialog::getOpenFileName(
            this, QStringLiteral("导入工艺"), QString(), QStringLiteral("工艺包 (*.json)"));
    }
    if (filePath.isEmpty())
        return;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (m_console)
            printE(QStringLiteral("无法读取工艺文件。"));
        return;
    }
    QJsonParseError error;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError || !doc.isObject()) {
        if (m_console)
            printE(QStringLiteral("工艺文件不是有效的 JSON。"));
        return;
    }
    const QJsonObject root = doc.object();
    QString type = js(root, "type", forcedType);
    if (!forcedType.isEmpty() && !type.isEmpty() && type != forcedType) {
        if (m_console)
            printW(QStringLiteral("文件类型是 %1，与当前导入命令不一致。").arg(type));
        return;
    }
    commitPass();
    if (type == QStringLiteral("pass")) {
        if (d->layerIndex < 0 || d->passIndex < 0) {
            if (m_console)
                printW(QStringLiteral("请先选择焊道。"));
            return;
        }
        PassParams pass = passFromJson(root.value(QStringLiteral("pass")).toObject());
        pass.name = passTitle(d->passIndex);
        d->lib.layers[d->layerIndex].passes[d->passIndex] = pass;
    } else if (type == QStringLiteral("layer")) {
        if (currentIsDebug()) {
            if (m_console)
                printW(QStringLiteral("调试工艺不接受焊缝工艺覆盖。"));
            return;
        }
        LayerParams layer = layerFromJson(root.value(QStringLiteral("layer")).toObject());
        if (layer.debug) {
            if (m_console)
                printW(QStringLiteral("不能把调试工艺导入到普通图层。"));
            return;
        }
        layer.category = d->lib.layers.at(d->layerIndex).category;
        layer.color = d->lib.layers.at(d->layerIndex).color;
        layer.debug = false;
        const QString previous = d->lib.layers.at(d->layerIndex).name;
        if (layer.name != previous && layerNames().contains(layer.name))
            layer.name = previous;
        d->lib.layers[d->layerIndex] = layer;
        d->passIndex = 0;
    } else {
        Library incoming;
        incoming.keepExpert = jb(root, "keepExpert", false);
        incoming.global = globalFromJson(root.value(QStringLiteral("global")).toObject());
        incoming.visibility = visibilityFromJson(root.value(QStringLiteral("visibility")).toObject());
        const QJsonArray layers = root.value(QStringLiteral("layers")).toArray();
        for (int i = 0; i < layers.size(); ++i)
            incoming.layers.append(layerFromJson(layers.at(i).toObject()));
        const QJsonArray rules = root.value(QStringLiteral("rules")).toArray();
        for (int i = 0; i < rules.size(); ++i)
            incoming.rules.append(ruleFromJson(rules.at(i).toObject()));
        if (incoming.layers.isEmpty()) {
            if (m_console)
                printE(QStringLiteral("工艺包中没有图层。"));
            return;
        }
        bool hasDebug = false;
        for (int i = 0; i < incoming.layers.size(); ++i) {
            if (incoming.layers.at(i).debug)
                hasDebug = true;
        }
        if (!hasDebug) {
            if (m_console)
                printW(QStringLiteral("工艺包缺少调试工艺，已保留当前调试工艺。"));
            for (int i = 0; i < d->lib.layers.size(); ++i) {
                if (d->lib.layers.at(i).debug) {
                    incoming.layers.prepend(d->lib.layers.at(i));
                    break;
                }
            }
        }
        d->lib = incoming;
        d->layerIndex = 0;
        d->passIndex = 0;
    }
    d->committed = d->lib;
    reloadUi();
    if (m_console)
        printI(QStringLiteral("工艺已从文件导入。"));
}

void ExpertProcess::exportToFile(const QString& type)
{
    if (!d->expert) {
        if (m_console)
            printW(QStringLiteral("导出工艺需要专家模式。"));
        return;
    }
    if (!commitPass())
        return;
    readGlobal();
    readVisibility();
    readMatch();
    const QString path = QFileDialog::getSaveFileName(
        this, QStringLiteral("导出工艺"), QStringLiteral("process-library.json"),
        QStringLiteral("工艺包 (*.json)"));
    if (path.isEmpty())
        return;
    QJsonObject root;
    put(root, "version", 1);
    put(root, "type", type);
    if (type == QStringLiteral("pass")) {
        root.insert(QStringLiteral("pass"), passToJson(d->lib.layers.at(d->layerIndex).passes.at(d->passIndex)));
    } else if (type == QStringLiteral("layer")) {
        root.insert(QStringLiteral("layer"), layerToJson(d->lib.layers.at(d->layerIndex)));
    } else {
        put(root, "keepExpert", d->lib.keepExpert);
        root.insert(QStringLiteral("global"), globalToJson(d->lib.global));
        root.insert(QStringLiteral("visibility"), visibilityToJson(d->lib.visibility));
        QJsonArray layers;
        for (int i = 0; i < d->lib.layers.size(); ++i)
            layers.append(layerToJson(d->lib.layers.at(i)));
        QJsonArray rules;
        for (int i = 0; i < d->lib.rules.size(); ++i)
            rules.append(ruleToJson(d->lib.rules.at(i)));
        root.insert(QStringLiteral("layers"), layers);
        root.insert(QStringLiteral("rules"), rules);
    }
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (m_console)
            printE(QStringLiteral("无法写入工艺文件。"));
        return;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (m_console)
        printI(QStringLiteral("工艺已导出。专家密码不会写入文件。"));
}
