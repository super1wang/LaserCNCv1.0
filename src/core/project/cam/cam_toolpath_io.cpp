#include "core/project/cam/cam_toolpath_io.h"

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/logging/logger.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_contracts.h"

#include <QColor>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QList>
#include <QSet>
#include <QVector>

#include <toml.hpp>

#include <algorithm>
#include <fstream>
#include <vector>

namespace lcnc::cam {

namespace {

constexpr char kCamToolpathTomlFile[]   = "cam_toolpath.toml";
constexpr char kCamToolpathPointsFile[] = "cam_toolpath_points.bin";
constexpr int  kCamToolpathSchemaVersion = 1;

// 二进制点集 magic 头（"LCNCTPT1"）。
constexpr quint64 kPointsBinMagic = 0x315450434E434C00ull;
constexpr quint32 kPointsBinVersion = 1;

// v1 process_cutting_plan.toml 字段（仅用于一次性迁移）。
constexpr char kLegacyPlanFile[]        = "process_cutting_plan.toml";
constexpr char kFieldLayerId[]          = "layerId";
constexpr char kFieldToolName[]         = "toolName";
constexpr char kFieldEnabled[]          = "enabled";
constexpr char kFieldCompensation[]     = "compensationIndex";
constexpr char kFieldIncludedContours[] = "includedContours";
constexpr char kFieldManualOrder[]      = "manualContourOrder";
constexpr char kFieldLastAxis[]         = "lastAutoSortAxis";

QString camToolpathTomlPath(const QString& packageDir)
{
    return QDir(packageDir).filePath(QString::fromLatin1(kCamToolpathTomlFile));
}

QString camToolpathPointsPath(const QString& packageDir)
{
    return QDir(packageDir).filePath(QString::fromLatin1(kCamToolpathPointsFile));
}

const char* sortStrategyToString(CuttingPlanSortStrategy s)
{
    switch (s) {
    case CuttingPlanSortStrategy::CamOrder:         return "CamOrder";
    case CuttingPlanSortStrategy::LayerThenContour: return "LayerThenContour";
    case CuttingPlanSortStrategy::ToolThenLayer:    return "ToolThenLayer";
    case CuttingPlanSortStrategy::Manual:           return "Manual";
    }
    return "LayerThenContour";
}

CuttingPlanSortStrategy sortStrategyFromString(const QString& s, CuttingPlanSortStrategy def)
{
    if (s == "CamOrder")         return CuttingPlanSortStrategy::CamOrder;
    if (s == "LayerThenContour") return CuttingPlanSortStrategy::LayerThenContour;
    if (s == "ToolThenLayer")    return CuttingPlanSortStrategy::ToolThenLayer;
    if (s == "Manual")           return CuttingPlanSortStrategy::Manual;
    return def;
}

const char* autoSortAxisToString(AutoSortAxis a)
{
    switch (a) {
    case AutoSortAxis::XPos: return "X+";
    case AutoSortAxis::XNeg: return "X-";
    case AutoSortAxis::YPos: return "Y+";
    case AutoSortAxis::YNeg: return "Y-";
    case AutoSortAxis::ZPos: return "Z+";
    case AutoSortAxis::ZNeg: return "Z-";
    }
    return "X+";
}

AutoSortAxis autoSortAxisFromString(const QString& s, AutoSortAxis def)
{
    if (s == "X+") return AutoSortAxis::XPos;
    if (s == "X-") return AutoSortAxis::XNeg;
    if (s == "Y+") return AutoSortAxis::YPos;
    if (s == "Y-") return AutoSortAxis::YNeg;
    if (s == "Z+") return AutoSortAxis::ZPos;
    if (s == "Z-") return AutoSortAxis::ZNeg;
    return def;
}

// ---- 二进制点集 IO ----------------------------------------------------------

bool writePointsBin(const QString& filePath,
                    const std::vector<LaserContour>& contours,
                    QString* errorMsg)
{
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errorMsg) *errorMsg = QStringLiteral("无法写入: %1").arg(filePath);
        return false;
    }
    QDataStream ds(&f);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::DoublePrecision);
    ds.setVersion(QDataStream::Qt_6_5);

    ds << kPointsBinMagic << kPointsBinVersion;
    const quint32 contourCount = static_cast<quint32>(contours.size());
    ds << contourCount;

    for (const LaserContour& c : contours) {
        ds << static_cast<quint64>(c.contourId);
        ds << static_cast<quint32>(c.points.size());
        for (const ToolpathPoint& p : c.points) {
            ds << p.position.X() << p.position.Y() << p.position.Z();
            ds << p.normal.X()   << p.normal.Y()   << p.normal.Z();
            ds << p.tangent.X()  << p.tangent.Y()  << p.tangent.Z();
            ds << p.param;
            ds << p.machineCoord.x << p.machineCoord.y << p.machineCoord.z;
            ds << p.machineCoord.r1 << p.machineCoord.r2;
            ds << p.machineCoord.r1Name << p.machineCoord.r2Name;
            ds << static_cast<quint8>(p.machineCoord.valid ? 1 : 0);
        }
    }
    return ds.status() == QDataStream::Ok;
}

bool readPointsBin(const QString& filePath,
                   QHash<std::uint64_t, std::vector<ToolpathPoint>>& pointsByContourId,
                   QString* errorMsg)
{
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) {
        if (errorMsg) *errorMsg = QStringLiteral("无法读取: %1").arg(filePath);
        return false;
    }
    QDataStream ds(&f);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::DoublePrecision);
    ds.setVersion(QDataStream::Qt_6_5);

    quint64 magic = 0;
    quint32 version = 0, contourCount = 0;
    ds >> magic >> version >> contourCount;
    if (magic != kPointsBinMagic) {
        if (errorMsg) *errorMsg = QStringLiteral("点集文件 magic 不匹配");
        return false;
    }
    if (version != kPointsBinVersion) {
        if (errorMsg) *errorMsg = QStringLiteral("点集文件版本 %1 不支持").arg(version);
        return false;
    }

    for (quint32 i = 0; i < contourCount; ++i) {
        quint64 contourId = 0;
        quint32 pointCount = 0;
        ds >> contourId >> pointCount;
        std::vector<ToolpathPoint> pts;
        pts.reserve(pointCount);
        for (quint32 k = 0; k < pointCount; ++k) {
            double px, py, pz, nx, ny, nz, tx, ty, tz, par;
            double mx, my, mz, r1, r2;
            QString r1Name, r2Name;
            quint8 valid = 0;
            ds >> px >> py >> pz >> nx >> ny >> nz >> tx >> ty >> tz >> par;
            ds >> mx >> my >> mz >> r1 >> r2 >> r1Name >> r2Name >> valid;
            ToolpathPoint tp;
            tp.position = gp_Pnt(px, py, pz);
            tp.normal   = gp_Dir(nx, ny, nz);
            tp.tangent  = gp_Dir(tx, ty, tz);
            tp.param    = par;
            tp.machineCoord.x = mx;
            tp.machineCoord.y = my;
            tp.machineCoord.z = mz;
            tp.machineCoord.r1 = r1;
            tp.machineCoord.r2 = r2;
            tp.machineCoord.r1Name = r1Name;
            tp.machineCoord.r2Name = r2Name;
            tp.machineCoord.valid  = (valid != 0);
            pts.push_back(std::move(tp));
        }
        pointsByContourId.insert(static_cast<std::uint64_t>(contourId), std::move(pts));
    }
    return ds.status() == QDataStream::Ok;
}

} // namespace

bool hasCamToolpathCache(const QString& packageDir)
{
    return QFileInfo::exists(camToolpathTomlPath(packageDir))
        && QFileInfo::exists(camToolpathPointsPath(packageDir));
}

bool saveCamToolpath(const CamDataManager& cam, const QString& packageDir, QString* errorMsg)
{
    const LaserToolpath& tp = cam.toolpath();
    if (tp.contourCount() == 0) {
        // 无刀路 → 不写文件，让旧文件保持（或在 clearToolpath 时显式删除）。
        return true;
    }
    if (!QDir().mkpath(packageDir)) {
        if (errorMsg) *errorMsg = QStringLiteral("无法创建目录: %1").arg(packageDir);
        return false;
    }

    // 1. 写 toml 元数据
    toml::value root(toml::table{});
    root["schemaVersion"] = kCamToolpathSchemaVersion;

    std::uint64_t maxContour = 0, maxLayer = 0;
    for (const LaserContour& c : tp.contours())
        maxContour = std::max<std::uint64_t>(maxContour, c.contourId);
    for (const ToolpathLayer& l : tp.layers())
        maxLayer = std::max<std::uint64_t>(maxLayer, l.layerId);
    root["nextContourId"] = static_cast<std::int64_t>(maxContour + 1);
    root["nextLayerId"]   = static_cast<std::int64_t>(maxLayer + 1);

    // signature → id 映射
    toml::array sigContourArr;
    auto sigContourMap = cam.signatureToContourId();
    for (auto it = sigContourMap.constBegin(); it != sigContourMap.constEnd(); ++it) {
        toml::value e(toml::table{});
        e["sig"] = static_cast<std::int64_t>(it.key());
        e["id"]  = static_cast<std::int64_t>(it.value());
        sigContourArr.push_back(e);
    }
    root["signatureContours"] = sigContourArr;

    toml::array sigLayerArr;
    auto sigLayerMap = cam.signatureToLayerId();
    for (auto it = sigLayerMap.constBegin(); it != sigLayerMap.constEnd(); ++it) {
        toml::value e(toml::table{});
        e["sig"] = static_cast<std::int64_t>(it.key());
        e["id"]  = static_cast<std::int64_t>(it.value());
        sigLayerArr.push_back(e);
    }
    root["signatureLayers"] = sigLayerArr;

    toml::array layersArr;
    for (const ToolpathLayer& l : tp.layers()) {
        toml::value entry(toml::table{});
        entry["layerId"]   = static_cast<std::int64_t>(l.layerId);
        entry["signature"] = static_cast<std::int64_t>(l.signature);
        entry["name"]      = l.name.toStdString();
        entry["color"]     = l.color.name(QColor::HexRgb).toStdString();
        entry["enabled"]   = l.enabled;
        entry["toolName"]  = l.toolName.toStdString();
        entry["compensationIndex"] = l.compensationIndex.toStdString();
        toml::array contourIds;
        for (auto id : l.contourIds)
            contourIds.push_back(static_cast<std::int64_t>(id));
        entry["contourIds"] = contourIds;
        toml::array includedArr;
        QList<std::uint64_t> sortedIncluded(l.includedContours.cbegin(), l.includedContours.cend());
        std::sort(sortedIncluded.begin(), sortedIncluded.end());
        for (auto cid : sortedIncluded)
            includedArr.push_back(static_cast<std::int64_t>(cid));
        entry["includedContours"] = includedArr;
        layersArr.push_back(entry);
    }
    root["layers"] = layersArr;

    toml::array contoursArr;
    for (const LaserContour& c : tp.contours()) {
        toml::value entry(toml::table{});
        entry["contourId"]      = static_cast<std::int64_t>(c.contourId);
        entry["layerId"]        = static_cast<std::int64_t>(c.layerId);
        entry["signature"]      = static_cast<std::int64_t>(c.signature);
        entry["name"]           = c.name.toStdString();
        entry["enabled"]        = c.enabled;
        entry["workpieceEntry"] = c.workpieceEntry.toStdString();
        entry["sourceInfo"]     = c.sourceInfo.toStdString();
        entry["contourType"]    = static_cast<std::int64_t>(c.contourType);
        entry["leadInLength"]      = c.leadIn.length;
        entry["leadInNormalAngle"] = c.leadIn.normalAngle;
        entry["leadInValid"]       = c.leadIn.valid;
        if (c.leadIn.valid) {
            entry["leadInEntryX"]    = c.leadIn.entryPoint.X();
            entry["leadInEntryY"]    = c.leadIn.entryPoint.Y();
            entry["leadInEntryZ"]    = c.leadIn.entryPoint.Z();
            entry["leadInEntryParam"]= c.leadIn.entryParam;
            entry["leadInEntryEdge"] = static_cast<std::int64_t>(c.leadIn.entryEdgeIndex);
        }
        contoursArr.push_back(entry);
    }
    root["contours"] = contoursArr;

    // 容器级"切割链表"状态随 toolpath 一起持久化。
    const LayerContainer& container = cam.layerContainer();
    toml::array manualArr;
    for (auto cid : container.manualContourOrder())
        manualArr.push_back(static_cast<std::int64_t>(cid));
    root["manualContourOrder"] = manualArr;
    root["sortStrategy"]     = std::string(sortStrategyToString(container.sortStrategy()));
    root["lastAutoSortAxis"] = std::string(autoSortAxisToString(container.lastAutoSortAxis()));

    std::ofstream out(camToolpathTomlPath(packageDir).toStdString(), std::ios::binary);
    if (!out.is_open()) {
        if (errorMsg) *errorMsg = QStringLiteral("无法写入 cam_toolpath.toml");
        return false;
    }
    out << toml::format(root);
    out.close();

    // 2. 写二进制点集
    if (!writePointsBin(camToolpathPointsPath(packageDir), tp.contours(), errorMsg))
        return false;

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: saved {} layers, {} contours",
              tp.layers().size(),
              tp.contourCount());
    return true;
}

bool loadCamToolpath(CamDataManager& cam, const QString& packageDir, QString* errorMsg)
{
    const QString tomlPath  = camToolpathTomlPath(packageDir);
    const QString pointsPath = camToolpathPointsPath(packageDir);
    if (!QFileInfo::exists(tomlPath) || !QFileInfo::exists(pointsPath)) {
        if (errorMsg) *errorMsg = QStringLiteral("项目无刀路缓存");
        return false;
    }

    // 1. 解析 toml
    toml::value root;
    try {
        root = toml::parse(tomlPath.toStdString());
    } catch (const std::exception& e) {
        if (errorMsg) *errorMsg = QStringLiteral("解析 cam_toolpath.toml 失败: %1")
                                       .arg(QString::fromLocal8Bit(e.what()));
        return false;
    }
    if (!root.is_table()) {
        if (errorMsg) *errorMsg = QStringLiteral("cam_toolpath.toml 根节点错误");
        return false;
    }

    // 2. 读点集（先于 LaserContour，确保按 contourId 关联）。
    QHash<std::uint64_t, std::vector<ToolpathPoint>> pointsByContourId;
    if (!readPointsBin(pointsPath, pointsByContourId, errorMsg))
        return false;

    // 3. 装配 LaserToolpath
    LaserToolpath toolpath;

    if (root.contains("layers") && root.at("layers").is_array()) {
        for (const toml::value& e : root.at("layers").as_array()) {
            if (!e.is_table()) continue;
            ToolpathLayer l;
            if (e.contains("layerId"))   l.layerId   = static_cast<std::uint64_t>(e.at("layerId").as_integer());
            if (e.contains("signature")) l.signature = static_cast<std::uint64_t>(e.at("signature").as_integer());
            if (e.contains("name"))      l.name      = QString::fromStdString(e.at("name").as_string());
            if (e.contains("color"))     l.color     = QColor(QString::fromStdString(e.at("color").as_string()));
            if (e.contains("enabled"))   l.enabled   = e.at("enabled").as_boolean();
            if (e.contains("toolName"))  l.toolName  = QString::fromStdString(e.at("toolName").as_string());
            if (e.contains("compensationIndex"))
                l.compensationIndex = QString::fromStdString(e.at("compensationIndex").as_string());
            if (e.contains("contourIds") && e.at("contourIds").is_array()) {
                for (const toml::value& id : e.at("contourIds").as_array())
                    if (id.is_integer())
                        l.contourIds.push_back(static_cast<std::uint64_t>(id.as_integer()));
            }
            if (e.contains("includedContours") && e.at("includedContours").is_array()) {
                for (const toml::value& id : e.at("includedContours").as_array())
                    if (id.is_integer())
                        l.includedContours.insert(static_cast<std::uint64_t>(id.as_integer()));
            }
            toolpath.layers().push_back(l);
        }
    }

    if (root.contains("contours") && root.at("contours").is_array()) {
        for (const toml::value& e : root.at("contours").as_array()) {
            if (!e.is_table()) continue;
            LaserContour c;
            if (e.contains("contourId"))      c.contourId      = static_cast<std::uint64_t>(e.at("contourId").as_integer());
            if (e.contains("layerId"))        c.layerId        = static_cast<std::uint64_t>(e.at("layerId").as_integer());
            if (e.contains("signature"))      c.signature      = static_cast<std::uint64_t>(e.at("signature").as_integer());
            if (e.contains("name"))           c.name           = QString::fromStdString(e.at("name").as_string());
            if (e.contains("enabled"))        c.enabled        = e.at("enabled").as_boolean();
            if (e.contains("workpieceEntry")) c.workpieceEntry = QString::fromStdString(e.at("workpieceEntry").as_string());
            if (e.contains("sourceInfo"))     c.sourceInfo     = QString::fromStdString(e.at("sourceInfo").as_string());
            if (e.contains("contourType"))    c.contourType    = static_cast<int>(e.at("contourType").as_integer());
            if (e.contains("leadInLength"))   c.leadIn.length      = e.at("leadInLength").as_floating();
            if (e.contains("leadInNormalAngle")) c.leadIn.normalAngle = e.at("leadInNormalAngle").as_floating();
            if (e.contains("leadInValid"))    c.leadIn.valid       = e.at("leadInValid").as_boolean();
            if (c.leadIn.valid) {
                if (e.contains("leadInEntryX") && e.contains("leadInEntryY") && e.contains("leadInEntryZ")) {
                    c.leadIn.entryPoint = gp_Pnt(
                        e.at("leadInEntryX").as_floating(),
                        e.at("leadInEntryY").as_floating(),
                        e.at("leadInEntryZ").as_floating());
                }
                if (e.contains("leadInEntryParam")) c.leadIn.entryParam = e.at("leadInEntryParam").as_floating();
                if (e.contains("leadInEntryEdge"))  c.leadIn.entryEdgeIndex = static_cast<int>(e.at("leadInEntryEdge").as_integer());
            }

            auto it = pointsByContourId.find(c.contourId);
            if (it != pointsByContourId.end())
                c.points = std::move(it.value());
            toolpath.contours().push_back(std::move(c));
        }
    }

    // 4. 读 next id + signature 映射
    ContourId nextContour = 1;
    std::uint64_t nextLayer = 1;
    if (root.contains("nextContourId") && root.at("nextContourId").is_integer())
        nextContour = static_cast<ContourId>(root.at("nextContourId").as_integer());
    if (root.contains("nextLayerId") && root.at("nextLayerId").is_integer())
        nextLayer = static_cast<std::uint64_t>(root.at("nextLayerId").as_integer());

    QHash<std::uint64_t, std::uint64_t> sigContour;
    if (root.contains("signatureContours") && root.at("signatureContours").is_array()) {
        for (const toml::value& e : root.at("signatureContours").as_array()) {
            if (!e.is_table()) continue;
            if (!e.contains("sig") || !e.contains("id")) continue;
            sigContour.insert(static_cast<std::uint64_t>(e.at("sig").as_integer()),
                              static_cast<std::uint64_t>(e.at("id").as_integer()));
        }
    }
    QHash<std::uint64_t, std::uint64_t> sigLayer;
    if (root.contains("signatureLayers") && root.at("signatureLayers").is_array()) {
        for (const toml::value& e : root.at("signatureLayers").as_array()) {
            if (!e.is_table()) continue;
            if (!e.contains("sig") || !e.contains("id")) continue;
            sigLayer.insert(static_cast<std::uint64_t>(e.at("sig").as_integer()),
                            static_cast<std::uint64_t>(e.at("id").as_integer()));
        }
    }

    // 5. 灌进 CamDataManager（不做任何视图刷新，那是 CAM 模块职责）。
    cam.restoreSignatureTables(sigContour, sigLayer, nextContour, nextLayer);
    cam.replaceToolpath(std::move(toolpath), nextContour, nextLayer);

    // 恢复容器级 manual order / sortStrategy / lastAutoSortAxis。
    LayerContainer& container = cam.layerContainer();
    if (root.contains("manualContourOrder") && root.at("manualContourOrder").is_array()) {
        QVector<ContourId> manual;
        for (const toml::value& v : root.at("manualContourOrder").as_array()) {
            if (!v.is_integer()) continue;
            const auto cid = static_cast<ContourId>(v.as_integer());
            if (cid != 0) manual.append(cid);
        }
        container.setManualContourOrder(manual);
    }
    if (root.contains("sortStrategy") && root.at("sortStrategy").is_string()) {
        container.setSortStrategy(sortStrategyFromString(
            QString::fromStdString(root.at("sortStrategy").as_string()),
            CuttingPlanSortStrategy::LayerThenContour));
    }
    if (root.contains("lastAutoSortAxis") && root.at("lastAutoSortAxis").is_string()) {
        container.setLastAutoSortAxis(autoSortAxisFromString(
            QString::fromStdString(root.at("lastAutoSortAxis").as_string()), AutoSortAxis::XPos));
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: restored {} layers, {} contours from cache",
              cam.toolpath().layers().size(),
              cam.toolpath().contourCount());
    return true;
}

bool migrateLegacyProcessCuttingPlan(CamDataManager& cam, const QString& packageDir, QString* errorMsg)
{
    const QString filePath = QDir(packageDir).filePath(QString::fromLatin1(kLegacyPlanFile));
    if (!QFileInfo::exists(filePath))
        return true; // 新项目或已迁移项目：无需做任何事。

    toml::value root;
    try {
        root = toml::parse(filePath.toStdString());
    } catch (const std::exception& e) {
        if (errorMsg)
            *errorMsg = QStringLiteral("解析 process_cutting_plan.toml 失败: %1")
                            .arg(QString::fromLocal8Bit(e.what()));
        return false;
    }
    if (!root.is_table()) {
        if (errorMsg) *errorMsg = QStringLiteral("process_cutting_plan.toml 根节点不是 table");
        return false;
    }

    LayerContainer& container = cam.layerContainer();
    if (root.contains("sortStrategy") && root.at("sortStrategy").is_string()) {
        container.setSortStrategy(sortStrategyFromString(
            QString::fromStdString(root.at("sortStrategy").as_string()),
            CuttingPlanSortStrategy::LayerThenContour));
    }

    int migratedLayers = 0;
    if (root.contains("layers") && root.at("layers").is_array()) {
        for (const toml::value& entry : root.at("layers").as_array()) {
            if (!entry.is_table()) continue;
            std::uint64_t layerId = 0;
            if (entry.contains(kFieldLayerId) && entry.at(kFieldLayerId).is_integer())
                layerId = static_cast<std::uint64_t>(entry.at(kFieldLayerId).as_integer());
            if (layerId == 0) continue;

            if (entry.contains(kFieldToolName) && entry.at(kFieldToolName).is_string())
                container.setLayerToolName(layerId,
                    QString::fromStdString(entry.at(kFieldToolName).as_string()));
            if (entry.contains(kFieldEnabled) && entry.at(kFieldEnabled).is_boolean())
                container.setLayerEnabled(layerId, entry.at(kFieldEnabled).as_boolean());
            if (entry.contains(kFieldCompensation) && entry.at(kFieldCompensation).is_string())
                container.setLayerCompensationIndex(layerId,
                    QString::fromStdString(entry.at(kFieldCompensation).as_string()));
            if (entry.contains(kFieldIncludedContours) && entry.at(kFieldIncludedContours).is_array()) {
                QSet<ContourId> included;
                for (const toml::value& cid : entry.at(kFieldIncludedContours).as_array())
                    if (cid.is_integer())
                        included.insert(static_cast<ContourId>(cid.as_integer()));
                container.setLayerIncludedContours(layerId, included);
            }
            ++migratedLayers;
        }
    }

    if (root.contains(kFieldManualOrder) && root.at(kFieldManualOrder).is_array()) {
        QVector<ContourId> manual;
        for (const toml::value& v : root.at(kFieldManualOrder).as_array()) {
            if (!v.is_integer()) continue;
            const auto cid = static_cast<ContourId>(v.as_integer());
            if (cid != 0) manual.append(cid);
        }
        container.setManualContourOrder(manual);
    }
    if (root.contains(kFieldLastAxis) && root.at(kFieldLastAxis).is_string()) {
        container.setLastAutoSortAxis(autoSortAxisFromString(
            QString::fromStdString(root.at(kFieldLastAxis).as_string()), AutoSortAxis::XPos));
    }

    // 把旧文件改名以阻止下次再做迁移；新版保存路径不再产出该文件。
    const QString legacyPath = filePath + QStringLiteral(".legacy");
    QFile::remove(legacyPath);
    if (!QFile::rename(filePath, legacyPath)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "cam.toolpath: failed to rename legacy '{}' to '{}'",
                  filePath.toStdString(), legacyPath.toStdString());
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: migrated v1 process_cutting_plan.toml ({} layers)", migratedLayers);
    return true;
}

} // namespace lcnc::cam
