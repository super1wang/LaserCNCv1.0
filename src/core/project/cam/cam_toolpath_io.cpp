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
constexpr int  kCamToolpathSchemaVersion = 4;

// 二进制点集 magic 头（"LCNCTPT1"）。
constexpr quint64 kPointsBinMagic = 0x315450434E434C00ull;
constexpr quint32 kPointsBinVersion = 4;

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
        // 中文翻译：无法写入: %1
        if (errorMsg) *errorMsg = QStringLiteral("Unable to write: %1").arg(filePath);
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
            ds << p.crossSectionNormal.X()
               << p.crossSectionNormal.Y()
               << p.crossSectionNormal.Z();
            ds << static_cast<quint8>(p.crossSectionNormalValid ? 1 : 0);
            ds << p.tangent.X()  << p.tangent.Y()  << p.tangent.Z();
            ds << p.param << static_cast<qint32>(p.sourceEdgeIndex);
            ds << p.machineCoord.x << p.machineCoord.y << p.machineCoord.z;
            ds << p.machineCoord.r1 << p.machineCoord.r2;
            ds << p.machineCoord.r1Name << p.machineCoord.r2Name;
            ds << static_cast<quint8>(p.machineCoord.valid ? 1 : 0);
        }
        ds << static_cast<quint8>(c.leadInSolution.valid ? 1 : 0);
        if (c.leadInSolution.valid) {
            const ToolpathPoint& p = c.leadInSolution.point;
            ds << p.position.X() << p.position.Y() << p.position.Z();
            ds << p.normal.X()   << p.normal.Y()   << p.normal.Z();
            ds << p.crossSectionNormal.X()
               << p.crossSectionNormal.Y()
               << p.crossSectionNormal.Z();
            ds << static_cast<quint8>(p.crossSectionNormalValid ? 1 : 0);
            ds << p.tangent.X()  << p.tangent.Y()  << p.tangent.Z();
            ds << p.param << static_cast<qint32>(p.sourceEdgeIndex);
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
                   QHash<std::uint64_t, LeadInSolution>& leadInsByContourId,
                   QString* errorMsg)
{
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) {
        // 中文翻译：无法读取: %1
        if (errorMsg) *errorMsg = QStringLiteral("Unable to read: %1").arg(filePath);
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
        // 中文翻译：点集文件 magic 不匹配
        if (errorMsg) *errorMsg = QStringLiteral("Point set file magic mismatch");
        return false;
    }
    if (version != kPointsBinVersion) {
        // 中文翻译：点集文件版本 %1 不支持
        if (errorMsg) *errorMsg = QStringLiteral("Point set file version %1 is not supported").arg(version);
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
            double cnx = 0.0, cny = 0.0, cnz = 1.0;
            double mx, my, mz, r1, r2;
            QString r1Name, r2Name;
            quint8 valid = 0;
            quint8 crossValid = 0;
            qint32 sourceEdgeIndex = -1;
            ds >> px >> py >> pz >> nx >> ny >> nz;
            ds >> cnx >> cny >> cnz >> crossValid;
            ds >> tx >> ty >> tz >> par;
            ds >> sourceEdgeIndex;
            ds >> mx >> my >> mz >> r1 >> r2 >> r1Name >> r2Name >> valid;
            ToolpathPoint tp;
            tp.position = gp_Pnt(px, py, pz);
            tp.normal   = gp_Dir(nx, ny, nz);
            tp.crossSectionNormal = gp_Dir(cnx, cny, cnz);
            tp.crossSectionNormalValid = (crossValid != 0);
            tp.tangent  = gp_Dir(tx, ty, tz);
            tp.param    = par;
            tp.sourceEdgeIndex = sourceEdgeIndex;
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

        quint8 leadValid = 0;
        ds >> leadValid;
        if (leadValid != 0) {
                double px, py, pz, nx, ny, nz, tx, ty, tz, par;
                double cnx, cny, cnz, mx, my, mz, r1, r2;
                QString r1Name, r2Name;
                quint8 crossValid = 0, machineValid = 0;
                qint32 sourceEdgeIndex = -1;
                ds >> px >> py >> pz >> nx >> ny >> nz;
                ds >> cnx >> cny >> cnz >> crossValid;
                ds >> tx >> ty >> tz >> par;
                ds >> sourceEdgeIndex;
                ds >> mx >> my >> mz >> r1 >> r2 >> r1Name >> r2Name >> machineValid;
                LeadInSolution solution;
                solution.point.position = gp_Pnt(px, py, pz);
                solution.point.normal = gp_Dir(nx, ny, nz);
                solution.point.crossSectionNormal = gp_Dir(cnx, cny, cnz);
                solution.point.crossSectionNormalValid = (crossValid != 0);
                solution.point.tangent = gp_Dir(tx, ty, tz);
                solution.point.param = par;
                solution.point.sourceEdgeIndex = sourceEdgeIndex;
                solution.point.machineCoord.x = mx;
                solution.point.machineCoord.y = my;
                solution.point.machineCoord.z = mz;
                solution.point.machineCoord.r1 = r1;
                solution.point.machineCoord.r2 = r2;
                solution.point.machineCoord.r1Name = r1Name;
                solution.point.machineCoord.r2Name = r2Name;
                solution.point.machineCoord.valid = (machineValid != 0);
                solution.valid = true;
                leadInsByContourId.insert(static_cast<std::uint64_t>(contourId),
                                          std::move(solution));
        }
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
    const bool hasFaceStage = cam.pipelineStageState(CamPipelineStage::FaceSeparation).available
        || !cam.machiningFaceRecords().empty();
    if (tp.contourCount() == 0 && !hasFaceStage) {
        // No committed CAM stage → no cache to write.  A face-only project is
        // nevertheless meaningful and must persist so the user can reopen it
        // and continue at contour extraction.
        return true;
    }
    if (!QDir().mkpath(packageDir)) {
        // 中文翻译：无法创建目录: %1
        if (errorMsg) *errorMsg = QStringLiteral("Unable to create directory: %1").arg(packageDir);
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
        entry["xcafEntry"]      = c.xcafEntry.toStdString(); // v3: 关联工程文档内的 Cam 几何 label
        entry["name"]           = c.name.toStdString();
        entry["enabled"]        = c.enabled;
        entry["workpieceEntry"] = c.workpieceEntry.toStdString();
        entry["sourceInfo"]     = c.sourceInfo.toStdString();
        entry["contourType"]    = static_cast<std::int64_t>(c.contourType);
        entry["leadInLength"]      = c.appliedParams.leadInLength;
        entry["appliedLeadInLength"] = c.appliedParams.leadInLength;
        entry["appliedDeflection"] = c.appliedParams.deflection;
        entry["pendingLeadInLength"] = c.pendingParams.leadInLength;
        entry["pendingDeflection"] = c.pendingParams.deflection;
        entry["needsRecalculation"] = c.needsRecalculation;
        entry["leadInValid"]       = c.leadIn.valid;
        if (c.leadIn.valid) {
            entry["leadInEntryX"]    = c.leadIn.entryPoint.X();
            entry["leadInEntryY"]    = c.leadIn.entryPoint.Y();
            entry["leadInEntryZ"]    = c.leadIn.entryPoint.Z();
            entry["leadInEntryParam"]= c.leadIn.entryParam;
            entry["leadInEntryEdgeIndex"] = static_cast<std::int64_t>(c.leadIn.entryEdgeIndex);
            entry["leadInEntryPointIndex"] = static_cast<std::int64_t>(c.leadIn.entryPointIndex);
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

    // 工程级刀路生成参数（随工程持久化，保证重开可复现）。
    const CamDataManager::GenerationParams& gp = cam.generationParams();
    const CamDataManager::GenerationParams& appliedGp = cam.appliedGenerationParams();
    toml::value gen(toml::table{});
    gen["leadInLength"]         = gp.leadInLength;
    gen["deflection"]           = gp.deflection;
    gen["smoothAngle"]          = gp.smoothAngle;
    gen["useFaceClassification"] = gp.useFaceClassification;
    gen["extractionStrategy"]   = gp.extractionStrategy;
    gen["normalSampleStep"]     = gp.normalSampleStep;
    gen["dirty"]                = cam.generationParamsDirty();
    gen["appliedLeadInLength"]  = appliedGp.leadInLength;
    gen["appliedDeflection"]    = appliedGp.deflection;
    gen["appliedSmoothAngle"]   = appliedGp.smoothAngle;
    gen["appliedUseFaceClassification"] = appliedGp.useFaceClassification;
    gen["appliedExtractionStrategy"]   = appliedGp.extractionStrategy;
    root["generation"]          = gen;

    // Machining-face records (manual picks survive save/reload via signature).
    toml::array faceRecArr;
    for (const auto& rec : cam.machiningFaceRecords()) {
        toml::value entry(toml::table{});
        entry["faceId"]         = static_cast<std::int64_t>(rec.faceId);
        entry["signature"]      = static_cast<std::int64_t>(rec.signature);
        entry["workpieceEntry"] = rec.workpieceEntry.toStdString();
        entry["manual"]         = rec.manual;
        entry["role"]           = static_cast<int>(rec.role);
        faceRecArr.push_back(entry);
    }
    root["machiningFaces"] = faceRecArr;

    toml::array pipelineStages;
    for (int index = 0; index < static_cast<int>(CamPipelineStage::Count); ++index) {
        const CamPipelineStage stage = static_cast<CamPipelineStage>(index);
        const CamPipelineStageState& state = cam.pipelineStageState(stage);
        toml::value entry(toml::table{});
        entry["stage"] = index;
        entry["available"] = state.available;
        entry["dirty"] = state.dirty;
        entry["revision"] = static_cast<std::int64_t>(state.revision);
        entry["inputRevision"] = static_cast<std::int64_t>(state.inputRevision);
        entry["failureReason"] = state.failureReason.toStdString();
        pipelineStages.push_back(entry);
    }
    root["pipelineStages"] = pipelineStages;

    std::ofstream out(camToolpathTomlPath(packageDir).toStdString(), std::ios::binary);
    if (!out.is_open()) {
        // 中文翻译：无法写入 cam_toolpath.toml
        if (errorMsg) *errorMsg = QStringLiteral("Unable to write to cam_toolpath.toml");
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
        // 中文翻译：项目无刀路缓存
        if (errorMsg) *errorMsg = QStringLiteral("Project has no toolpath cache");
        return false;
    }

    // 1. 解析 toml
    toml::value root;
    try {
        root = toml::parse(tomlPath.toStdString());
    } catch (const std::exception& e) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.toolpath: failed to parse '{}': {}",
                 tomlPath.toStdString(),
                 e.what());
        // 中文翻译：解析 cam_toolpath.toml 失败: %1
        if (errorMsg) *errorMsg = QStringLiteral("Failed to parse cam_toolpath.toml: %1")
                                       .arg(QString::fromLocal8Bit(e.what()));
        return false;
    }
    if (!root.is_table()) {
        // 中文翻译：cam_toolpath.toml 根节点错误
        if (errorMsg) *errorMsg = QStringLiteral("cam_toolpath.toml root node error");
        return false;
    }
    const int schemaVersion = root.contains("schemaVersion") && root.at("schemaVersion").is_integer()
        ? static_cast<int>(root.at("schemaVersion").as_integer()) : 0;
    if (schemaVersion != kCamToolpathSchemaVersion) {
        if (errorMsg)
            *errorMsg = QStringLiteral("CAM toolpath schema version %1 is not supported; expected %2")
                            .arg(schemaVersion).arg(kCamToolpathSchemaVersion);
        return false;
    }

    // 2. 读点集（先于 LaserContour，确保按 contourId 关联）。
    QHash<std::uint64_t, std::vector<ToolpathPoint>> pointsByContourId;
    QHash<std::uint64_t, LeadInSolution> leadInsByContourId;
    if (!readPointsBin(pointsPath, pointsByContourId, leadInsByContourId, errorMsg))
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
            if (e.contains("xcafEntry"))      c.xcafEntry      = QString::fromStdString(e.at("xcafEntry").as_string());
            if (e.contains("name"))           c.name           = QString::fromStdString(e.at("name").as_string());
            if (e.contains("enabled"))        c.enabled        = e.at("enabled").as_boolean();
            if (e.contains("workpieceEntry")) c.workpieceEntry = QString::fromStdString(e.at("workpieceEntry").as_string());
            if (e.contains("sourceInfo"))     c.sourceInfo     = QString::fromStdString(e.at("sourceInfo").as_string());
            if (e.contains("contourType"))    c.contourType    = static_cast<int>(e.at("contourType").as_integer());
            if (e.contains("leadInLength"))   c.leadIn.length      = e.at("leadInLength").as_floating();
            c.appliedParams.leadInLength = e.contains("appliedLeadInLength")
                ? e.at("appliedLeadInLength").as_floating() : c.leadIn.length;
            c.appliedParams.deflection = e.contains("appliedDeflection")
                ? e.at("appliedDeflection").as_floating() : 0.0;
            c.pendingParams.leadInLength = e.contains("pendingLeadInLength")
                ? e.at("pendingLeadInLength").as_floating() : c.appliedParams.leadInLength;
            c.pendingParams.deflection = e.contains("pendingDeflection")
                ? e.at("pendingDeflection").as_floating() : c.appliedParams.deflection;
            c.needsRecalculation = e.contains("needsRecalculation")
                && e.at("needsRecalculation").as_boolean();
            c.leadIn.length = c.appliedParams.leadInLength;
            if (e.contains("leadInValid"))    c.leadIn.valid       = e.at("leadInValid").as_boolean();
            if (c.leadIn.valid) {
                if (e.contains("leadInEntryX") && e.contains("leadInEntryY") && e.contains("leadInEntryZ")) {
                    c.leadIn.entryPoint = gp_Pnt(
                        e.at("leadInEntryX").as_floating(),
                        e.at("leadInEntryY").as_floating(),
                        e.at("leadInEntryZ").as_floating());
                }
                if (e.contains("leadInEntryParam")) c.leadIn.entryParam = e.at("leadInEntryParam").as_floating();
                if (e.contains("leadInEntryEdgeIndex"))
                    c.leadIn.entryEdgeIndex = static_cast<int>(e.at("leadInEntryEdgeIndex").as_integer());
                if (e.contains("leadInEntryPointIndex"))
                    c.leadIn.entryPointIndex = static_cast<int>(e.at("leadInEntryPointIndex").as_integer());
                else
                    c.leadIn.entryPointIndex = 0;
            }

            auto it = pointsByContourId.find(c.contourId);
            if (it != pointsByContourId.end())
                c.points = std::move(it.value());
            auto leadIt = leadInsByContourId.find(c.contourId);
            if (leadIt != leadInsByContourId.end()) {
                c.leadInSolution = std::move(leadIt.value());
                if (!c.points.empty()) {
                    gp_Vec direction(c.points.front().position,
                                     c.leadInSolution.point.position);
                    if (direction.Magnitude() > 1e-9)
                        c.leadInSolution.direction = gp_Dir(direction);
                }
            }
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

    // 工程级刀路生成参数。
    if (root.contains("generation") && root.at("generation").is_table()) {
        const toml::value& gen = root.at("generation");
        CamDataManager::GenerationParams gp = cam.generationParams();
        if (gen.contains("leadInLength"))         gp.leadInLength         = gen.at("leadInLength").as_floating();
        if (gen.contains("deflection"))           gp.deflection           = gen.at("deflection").as_floating();
        if (gen.contains("smoothAngle"))          gp.smoothAngle          = gen.at("smoothAngle").as_floating();
        if (gen.contains("useFaceClassification")) gp.useFaceClassification = gen.at("useFaceClassification").as_boolean();
        if (gen.contains("extractionStrategy"))    gp.extractionStrategy = static_cast<int>(gen.at("extractionStrategy").as_integer());
        if (gen.contains("normalSampleStep"))     gp.normalSampleStep     = gen.at("normalSampleStep").as_floating();
        cam.generationParams() = gp;
        CamDataManager::GenerationParams applied = gp;
        if (gen.contains("appliedLeadInLength")) applied.leadInLength = gen.at("appliedLeadInLength").as_floating();
        if (gen.contains("appliedDeflection")) applied.deflection = gen.at("appliedDeflection").as_floating();
        if (gen.contains("appliedSmoothAngle")) applied.smoothAngle = gen.at("appliedSmoothAngle").as_floating();
        if (gen.contains("appliedUseFaceClassification"))
            applied.useFaceClassification = gen.at("appliedUseFaceClassification").as_boolean();
        if (gen.contains("appliedExtractionStrategy"))
            applied.extractionStrategy = static_cast<int>(gen.at("appliedExtractionStrategy").as_integer());
        cam.appliedGenerationParams() = applied;
        cam.setGenerationParamsDirty(gen.contains("dirty") && gen.at("dirty").as_boolean());
    }

    // Machining-face records (manual picks bound by signature on next generate).
    if (root.contains("machiningFaces") && root.at("machiningFaces").is_array()) {
        std::vector<CamDataManager::MachiningFaceRecord> faceRecords;
        for (const toml::value& e : root.at("machiningFaces").as_array()) {
            if (!e.is_table()) continue;
            CamDataManager::MachiningFaceRecord rec;
            if (e.contains("faceId"))         rec.faceId         = static_cast<std::uint64_t>(e.at("faceId").as_integer());
            if (e.contains("signature"))      rec.signature      = static_cast<std::uint64_t>(e.at("signature").as_integer());
            if (e.contains("workpieceEntry")) rec.workpieceEntry = QString::fromStdString(e.at("workpieceEntry").as_string());
            if (e.contains("manual"))         rec.manual         = e.at("manual").as_boolean();
            if (e.contains("role")) {
                const int storedRole = static_cast<int>(e.at("role").as_integer());
                rec.role = static_cast<MachiningFaceRole>(storedRole);
            }
            faceRecords.push_back(rec);
        }
        cam.setMachiningFaceRecords(std::move(faceRecords));
    }

    if (root.contains("pipelineStages") && root.at("pipelineStages").is_array()) {
        for (const toml::value& e : root.at("pipelineStages").as_array()) {
            if (!e.is_table() || !e.contains("stage"))
                continue;
            const int index = static_cast<int>(e.at("stage").as_integer());
            if (index < 0 || index >= static_cast<int>(CamPipelineStage::Count))
                continue;
            CamPipelineStageState state;
            if (e.contains("available")) state.available = e.at("available").as_boolean();
            if (e.contains("dirty")) state.dirty = e.at("dirty").as_boolean();
            if (e.contains("revision")) state.revision = static_cast<std::uint64_t>(e.at("revision").as_integer());
            if (e.contains("inputRevision")) state.inputRevision = static_cast<std::uint64_t>(e.at("inputRevision").as_integer());
            if (e.contains("failureReason")) state.failureReason = QString::fromStdString(e.at("failureReason").as_string());
            cam.restorePipelineStageState(static_cast<CamPipelineStage>(index), state);
        }
    }

    for (LaserContour& contour : cam.toolpath().contours()) {
        if (contour.appliedParams.deflection <= 0.0)
            contour.appliedParams.deflection = cam.appliedGenerationParams().deflection;
        if (contour.pendingParams.deflection <= 0.0)
            contour.pendingParams.deflection = contour.appliedParams.deflection;
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "cam.toolpath: restored {} layers, {} contours from cache",
              cam.toolpath().layers().size(),
              cam.toolpath().contourCount());
    return true;
}

} // namespace lcnc::cam
