#include "core/project/lcnc_project_manifest.h"

#include <QDir>
#include <QFileInfo>

namespace lcnc {

bool LcncProjectManifest::validate(QString* errorMsg, bool allowLegacyFormat) const
{
    if (schema != QStringLiteral("lcnc.project")) {
        if (errorMsg)
            *errorMsg = QStringLiteral("不支持的项目 schema: %1").arg(schema);
        return false;
    }
    if (formatVersion <= 0 || formatVersion > kCurrentFormatVersion) {
        if (errorMsg)
            *errorMsg = QStringLiteral("不支持的项目版本: %1").arg(formatVersion);
        return false;
    }
    if (!allowLegacyFormat && formatVersion != kCurrentFormatVersion) {
        if (errorMsg) {
            *errorMsg = QStringLiteral("项目版本 %1 必须先使用 lcnc_project_upgrade 迁移到 v%2")
                            .arg(formatVersion)
                            .arg(kCurrentFormatVersion);
        }
        return false;
    }
    // v2: 校验工件 XBF；v1: 校验旧 projectXcafPath
    const QString xbf = formatVersion >= 2 ? workpieceXcafPath : projectXcafPath;
    if (xbf.trimmed().isEmpty()) {
        if (errorMsg)
            *errorMsg = QStringLiteral("项目 manifest 缺少 XBF 资源路径");
        return false;
    }
    if (formatVersion >= 4) {
        const QString normalizedSnapshot = QDir::cleanPath(toolSnapshotPath.trimmed());
        if (normalizedSnapshot.isEmpty() || QFileInfo(normalizedSnapshot).isAbsolute()
            || normalizedSnapshot == QStringLiteral("..")
            || normalizedSnapshot.startsWith(QStringLiteral("../"))) {
            if (errorMsg)
                *errorMsg = QStringLiteral("项目 manifest 包含无效的工具快照路径");
            return false;
        }
    }
    return true;
}

void LcncProjectManifest::readFrom(const toml::value& root)
{
    using namespace toml_io;

    schema = get_qstring(root, "schema", QStringLiteral("lcnc.project"));
    formatVersion = get_int(root, "formatVersion", kCurrentFormatVersion);
    projectName = get_qstring(root, "projectName", QString());
    documentName = get_qstring(root, "documentName", projectName);
    sourceFilePath = get_qstring(root, "sourceFilePath", QString());
    createdUtc = get_qstring(root, "createdUtc", QString());
    savedUtc = get_qstring(root, "savedUtc", QString());
    softwareVersion = get_qstring(root, "softwareVersion", QString());
    machineConfigurationFingerprint = get_qstring(root, "machineConfigurationFingerprint", QString());
    configurationSchemaVersion = get_qstring(root, "configurationSchemaVersion", QString());
    toolpathAlgorithmVersion = get_qstring(root, "toolpathAlgorithmVersion", QString());

    if (root.contains("resources") && root.at("resources").is_table()) {
        const auto& resources = root.at("resources");
        // v2: 新项目只用 workpiece.xbf；v1: 读旧 projectXcaf 兜底
        projectXcafPath = get_qstring(resources, "projectXcaf", QStringLiteral("project.xbf"));
        workpieceXcafPath = get_qstring(resources, "workpieceXcaf", projectXcafPath);
        camCacheDirectory = get_qstring(resources, "camCacheDirectory", QStringLiteral("cam/cache"));
        toolSnapshotPath = get_qstring(resources, "toolSnapshot", QStringLiteral("tools.toml"));
    } else if (formatVersion < 2) {
        // v1 旧格式：resources 段不存在时回退默认值
        projectXcafPath = QStringLiteral("project.xbf");
        workpieceXcafPath = projectXcafPath;
    } else {
        workpieceXcafPath = QStringLiteral("workpiece.xbf");
    }

    if (root.contains("saveOptions") && root.at("saveOptions").is_table()) {
        const auto& options = root.at("saveOptions");
        saveOptions.includeWorkpieceModel = get_bool(options, "workpieceModel", true);
        saveOptions.includeMachineModel = get_bool(options, "machineModel", false);
        saveOptions.includeParameters = get_bool(options, "parameters", true);
        saveOptions.includeCamData = get_bool(options, "camData", true);
        saveOptions.includeCamCache = get_bool(options, "camCache", false);
    }
}

void LcncProjectManifest::writeTo(toml::value& root) const
{
    using namespace toml_io;

    root["schema"] = qs(schema);
    root["formatVersion"] = kCurrentFormatVersion;
    root["projectName"] = qs(projectName);
    root["documentName"] = qs(documentName);
    root["sourceFilePath"] = qs(sourceFilePath);
    root["createdUtc"] = qs(createdUtc);
    root["savedUtc"] = qs(savedUtc);
    root["softwareVersion"] = qs(softwareVersion);
    root["machineConfigurationFingerprint"] = qs(machineConfigurationFingerprint);
    root["configurationSchemaVersion"] = qs(configurationSchemaVersion);
    root["toolpathAlgorithmVersion"] = qs(toolpathAlgorithmVersion);

    toml::value resources(toml::table{});
    resources["workpieceXcaf"] = qs(workpieceXcafPath);
    resources["camCacheDirectory"] = qs(camCacheDirectory);
    resources["toolSnapshot"] = qs(toolSnapshotPath);
    root["resources"] = resources;

    toml::value options(toml::table{});
    options["workpieceModel"] = saveOptions.includeWorkpieceModel;
    options["machineModel"] = saveOptions.includeMachineModel;
    options["parameters"] = saveOptions.includeParameters;
    options["camData"] = saveOptions.includeCamData;
    options["camCache"] = saveOptions.includeCamCache;
    root["saveOptions"] = options;
}

} // namespace lcnc
