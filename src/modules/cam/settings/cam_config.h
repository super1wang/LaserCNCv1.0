#pragma once

#include "core/settings/app_settings.h"
#include "core/settings/toml_config.h"

#include <QMap>
#include <QString>

#include <gp_Pnt.hxx>

/**
 * @brief CAM 模块持久化配置（cam.toml）。
 *
 * 字段：
 *   - 全局：machineModelPath / autoLoadMachineModel / machinePreset / machineRenderQualityPreset
 *   - [toolpath]：leadInLength / deflection / smoothAngle /
 *     useFaceClassification / showNormals / normalSampleStep
 *   - machineProfile（按机台 absolute 路径分组，array of tables）：
 *     axisOrigins、cutterHeadModelPosition、cutterHeadPhysicalPosition、
 *     workpieceInstallPosition
 *
 * 由 CamModule 在初始化时构造并通过 @ref loadDefault 装载。
 * 旧版 JSON（CamConfig.json）会在首次加载时自动迁移：解析 JSON、写入
 * 同目录下 cam.toml，并将原 JSON 重命名为 CamConfig.json.bak。
 *
 * 注：本类不再是单例。进程内通过
 * `lcnc::Kernel::current().service<CamModule>()->config()` 访问。
 */
class CamConfig : public lcnc::TomlConfig
{
public:
    CamConfig() = default;

    /// 装载 <exeDir>/config/cam.toml；若不存在但旧 CamConfig.json 存在则迁移。
    bool loadDefault();

    /// 保存到 loadDefault() 使用的同一路径。
    bool saveDefault() const;

    QString machineModelPath() const { return m_machineModelPath; }
    void setMachineModelPath(const QString& path);

    bool autoLoadMachineModel() const { return m_autoLoadMachineModel; }
    void setAutoLoadMachineModel(bool enabled);

    QString machinePreset() const { return m_machinePreset; }
    void setMachinePreset(const QString& preset);

    lcnc::RenderQualityPreset machineRenderQualityPreset() const { return m_machineRenderQualityPreset; }
    void setMachineRenderQualityPreset(lcnc::RenderQualityPreset quality);

    bool autoInstallWorkpiece() const { return m_autoInstallWorkpiece; }
    void setAutoInstallWorkpiece(bool enabled);

    double leadInLength() const { return m_leadInLength; }
    void setLeadInLength(double mm);

    double deflection() const { return m_deflection; }
    void setDeflection(double mm);

    double smoothAngle() const { return m_smoothAngle; }
    void setSmoothAngle(double deg);

    bool useFaceClassification() const { return m_useFaceClassification; }
    void setUseFaceClassification(bool enabled);

    bool showNormals() const { return m_showNormals; }
    void setShowNormals(bool enabled);

    double normalSampleStep() const { return m_normalSampleStep; }
    void setNormalSampleStep(double mm);

    bool axisOriginForMachine(const QString& machinePath,
                              const QString& axisName,
                              gp_Pnt* outOrigin) const;
    void setAxisOriginForMachine(const QString& machinePath,
                                 const QString& axisName,
                                 const gp_Pnt& origin);

    bool cutterHeadModelPositionForMachine(const QString& machinePath,
                                           gp_Pnt* outPosition) const;
    void setCutterHeadModelPositionForMachine(const QString& machinePath,
                                              const gp_Pnt& position);

    bool cutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                              gp_Pnt* outPosition) const;
    void setCutterHeadPhysicalPositionForMachine(const QString& machinePath,
                                                 const gp_Pnt& position);

    bool workpieceInstallPositionForMachine(const QString& machinePath,
                                            gp_Pnt* outPosition) const;
    void setWorkpieceInstallPositionForMachine(const QString& machinePath,
                                               const gp_Pnt& position);

    /// 标定位的物理 A/C 角度（度）。已记录返回 true，否则保持 outA/outC 不变。
    bool acAngleOffsetForMachine(const QString& machinePath,
                                 double* outA,
                                 double* outC) const;
    /// 写入标定位对应的物理 A/C 角度（度），自动持久化。
    void setAcAngleOffsetForMachine(const QString& machinePath,
                                    double aAngle,
                                    double cAngle);

    /// 用户在向导中输入的物理 AC 中心 XYZ（mm）。已记录返回 true。
    bool physicalAcCenterForMachine(const QString& machinePath,
                                    gp_Pnt* outCenter) const;
    /// 写入物理 AC 中心 XYZ（mm），自动持久化；用于向导回显。
    void setPhysicalAcCenterForMachine(const QString& machinePath,
                                       const gp_Pnt& center);

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "CamConfig"; }

private:
    struct MachineProfile {
        QMap<QString, gp_Pnt> axisOrigins;
        bool hasCutterHeadModel{false};
        gp_Pnt cutterHeadModelPosition;
        bool hasCutterHeadPhysical{false};
        gp_Pnt cutterHeadPhysicalPosition;
        bool hasWorkpieceInstallPosition{false};
        gp_Pnt workpieceInstallPosition;
        bool hasAcAngleOffset{false};
        double acAngleOffsetA{0.0};
        double acAngleOffsetC{0.0};
        bool hasPhysicalAcCenter{false};
        gp_Pnt physicalAcCenter{};
    };

    static QString configDirectoryPath();
    static QString tomlFilePath();
    static QString legacyJsonPath();
    static QString machineKey(const QString& machinePath);

    /// 解析旧 JSON 内容到当前对象。
    bool importLegacyJson(const QString& jsonPath);

    MachineProfile* mutableProfileForMachine(const QString& machinePath);
    const MachineProfile* profileForMachine(const QString& machinePath) const;

    QString m_machineModelPath;
    bool m_autoLoadMachineModel{true};
    QString m_machinePreset;
    lcnc::RenderQualityPreset m_machineRenderQualityPreset{lcnc::RenderQualityPreset::Medium};
    bool m_autoInstallWorkpiece{true};
    double m_leadInLength{5.0};
    double m_deflection{0.1};
    double m_smoothAngle{5.0};
    bool m_useFaceClassification{true};
    bool m_showNormals{false};
    double m_normalSampleStep{2.0};

    QMap<QString, MachineProfile> m_machineProfiles;
};
