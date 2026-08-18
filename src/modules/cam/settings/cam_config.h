#pragma once

#include "core/settings/app_settings.h"
#include "core/settings/toml_config.h"

#include <QMap>
#include <QSet>
#include <QString>

#include <gp_Pnt.hxx>

enum class CutterCollisionProxyMode : int
{
    SimulatedCone = 0,
    ModelFile = 1
};

/**
 * @brief CAM 模块持久化配置（cam.toml）。
 *
 * 字段：
 *   - 全局：machineModelPath / autoLoadMachineModel / machinePreset / machineRenderQualityPreset
 *   - [toolpath]：leadInLength / deflection / smoothAngle /
 *     useFaceClassification / showNormals / normalSampleStep
 *   - machineProfile（按机台 absolute 路径分组，array of tables）：
 *     axisOrigins、cutterHeadModelPosition、cutterHeadPhysicalPosition、
 *     legacy workpieceInstallPosition (read only for one-time migration)
 *
 * 由 CamModule 在初始化时构造并通过 @ref loadDefault 装载。
 * Only the current TOML configuration is accepted.
 *
 * 注：本类不再是单例。进程内通过
 * `lcnc::Kernel::current().service<CamModule>()->config()` 访问。
 */
class CamConfig : public lcnc::TomlConfig
{
public:
    CamConfig() = default;

    /// 装载 <exeDir>/config/cam.toml。
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

    CutterCollisionProxyMode cutterCollisionProxyMode() const { return m_cutterCollisionProxyMode; }
    void setCutterCollisionProxyMode(CutterCollisionProxyMode mode);

    QString cutterNozzleModelPath() const { return m_cutterNozzleModelPath; }
    void setCutterNozzleModelPath(const QString& path);

    double simulatedConeLengthMm() const { return m_simulatedConeLengthMm; }
    void setSimulatedConeLengthMm(double value);
    double simulatedConeTipRadiusMm() const { return m_simulatedConeTipRadiusMm; }
    void setSimulatedConeTipRadiusMm(double value);
    double simulatedConeBaseRadiusMm() const { return m_simulatedConeBaseRadiusMm; }
    void setSimulatedConeBaseRadiusMm(double value);
    double cutterCollisionClearanceMm() const { return m_cutterCollisionClearanceMm; }
    void setCutterCollisionClearanceMm(double value);
    double maximumRapidSafetyOffsetMm() const { return m_maximumRapidSafetyOffsetMm; }
    bool blockMachiningOnCollisionWarning() const { return m_blockMachiningOnCollisionWarning; }
    void setBlockMachiningOnCollisionWarning(bool enabled);
    void setMaximumRapidSafetyOffsetMm(double value);

    double leadInLength() const { return m_leadInLength; }
    void setLeadInLength(double mm);

    double deflection() const { return m_deflection; }
    void setDeflection(double mm);

    double cuttingOffsetMm() const { return m_cuttingOffsetMm; }
    void setCuttingOffsetMm(double mm);
    double rapidOffsetMm() const { return m_rapidOffsetMm; }
    void setRapidOffsetMm(double mm);

    double smoothAngle() const { return m_smoothAngle; }
    void setSmoothAngle(double deg);

    bool useFaceClassification() const { return m_useFaceClassification; }
    void setUseFaceClassification(bool enabled);

    int  extractionStrategy() const { return m_extractionStrategy; }
    void setExtractionStrategy(int strategy);

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

    /// Collision sources are persisted per machine profile.  An empty path
    /// addresses the dedicated no-machine (virtual cutter/workpiece) profile.
    bool collisionDetectionEnabledForMachine(const QString& machinePath) const;
    void setCollisionDetectionEnabledForMachine(const QString& machinePath, bool enabled);
    QSet<QString> activeCollisionSourcesForMachine(const QString& machinePath) const;
    QSet<QString> passiveCollisionSourcesForMachine(const QString& machinePath) const;
    void setCollisionSourcesForMachine(const QString& machinePath,
                                       const QSet<QString>& active,
                                       const QSet<QString>& passive);

    /// Legacy v4 machine-profile value.  It is accepted only to migrate XYZ
    /// into MachineConfigurationService::WorkpieceSetupTransform and is never
    /// written back to cam.toml.
    bool workpieceInstallPositionForMachine(const QString& machinePath,
                                            gp_Pnt* outPosition) const;
    void clearLegacyWorkpieceInstallPositionForMachine(const QString& machinePath);

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
        bool collisionDetectionEnabled{false};
        QSet<QString> activeCollisionSources{QStringLiteral("cutter")};
        QSet<QString> passiveCollisionSources{QStringLiteral("workpiece")};
        bool hasWorkpieceInstallPosition{false}; // legacy input only
        gp_Pnt workpieceInstallPosition;
        bool hasAcAngleOffset{false};
        double acAngleOffsetA{0.0};
        double acAngleOffsetC{0.0};
        bool hasPhysicalAcCenter{false};
        gp_Pnt physicalAcCenter{};
    };

    static QString configDirectoryPath();
    static QString tomlFilePath();
    static QString machineKey(const QString& machinePath);

    MachineProfile* mutableProfileForMachine(const QString& machinePath);
    const MachineProfile* profileForMachine(const QString& machinePath) const;

    QString m_machineModelPath;
    bool m_autoLoadMachineModel{true};
    QString m_machinePreset;
    lcnc::RenderQualityPreset m_machineRenderQualityPreset{lcnc::RenderQualityPreset::Medium};
    bool m_autoInstallWorkpiece{true};
    CutterCollisionProxyMode m_cutterCollisionProxyMode{CutterCollisionProxyMode::SimulatedCone};
    QString m_cutterNozzleModelPath;
    double m_simulatedConeLengthMm{20.0};
    double m_simulatedConeTipRadiusMm{0.2};
    double m_simulatedConeBaseRadiusMm{5.0};
    double m_cutterCollisionClearanceMm{0.5};
    double m_maximumRapidSafetyOffsetMm{100.0};
    bool m_blockMachiningOnCollisionWarning{true};
    double m_leadInLength{5.0};
    double m_deflection{0.1};
    double m_cuttingOffsetMm{1.0};
    double m_rapidOffsetMm{5.0};
    double m_smoothAngle{5.0};
    bool m_useFaceClassification{true};
    int m_extractionStrategy{0}; ///< ExtractionStrategy (LargestSmoothConnectedSurface)
    bool m_showNormals{false};
    double m_normalSampleStep{2.0};

    QMap<QString, MachineProfile> m_machineProfiles;
};
